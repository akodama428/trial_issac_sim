"""trajectory_planner_node — フェーズ受信時に MoveIt2 GetMotionPlan を呼び出し harvest_motion_plan を publish する。

アーキテクチャ仕様: docs/index.html §trajectory_planner_node
"""
from __future__ import annotations

import time

from tomato_harvest_sim.robot.motion_planner.observability import metric_line
from tomato_harvest_sim.robot.motion_planner.replan_trigger import (
    TriggerMemory,
    evaluate_replan_trigger,
    memory_after_trigger,
    should_inject_place_replan,
    trigger_starts_planner,
)
from tomato_harvest_sim.robot.motion_planner.state_aggregation import (
    PlannerStateAggregator,
)
from tomato_harvest_sim.robot.motion_planner.place_suffix_replan import (
    PlaceSuffixReplanGate,
    evaluate_place_suffix_update,
)


def main() -> None:
    import json
    import os
    import uuid
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    from dataclasses import replace

    from tomato_harvest_sim.msg.contracts import (
        HarvestTaskPhase,
        JointStateSnapshot,
        PlanProducerKind,
        TargetEstimate,
    )
    from tomato_harvest_sim.msg.topics import (
        HARVEST_MOTION_PLAN_TOPIC,
        JOINT_STATES_TOPIC,
        PHASE_TOPIC,
        SCENE_SNAPSHOT_TOPIC,
        TARGET_ESTIMATE_TOPIC,
        TRAJECTORY_STATUS_TOPIC,
    )
    from tomato_harvest_sim.msg.serialization import (
        harvest_motion_plan_to_json,
        scene_snapshot_from_dict,
        target_estimate_from_json,
    )
    from tomato_harvest_sim.robot.motion_planner import build_planner

    rclpy.init()

    class TrajectoryPlannerNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("trajectory_planner_node")
            planner, info = build_planner()
            self._planner = planner
            self._pub = self.create_publisher(String, HARVEST_MOTION_PLAN_TOPIC, 10)
            self._state = PlannerStateAggregator()
            self._trigger_memory = TriggerMemory()
            self._place_replan_gate = PlaceSuffixReplanGate()
            self._latest_plan = None
            self._inject_place_replan = (
                os.environ.get("TOMATO_HARVEST_INJECT_PLACE_REPLAN_ONCE", "0") == "1"
            )
            self._place_replan_injected = False
            self._plan_revision = 0  # publish 済み plan の単調増加版数 (Step 1 契約)
            self._producer_instance_id = uuid.uuid4().hex

            self.create_subscription(String, PHASE_TOPIC, self._on_phase, 10)
            self.create_subscription(String, TARGET_ESTIMATE_TOPIC, self._on_estimate, 10)
            self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_state, 10)
            self.create_subscription(String, TRAJECTORY_STATUS_TOPIC, self._on_trajectory_status, 10)
            self.create_subscription(String, SCENE_SNAPSHOT_TOPIC, self._on_snapshot, 10)
            self.create_timer(1.0, self._on_replan_timer)

        def _on_phase(self, msg: String) -> None:
            try:
                phase = HarvestTaskPhase(msg.data)
            except ValueError:
                return
            self._state.update_phase(phase)
            if phase is HarvestTaskPhase.TARGET_FOUND:
                self._try_plan(trigger="target_found")

        def _on_estimate(self, msg: String) -> None:
            self._state.update_target_estimate(target_estimate_from_json(msg.data))

        def _on_joint_state(self, msg: JointState) -> None:
            self._state.update_joint_state(JointStateSnapshot(
                joint_names=tuple(str(n) for n in msg.name),
                positions_rad=tuple(float(v) for v in msg.position),
            ))

        def _on_snapshot(self, msg: String) -> None:
            try:
                self._state.update_scene_snapshot(
                    scene_snapshot_from_dict(json.loads(msg.data))
                )
            except Exception:
                pass

        def _on_trajectory_status(self, msg: String) -> None:
            try:
                status = json.loads(msg.data)
            except (json.JSONDecodeError, TypeError):
                status = {"status": msg.data.strip()}
            if not isinstance(status, dict):
                status = {"status": str(status)}
            tracking_error = status.get("tracking_error_rad")
            self._state.update_tracking_error(
                float(tracking_error) if tracking_error is not None else None
            )
            if str(status.get("status", "")).strip() == "aborted":
                self._state.observe_abort()
                phase = self._state.snapshot().phase
                self.get_logger().info(metric_line(
                    "phase_abort_observed",
                    phase=phase.value if phase is not None else "unknown",
                ))
            self._evaluate_replan_trigger()

        def _on_replan_timer(self) -> None:
            state = self._state.snapshot()
            if should_inject_place_replan(
                enabled=self._inject_place_replan,
                already_injected=self._place_replan_injected,
                phase=state.phase,
            ):
                self._place_replan_injected = True
                self._state.update_tracking_error(0.20)
                self.get_logger().info(metric_line(
                    "place_suffix_e2e_disturbance_injected",
                    phase=state.phase.value,
                    tracking_error_rad=0.20,
                ))
            self._evaluate_replan_trigger()

        def _evaluate_replan_trigger(self) -> None:
            state = self._state.snapshot()
            now_sec = time.monotonic()
            decision = evaluate_replan_trigger(
                state=state, memory=self._trigger_memory, now_sec=now_sec
            )
            phase = state.phase.value if state.phase is not None else "unknown"
            self.get_logger().info(metric_line(
                "replan_trigger_evaluated",
                phase=phase,
                trigger=decision.trigger.value if decision.trigger is not None else "none",
                triggered=decision.triggered,
                reason=decision.reason,
            ))
            if not decision.triggered or decision.trigger is None:
                return
            self._trigger_memory = memory_after_trigger(
                state=state, memory=self._trigger_memory, now_sec=now_sec
            )
            if not trigger_starts_planner(decision.trigger, state.phase):
                self.get_logger().info(metric_line(
                    "replan_trigger_observed",
                    phase=phase,
                    trigger=decision.trigger.value,
                    action="observe_only_until_suffix_planning",
                ))
                return
            if state.phase is HarvestTaskPhase.MOVING_TO_PLACE:
                self._try_place_suffix_plan(trigger=decision.trigger.value)
                return
            self._try_plan(trigger=decision.trigger.value)

        def _try_place_suffix_plan(self, *, trigger: str) -> None:
            state = self._state.snapshot()
            if (
                state.phase is not HarvestTaskPhase.MOVING_TO_PLACE
                or state.joint_state is None
                or state.scene_snapshot is None
                or state.target_estimate is None
                or self._latest_plan is None
            ):
                return
            if not self._place_replan_gate.try_begin():
                self.get_logger().info(metric_line(
                    "place_suffix_replan_suppressed",
                    reason="planner_in_flight", trigger=trigger,
                ))
                return
            started_at = time.perf_counter()
            try:
                from tomato_harvest_sim.msg.contracts import Pose3D, TfTreeSnapshot
                zero = Pose3D(0, 0, 0, 0, 0, 0)
                tf_tree = TfTreeSnapshot(
                    robot_base_frame_id="panda_link0",
                    camera_frame_id="fixed_camera_frame",
                    target_frame_id="target_tomato_frame",
                    robot_base_pose=zero,
                    camera_pose=zero,
                    target_pose=state.target_estimate.target_world_pose,
                )
                plan_place = getattr(self._planner, "plan_place_from_joint_state", None)
                candidate = (
                    plan_place(
                        self._latest_plan,
                        state.joint_state,
                        tf_tree,
                        state.scene_snapshot,
                    )
                    if plan_place is not None else None
                )
                latency_ms = (time.perf_counter() - started_at) * 1000.0
                if candidate is None:
                    self.get_logger().info(metric_line(
                        "place_suffix_replan_completed", success=False,
                        trigger=trigger, latency_ms=round(latency_ms, 3),
                    ))
                    return
                decision = evaluate_place_suffix_update(
                    current_plan=self._latest_plan,
                    candidate_plan=candidate,
                )
                self.get_logger().info(metric_line(
                    "place_suffix_replan_completed",
                    success=decision.adopted,
                    reason=decision.reason,
                    trigger=trigger,
                    latency_ms=round(latency_ms, 3),
                    max_trajectory_delta_rad=decision.max_trajectory_delta_rad,
                ))
                if not decision.adopted:
                    return
                self._publish_plan(candidate, trigger=trigger, phase=state.phase)
            finally:
                if self._place_replan_injected:
                    self._state.update_tracking_error(None)
                self._place_replan_gate.finish()

        def _try_plan(self, *, trigger: str) -> None:
            state = self._state.snapshot()
            if state.target_estimate is None or state.joint_state is None:
                return

            from tomato_harvest_sim.msg.contracts import Pose3D, ScenePhase, SceneSnapshot, TomatoStatus
            _p = Pose3D(0, 0, 0, 0, 0, 0)

            # 実際の SceneSnapshot があればそれを使い、collision objects の配置を正確にする
            if state.scene_snapshot is not None:
                scene_snapshot = state.scene_snapshot
            else:
                scene_snapshot = SceneSnapshot(
                    phase=ScenePhase.RUNNING,
                    active_camera="fixed_camera",
                    tomato_attached=False,
                    tomato_status=TomatoStatus.ATTACHED,
                    gripper_closed=False,
                    robot_home=False,
                    cycle_id=0,
                    robot_model="panda",
                    robot_base_pose=_p, fixed_camera_pose=_p, hand_camera_pose=_p,
                    branch_pose=_p, stem_pose=_p, tomato_pose=_p, tray_pose=_p,
                    robot_tool_pose=_p, target_tool_pose=None, grasp_result_reason=None,
                )

            tf_tree_snapshot = type("T", (), {
                "robot_base_frame_id": "panda_link0",
                "camera_frame_id": "fixed_camera_frame",
                "target_frame_id": "target_tomato_frame",
                "robot_base_pose": _p,
                "camera_pose": _p,
                "target_pose": state.target_estimate.target_world_pose,
            })()
            phase = state.phase.value if state.phase is not None else "unknown"
            started_at = time.perf_counter()
            try:
                plan = self._planner.plan(
                    state.target_estimate, state.joint_state, tf_tree_snapshot, scene_snapshot
                )
            except Exception:
                latency_ms = (time.perf_counter() - started_at) * 1000.0
                self.get_logger().info(metric_line(
                    "planner_completed", phase=phase, trigger=trigger,
                    latency_ms=round(latency_ms, 3), success=False,
                ))
                raise
            latency_ms = (time.perf_counter() - started_at) * 1000.0
            self.get_logger().info(metric_line(
                "planner_completed", phase=phase, trigger=trigger,
                latency_ms=round(latency_ms, 3), success=plan is not None,
            ))
            if plan is None:
                return
            self._publish_plan(plan, trigger=trigger, phase=state.phase)

        def _publish_plan(
            self, plan: object, *, trigger: str, phase: HarvestTaskPhase | None
        ) -> None:
            from tomato_harvest_sim.msg.contracts import HarvestMotionPlan
            if not isinstance(plan, HarvestMotionPlan):
                return
            self._plan_revision += 1
            plan = replace(
                plan,
                plan_revision=self._plan_revision,
                generated_at_sec=time.time(),
                planned_from_phase=phase,
                producer_kind=PlanProducerKind.GLOBAL_PLANNER,
                producer_instance_id=self._producer_instance_id,
            )
            self.get_logger().info(metric_line(
                "plan_published",
                plan_revision=plan.plan_revision,
                planned_from_phase=phase.value if phase is not None else "unknown",
                producer_kind=plan.producer_kind.value,
                producer_instance_id=plan.producer_instance_id,
                trigger=trigger,
            ))
            out = String()
            out.data = harvest_motion_plan_to_json(plan)
            self._pub.publish(out)
            self._latest_plan = plan

    node = TrajectoryPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
