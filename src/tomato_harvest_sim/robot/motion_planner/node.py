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
    should_plan_on_snapshot_arrival,
    trigger_starts_planner,
)
from tomato_harvest_sim.robot.motion_planner.state_aggregation import (
    PlannerStateAggregator,
)
from tomato_harvest_sim.robot.motion_planner.phase_suffix_replan import (
    PHASE_ENTRY_PLANNING_PHASES,
    PhasePlanningGate,
    evaluate_phase_plan_update,
    should_plan_phase_on_entry,
)

# ABORT/STALL/SCENE_CHANGE trigger の再評価を /trajectory_status の受信のみに
# 依存させない (Issue #53)。servo_execution_adapter は deadline abort後に
# lifecycle target を clear し、以後の control tick で status を一切
# publish しなくなるため、_on_trajectory_status 起点の評価だけでは
# minimum_interval に一度でも阻まれた abort が二度と再評価されず、
# 復旧不能なデッドロックになる。この周期timerが取りこぼしを拾い直す。
_REPLAN_TRIGGER_POLL_INTERVAL_SEC = 0.3
_ROBOT_BASE_FRAME_ID = "panda_link0"


def main() -> None:
    import json
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
            self._planner = build_planner()
            self._pub = self.create_publisher(String, HARVEST_MOTION_PLAN_TOPIC, 10)
            self._state = PlannerStateAggregator()
            self._trigger_memory = TriggerMemory()
            self._phase_planning_gate = PhasePlanningGate()
            self._latest_plan = None
            self._plan_revision = 0  # publish 済み plan の単調増加版数 (Step 1 契約)
            self._producer_instance_id = uuid.uuid4().hex

            self.create_subscription(String, PHASE_TOPIC, self._on_phase, 10)
            self.create_subscription(String, TARGET_ESTIMATE_TOPIC, self._on_estimate, 10)
            self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_state, 10)
            self.create_subscription(String, TRAJECTORY_STATUS_TOPIC, self._on_trajectory_status, 10)
            self.create_subscription(String, SCENE_SNAPSHOT_TOPIC, self._on_snapshot, 10)
            self.create_timer(
                _REPLAN_TRIGGER_POLL_INTERVAL_SEC, self._evaluate_replan_trigger
            )

        def _on_phase(self, msg: String) -> None:
            try:
                phase = HarvestTaskPhase(msg.data)
            except ValueError:
                return
            previous_phase = self._state.snapshot().phase
            self._state.update_phase(phase)
            if phase is HarvestTaskPhase.TARGET_FOUND:
                self._try_plan(trigger="target_found")
            if should_plan_phase_on_entry(previous_phase, phase):
                self._try_phase_plan(trigger="phase_entry")

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
                return
            # snapshot未着でtarget_found計画を保留した場合、到着時点で起動する
            # (Issue #37)。合成ゼロsceneでの計画はplace姿勢のゴミ化と
            # 実障害物未回避の軌道を生むため廃止した。
            if should_plan_on_snapshot_arrival(
                phase=self._state.snapshot().phase,
                has_plan=self._latest_plan is not None,
            ):
                self._try_plan(trigger="scene_snapshot_ready")

        def _on_trajectory_status(self, msg: String) -> None:
            try:
                status = json.loads(msg.data)
            except (json.JSONDecodeError, TypeError):
                status = {"status": msg.data.strip()}
            if not isinstance(status, dict):
                status = {"status": str(status)}
            tracking_error = status.get("tracking_error_rad")
            if tracking_error is not None:
                self._state.observe_tracking_error(float(tracking_error))
            self._state.observe_stall(bool(status.get("stalled", False)))
            if str(status.get("status", "")).strip() == "aborted":
                self._state.observe_abort()
                phase = self._state.snapshot().phase
                # 実行系由来のabort診断 (Issue #32)。最大追従誤差・律速joint・
                # abort分類を同じイベントに載せ、abort原因を後追い可能にする。
                self.get_logger().info(metric_line(
                    "phase_abort_observed",
                    phase=phase.value if phase is not None else "unknown",
                    max_joint_error_rad=status.get("max_joint_error_rad"),
                    limiting_joint=status.get("limiting_joint"),
                    limiting_joint_desired_rad=status.get("limiting_joint_desired_rad"),
                    limiting_joint_actual_rad=status.get("limiting_joint_actual_rad"),
                    abort_reason=status.get("abort_reason"),
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
                    action="observe_only_under_servo",
                ))
                return
            self._try_phase_plan(trigger=decision.trigger.value)

        def _try_phase_plan(self, *, trigger: str) -> None:
            state = self._state.snapshot()
            if (
                state.phase not in PHASE_ENTRY_PLANNING_PHASES
                or state.joint_state is None
                or state.scene_snapshot is None
                or self._latest_plan is None
            ):
                return
            phase = state.phase
            completion_event = (
                "phase_plan_completed"
                if trigger == "phase_entry"
                else "suffix_replan_completed"
            )
            if not self._phase_planning_gate.try_begin():
                self.get_logger().info(metric_line(
                    "phase_plan_suppressed", phase=phase.value,
                    reason="planner_in_flight", trigger=trigger,
                ))
                return
            started_at = time.perf_counter()
            try:
                candidate = self._planner.plan_phase_trajectory(
                    phase,
                    self._latest_plan,
                    state.joint_state,
                    _ROBOT_BASE_FRAME_ID,
                    state.scene_snapshot,
                )
                latency_ms = (time.perf_counter() - started_at) * 1000.0
                if candidate is None:
                    self.get_logger().info(metric_line(
                        completion_event, phase=phase.value,
                        success=False, trigger=trigger,
                        latency_ms=round(latency_ms, 3),
                    ))
                    return
                decision = evaluate_phase_plan_update(
                    phase=phase,
                    current_plan=self._latest_plan,
                    candidate_plan=candidate,
                )
                self.get_logger().info(metric_line(
                    completion_event,
                    phase=phase.value,
                    success=decision.adopted,
                    reason=decision.reason,
                    trigger=trigger,
                    latency_ms=round(latency_ms, 3),
                    max_trajectory_delta_rad=decision.max_trajectory_delta_rad,
                ))
                if not decision.adopted:
                    return
                self._publish_plan(candidate, trigger=trigger, phase=phase)
            finally:
                self._phase_planning_gate.finish()

        def _try_plan(self, *, trigger: str) -> None:
            state = self._state.snapshot()
            if state.target_estimate is None:
                return
            # snapshot未着なら計画しない (Issue #37)。合成ゼロsceneでの計画は
            # tray依存のplace姿勢をゴミ化し (goal sampling失敗 99999)、実際の
            # 枝・茎を回避しない軌道で物理固着を誘発していた。到着時に
            # _on_snapshot が scene_snapshot_ready trigger で再起動する。
            if state.scene_snapshot is None:
                self.get_logger().info(metric_line(
                    "planner_deferred",
                    trigger=trigger,
                    reason="scene_snapshot_not_ready",
                ))
                return

            scene_snapshot = state.scene_snapshot
            phase = state.phase.value if state.phase is not None else "unknown"
            started_at = time.perf_counter()
            try:
                plan = self._planner.plan(
                    state.target_estimate,
                    scene_snapshot,
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
                planner_name=f"{plan.planner_name}:{trigger}",
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
