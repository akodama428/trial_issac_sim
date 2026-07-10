"""trajectory_planner_node — フェーズ受信時に MoveIt2 GetMotionPlan を呼び出し harvest_motion_plan を publish する。

アーキテクチャ仕様: docs/index.html §trajectory_planner_node
"""
from __future__ import annotations

import time

from tomato_harvest_sim.robot.motion_planner.observability import metric_line


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
            self._phase: HarvestTaskPhase | None = None
            self._estimate: TargetEstimate | None = None
            self._joint_state: JointStateSnapshot | None = None
            self._scene_snapshot = None  # 実際の SceneSnapshot (tray_pose 等を含む)
            self._plan_revision = 0  # publish 済み plan の単調増加版数 (Step 1 契約)
            self._producer_instance_id = uuid.uuid4().hex

            self.create_subscription(String, PHASE_TOPIC, self._on_phase, 10)
            self.create_subscription(String, TARGET_ESTIMATE_TOPIC, self._on_estimate, 10)
            self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_state, 10)
            self.create_subscription(String, TRAJECTORY_STATUS_TOPIC, self._on_trajectory_status, 10)
            self.create_subscription(String, SCENE_SNAPSHOT_TOPIC, self._on_snapshot, 10)

        def _on_phase(self, msg: String) -> None:
            try:
                self._phase = HarvestTaskPhase(msg.data)
            except ValueError:
                return
            if self._phase is HarvestTaskPhase.TARGET_FOUND:
                self._try_plan(trigger="target_found")

        def _on_estimate(self, msg: String) -> None:
            self._estimate = target_estimate_from_json(msg.data)

        def _on_joint_state(self, msg: JointState) -> None:
            self._joint_state = JointStateSnapshot(
                joint_names=tuple(str(n) for n in msg.name),
                positions_rad=tuple(float(v) for v in msg.position),
            )

        def _on_snapshot(self, msg: String) -> None:
            try:
                self._scene_snapshot = scene_snapshot_from_dict(json.loads(msg.data))
            except Exception:
                pass

        def _on_trajectory_status(self, msg: String) -> None:
            if msg.data.strip() == "aborted" and self._phase is not None:
                self.get_logger().info(metric_line(
                    "phase_abort_observed", phase=self._phase.value
                ))
                self._try_plan(trigger="trajectory_aborted")

        def _try_plan(self, *, trigger: str) -> None:
            if self._estimate is None or self._joint_state is None:
                return

            from tomato_harvest_sim.msg.contracts import Pose3D, ScenePhase, SceneSnapshot, TomatoStatus
            _p = Pose3D(0, 0, 0, 0, 0, 0)

            # 実際の SceneSnapshot があればそれを使い、collision objects の配置を正確にする
            if self._scene_snapshot is not None:
                scene_snapshot = self._scene_snapshot
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
                "target_pose": self._estimate.target_world_pose,
            })()
            phase = self._phase.value if self._phase is not None else "unknown"
            started_at = time.perf_counter()
            try:
                plan = self._planner.plan(
                    self._estimate, self._joint_state, tf_tree_snapshot, scene_snapshot
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
            self._plan_revision += 1
            plan = replace(
                plan,
                plan_revision=self._plan_revision,
                generated_at_sec=time.time(),
                planned_from_phase=self._phase,
                producer_kind=PlanProducerKind.GLOBAL_PLANNER,
                producer_instance_id=self._producer_instance_id,
            )
            self.get_logger().info(metric_line(
                "plan_published",
                plan_revision=plan.plan_revision,
                planned_from_phase=phase,
                producer_kind=plan.producer_kind.value,
                producer_instance_id=plan.producer_instance_id,
                trigger=trigger,
            ))
            out = String()
            out.data = harvest_motion_plan_to_json(plan)
            self._pub.publish(out)

    node = TrajectoryPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
