"""behavior_planner_node — フェーズ状態機械を保持しフェーズ遷移のみを担う。

購読:
  /tomato_harvest/control         — ControlCommand (start/stop/reset)
  /tomato_harvest/scene_snapshot  — SceneSnapshot (robot_tool_pose, tomato_status, robot_home)
  /tomato_harvest/target_estimate — TargetEstimate (DETECTING → TARGET_FOUND)
  /tomato_harvest/harvest_motion_plan — HarvestMotionPlan (TARGET_FOUND → MOVING_TO_PREGRASP)
  /tomato_harvest/execution_status — "running"/"succeeded"/"aborted" (移動フェーズ完了検知)

発行:
  /tomato_harvest/phase — HarvestTaskPhase 文字列

アーキテクチャ仕様: docs/index.html §behavior_planner_node
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from tomato_harvest_sim.msg.contracts import HarvestTaskPhase, TomatoStatus


@dataclass(frozen=True)
class ExecutionStatusObservation:
    """実行状態とtask policyに必要なabort分類を保持する。"""

    status: str
    abort_reason: str | None = None


def execution_status_observation(raw: str) -> ExecutionStatusObservation:
    """plain/JSON形式のexecution statusを正規化する。

    Args:
        raw: execution adapterがpublishしたstatus文字列。

    Returns:
        状態名と、JSONに含まれる場合はabort理由を保持する観測値。
    """
    import json

    text = raw.strip()
    if not text.startswith("{"):
        return ExecutionStatusObservation(text)
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return ExecutionStatusObservation(text)
    if not isinstance(data, dict):
        return ExecutionStatusObservation(text)
    status = str(data.get("status", "unknown")).strip() or "unknown"
    raw_reason = data.get("abort_reason")
    reason = str(raw_reason).strip() if raw_reason is not None else None
    return ExecutionStatusObservation(status, reason or None)


def _pose_error_m(a: object, b: object) -> float:
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def detaching_outcome(tomato_status: TomatoStatus) -> HarvestTaskPhase | None:
    """DETACHING の物理的成果からの遷移先を返す。遷移不要なら None。

    引き離しの目的はトマトを枝から分離することなので、JTC の succeeded を
    待たずに tomato_status で完了を判定する。トマト把持中は残留振動で
    JTC が goal_time 内に静止判定を満たせず abort するため、この成果ベース
    判定が唯一の前進経路になる。
    """
    if tomato_status is TomatoStatus.DETACHED:
        return HarvestTaskPhase.MOVING_TO_PLACE
    if tomato_status is TomatoStatus.FALLEN:
        return HarvestTaskPhase.FAILED
    return None


def should_defer_detaching_execution_result(
    phase: HarvestTaskPhase, *, evaluation_enabled: bool
) -> bool:
    """Issue #4評価中だけ軌道成功より物理hold完了を優先する。"""
    return evaluation_enabled and phase is HarvestTaskPhase.DETACHING


def moving_to_place_outcome(
    tomato_status: TomatoStatus,
    robot_tool_pose: object | None,
    place_pose: object | None,
    position_tolerance_m: float | None = None,
) -> HarvestTaskPhase | None:
    """MOVING_TO_PLACE の物理的成果からの遷移先を返す。遷移不要なら None。

    ツールが place_pose の許容距離内へ到達したら RELEASING。
    搬送中の落下は FAILED。DETACHING と同じく JTC abort に対する救済経路。
    """
    if tomato_status is TomatoStatus.FALLEN:
        return HarvestTaskPhase.FAILED
    if robot_tool_pose is None or place_pose is None:
        return None
    if position_tolerance_m is None:
        from tomato_harvest_sim.simulator.scene_config import load_placement_config
        position_tolerance_m = (
            load_placement_config().release_ready.position_tolerance_m
        )
    if _pose_error_m(robot_tool_pose, place_pose) < position_tolerance_m:
        return HarvestTaskPhase.RELEASING
    return None


def main() -> None:
    import json
    import os
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    from tomato_harvest_sim.msg.contracts import (
        ControlCommand,
        HarvestTaskPhase,
        TomatoStatus,
    )
    from tomato_harvest_sim.msg.topics import (
        CONTROL_TOPIC,
        EXECUTION_STATUS_TOPIC,
        HARVEST_MOTION_PLAN_TOPIC,
        PHASE_TOPIC,
        SCENE_SNAPSHOT_TOPIC,
        TARGET_ESTIMATE_TOPIC,
    )
    from tomato_harvest_sim.msg.serialization import (
        harvest_motion_plan_from_json,
        scene_snapshot_from_dict,
        target_estimate_from_json,
    )
    from tomato_harvest_sim.robot.behavior_planner.grasp_diagnostics import metric_payload
    from tomato_harvest_sim.robot.behavior_planner.phase_machine import (
        ControlReceived, ExecutionAborted, ExecutionSucceeded, PhaseMachineState,
        PlanAdopted, SnapshotTick, TargetEstimateReceived, Transition, advance,
    )

    rclpy.init()

    class BehaviorPlannerNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("behavior_planner_node")

            self._machine = PhaseMachineState()
            self._phase = self._machine.phase
            self._last_snapshot = None
            self._last_plan = None
            self._execution_status: str = "idle"
            self._friction_hold_evaluation_enabled = (
                int(os.environ.get("TOMATO_HARVEST_FRICTION_HOLD_EVAL_STEPS", "0"))
                > 0
            )

            self._pub = self.create_publisher(String, PHASE_TOPIC, 10)

            self.create_subscription(String, CONTROL_TOPIC, self._on_control, 10)
            self.create_subscription(String, SCENE_SNAPSHOT_TOPIC, self._on_snapshot, 10)
            self.create_subscription(String, TARGET_ESTIMATE_TOPIC, self._on_estimate, 10)
            self.create_subscription(String, HARVEST_MOTION_PLAN_TOPIC, self._on_plan, 10)
            self.create_subscription(String, EXECUTION_STATUS_TOPIC, self._on_execution_status, 10)

        # ------------------------------------------------------------------
        # Callbacks
        # ------------------------------------------------------------------

        def _on_control(self, msg: String) -> None:
            try:
                cmd = ControlCommand(msg.data.strip())
            except ValueError:
                return
            if cmd is ControlCommand.RESET:
                self._last_plan = None
            self._apply_transition(advance(self._machine, ControlReceived(cmd)))

        def _on_snapshot(self, msg: String) -> None:
            try:
                self._last_snapshot = scene_snapshot_from_dict(json.loads(msg.data))
            except Exception:
                return
            self._step()

        def _on_estimate(self, msg: String) -> None:
            if self._phase is not HarvestTaskPhase.DETECTING:
                return
            try:
                target_estimate_from_json(msg.data)  # validate
            except Exception:
                return
            self._apply_transition(advance(self._machine, TargetEstimateReceived()))

        def _on_plan(self, msg: String) -> None:
            try:
                self._last_plan = harvest_motion_plan_from_json(msg.data)
            except Exception as exc:
                self.get_logger().error(f"Failed to parse harvest_motion_plan: {exc}")
                return
            # TARGET_FOUND → MOVING_TO_PREGRASP (計画が届いた瞬間に遷移)
            self._apply_transition(advance(self._machine, PlanAdopted()))

        def _on_execution_status(self, msg: String) -> None:
            observation = execution_status_observation(msg.data)
            self._execution_status = observation.status
            if self._execution_status == "succeeded":
                self._on_trajectory_succeeded()
            elif self._execution_status == "aborted":
                self._on_trajectory_aborted(observation.abort_reason)

        # ------------------------------------------------------------------
        # Trajectory completion handlers
        # ------------------------------------------------------------------

        def _on_trajectory_succeeded(self) -> None:
            """軌道実行成功 → 移動フェーズを次フェーズへ進める。"""
            if should_defer_detaching_execution_result(
                self._phase,
                evaluation_enabled=self._friction_hold_evaluation_enabled,
            ):
                return
            self._apply_transition(advance(self._machine, ExecutionSucceeded()))

        def _on_trajectory_aborted(self, reason: str | None) -> None:
            """軌道実行中断 → フェーズは維持し、再計画の完了を待つ。

            再計画は trajectory_planner_node が /trajectory_status の "aborted" を
            受けて独立に行う。ここで phase を再 publish すると
            motion_command_node が古い plan で即座にコマンドを再生成し、
            実行系の "aborted" と往復する高速ループになるため何もしない。
            """
            if should_defer_detaching_execution_result(
                self._phase,
                evaluation_enabled=self._friction_hold_evaluation_enabled,
            ):
                return
            self._apply_transition(
                advance(self._machine, ExecutionAborted(reason))
            )

        # ------------------------------------------------------------------
        # Scene-snapshot-driven step
        # ------------------------------------------------------------------

        def _step(self) -> None:
            if not self._machine.running or self._last_snapshot is None:
                return
            snapshot = self._last_snapshot
            place_reached = moving_to_place_outcome(
                snapshot.tomato_status,
                snapshot.robot_tool_pose,
                self._last_plan.place_pose if self._last_plan is not None else None,
            ) is HarvestTaskPhase.RELEASING
            self._apply_transition(advance(
                self._machine,
                SnapshotTick(snapshot.tomato_status, place_reached, snapshot.robot_home),
            ))

        # ------------------------------------------------------------------
        # Helpers
        # ------------------------------------------------------------------

        def _emit_grasp_diagnostic(self, sample_kind: str) -> None:
            payload = metric_payload(
                self._last_snapshot,
                phase=self._phase.value,
                sample_kind=sample_kind,
                target_pose=self._last_plan.grasp_pose if self._last_plan is not None else None,
            )
            if payload is not None:
                self.get_logger().info(f"MOVEIT_METRIC {json.dumps(payload, sort_keys=True)}")

        def _set_phase(self, phase: HarvestTaskPhase) -> None:
            if self._phase is not phase:
                self.get_logger().info(f"Phase: {self._phase.value} → {phase.value}")
            self._phase = phase
            self._publish_phase()

        def _apply_transition(self, transition: Transition) -> None:
            previous = self._machine.phase
            if transition.diagnostic is not None:
                self._emit_grasp_diagnostic(transition.diagnostic)
            self._machine = transition.state
            self._phase = transition.state.phase
            if transition.warning is not None:
                self.get_logger().warning(transition.warning)
            if previous is not self._phase:
                self.get_logger().info(f"Phase: {previous.value} → {self._phase.value}")
                self._publish_phase()

        def _publish_phase(self) -> None:
            out = String()
            out.data = self._phase.value
            self._pub.publish(out)

    node = BehaviorPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
