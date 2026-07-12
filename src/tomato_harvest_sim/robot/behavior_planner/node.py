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

from tomato_harvest_sim.msg.contracts import HarvestTaskPhase, TomatoStatus


# -----------------------------------------------------------------------
# 定数
# -----------------------------------------------------------------------
_POSITION_TOLERANCE_M = 0.05   # 5cm: 移動フェーズ完了判定距離
_GRASP_SETTLE_STEPS = 30       # AT_GRASP → GRASP_EVALUATION までの待機ステップ数（物理安定化待ち）
_GRASP_EVAL_TIMEOUT = 300      # GRASP_EVALUATION のタイムアウトステップ数（約 12 秒 @ 25 Hz）


def _pose_error_m(a: object, b: object) -> float:
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def execution_status_value(raw: str) -> str:
    """execution_statusのraw値からstatus文字列を取り出す。

    executorはabort診断のためstatusをJSONで報告することがある (Issue #32)。
    旧来のplain文字列 ("succeeded"等) とJSON形式の両方を受け付け、
    phase遷移判定に使うstatus値だけを返す。
    """
    import json

    text = raw.strip()
    if not text.startswith("{"):
        return text
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return text
    if isinstance(data, dict):
        return str(data.get("status", "unknown")).strip() or "unknown"
    return text


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


def moving_to_place_outcome(
    tomato_status: TomatoStatus,
    robot_tool_pose: object | None,
    place_pose: object | None,
) -> HarvestTaskPhase | None:
    """MOVING_TO_PLACE の物理的成果からの遷移先を返す。遷移不要なら None。

    ツールが place_pose の _POSITION_TOLERANCE_M 以内へ到達したら PLACED。
    搬送中の落下は FAILED。DETACHING と同じく JTC abort に対する救済経路。
    """
    if tomato_status is TomatoStatus.FALLEN:
        return HarvestTaskPhase.FAILED
    if robot_tool_pose is None or place_pose is None:
        return None
    if _pose_error_m(robot_tool_pose, place_pose) < _POSITION_TOLERANCE_M:
        return HarvestTaskPhase.PLACED
    return None


def main() -> None:
    import json
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

    rclpy.init()

    class BehaviorPlannerNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("behavior_planner_node")

            self._phase = HarvestTaskPhase.IDLE
            self._running = False
            self._last_snapshot = None
            self._last_plan = None
            self._execution_status: str = "idle"

            # AT_GRASP settle / GRASP_EVALUATION timeout カウンター
            self._grasp_settle_count: int = 0
            self._grasp_eval_count: int = 0

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
            if cmd is ControlCommand.START:
                self._running = True
                self._set_phase(HarvestTaskPhase.DETECTING)
            elif cmd is ControlCommand.STOP:
                self._running = False
                self._set_phase(HarvestTaskPhase.STOPPED)
            elif cmd is ControlCommand.RESET:
                self._running = False
                self._last_plan = None
                self._grasp_settle_count = 0
                self._grasp_eval_count = 0
                self._set_phase(HarvestTaskPhase.IDLE)

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
            self._set_phase(HarvestTaskPhase.TARGET_FOUND)

        def _on_plan(self, msg: String) -> None:
            try:
                self._last_plan = harvest_motion_plan_from_json(msg.data)
            except Exception as exc:
                self.get_logger().error(f"Failed to parse harvest_motion_plan: {exc}")
                return
            # TARGET_FOUND → MOVING_TO_PREGRASP (計画が届いた瞬間に遷移)
            if self._phase is HarvestTaskPhase.TARGET_FOUND and self._running:
                self._grasp_settle_count = 0
                self._grasp_eval_count = 0
                self._set_phase(HarvestTaskPhase.MOVING_TO_PREGRASP)

        def _on_execution_status(self, msg: String) -> None:
            self._execution_status = execution_status_value(msg.data)
            if self._execution_status == "succeeded":
                self._on_trajectory_succeeded()
            elif self._execution_status == "aborted":
                self._on_trajectory_aborted()

        # ------------------------------------------------------------------
        # Trajectory completion handlers
        # ------------------------------------------------------------------

        def _on_trajectory_succeeded(self) -> None:
            """軌道実行成功 → 移動フェーズを次フェーズへ進める。"""
            if not self._running:
                return
            if self._phase is HarvestTaskPhase.MOVING_TO_PREGRASP:
                self._set_phase(HarvestTaskPhase.MOVING_TO_GRASP)
            elif self._phase is HarvestTaskPhase.MOVING_TO_GRASP:
                self._grasp_settle_count = 0
                self._set_phase(HarvestTaskPhase.AT_GRASP)
            elif self._phase is HarvestTaskPhase.DETACHING:
                self._set_phase(HarvestTaskPhase.MOVING_TO_PLACE)
            elif self._phase is HarvestTaskPhase.MOVING_TO_PLACE:
                self._set_phase(HarvestTaskPhase.PLACED)
            elif self._phase is HarvestTaskPhase.RETURNING_HOME:
                self._set_phase(HarvestTaskPhase.COMPLETE)

        def _on_trajectory_aborted(self) -> None:
            """軌道実行中断 → フェーズは維持し、再計画の完了を待つ。

            再計画は trajectory_planner_node が /trajectory_status の "aborted" を
            受けて独立に行う。ここで phase を再 publish すると
            motion_command_node が古い plan で即座にコマンドを再生成し、
            executor の "aborted" と往復する高速ループになるため何もしない。
            """
            if not self._running:
                return
            _moving = {
                HarvestTaskPhase.MOVING_TO_PREGRASP,
                HarvestTaskPhase.MOVING_TO_GRASP,
                HarvestTaskPhase.DETACHING,
                HarvestTaskPhase.MOVING_TO_PLACE,
                HarvestTaskPhase.RETURNING_HOME,
            }
            if self._phase in _moving:
                self.get_logger().warning(
                    f"trajectory aborted at phase={self._phase.value} — waiting for replan"
                )

        # ------------------------------------------------------------------
        # Scene-snapshot-driven step (AT_GRASP / GRASP_EVALUATION / PLACED)
        # ------------------------------------------------------------------

        def _step(self) -> None:
            if not self._running or self._last_snapshot is None:
                return
            snapshot = self._last_snapshot

            if self._phase is HarvestTaskPhase.AT_GRASP:
                self._grasp_settle_count += 1
                if self._grasp_settle_count >= _GRASP_SETTLE_STEPS:
                    self._grasp_eval_count = 0
                    self._set_phase(HarvestTaskPhase.GRASP_EVALUATION)

            elif self._phase is HarvestTaskPhase.GRASP_EVALUATION:
                self._grasp_eval_count += 1
                if snapshot.tomato_status is TomatoStatus.HELD:
                    self._grasp_settle_count = 0
                    self._grasp_eval_count = 0
                    self._set_phase(HarvestTaskPhase.DETACHING)
                elif snapshot.tomato_status is TomatoStatus.FALLEN:
                    self._grasp_settle_count = 0
                    self._grasp_eval_count = 0
                    self._set_phase(HarvestTaskPhase.FAILED)
                elif self._grasp_eval_count >= _GRASP_EVAL_TIMEOUT:
                    self.get_logger().warning("GRASP_EVALUATION timeout")
                    self._grasp_settle_count = 0
                    self._grasp_eval_count = 0
                    self._set_phase(HarvestTaskPhase.FAILED)

            elif self._phase is HarvestTaskPhase.DETACHING:
                next_phase = detaching_outcome(snapshot.tomato_status)
                if next_phase is not None:
                    self._set_phase(next_phase)

            elif self._phase is HarvestTaskPhase.MOVING_TO_PLACE:
                next_phase = moving_to_place_outcome(
                    snapshot.tomato_status,
                    snapshot.robot_tool_pose,
                    self._last_plan.place_pose if self._last_plan is not None else None,
                )
                if next_phase is not None:
                    self._set_phase(next_phase)

            elif self._phase is HarvestTaskPhase.PLACED:
                if snapshot.tomato_status is TomatoStatus.PLACED:
                    self._set_phase(HarvestTaskPhase.RETURNING_HOME)
                elif snapshot.tomato_status is TomatoStatus.FALLEN:
                    self._set_phase(HarvestTaskPhase.FAILED)

            elif self._phase is HarvestTaskPhase.RETURNING_HOME:
                if snapshot.robot_home:
                    self._set_phase(HarvestTaskPhase.COMPLETE)

        # ------------------------------------------------------------------
        # Helpers
        # ------------------------------------------------------------------

        def _set_phase(self, phase: HarvestTaskPhase) -> None:
            if self._phase is not phase:
                self.get_logger().info(f"Phase: {self._phase.value} → {phase.value}")
            self._phase = phase
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
