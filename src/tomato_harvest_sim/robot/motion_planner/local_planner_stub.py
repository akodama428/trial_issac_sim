"""joint-space local planner — 追従誤差イベントの実補正producer (Issue #28 改善3)。

Step 5でproducer複線化の受け皿として置いたスタブを実体化し、tracking error
イベント受信時に「現在関節状態から採用済みglobal planの終端へ接続し直す」
短いjoint-space trajectoryを生成する。終端関節構成を変えないため、把持直前の
global suffix replanが別IK解へ差し替わる問題を避けつつ、executor契約を維持する。

将来MoveIt Servo等の実時間solverへ交換する場合も、このproducer境界
(producer_kind / instance_id / revision刻印と`harvest_motion_plan` topic) を保つ。
"""
from __future__ import annotations

from dataclasses import replace

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
    PlanProducerKind,
)
from tomato_harvest_sim.robot.motion_planner.phase_suffix_replan import (
    suffix_trajectory,
)

LOCAL_PLANNER_STUB_NAME = "joint_space_local_planner"

# 接続軌道の速度・時間パラメータ。JTCの許容時間内で確実に完了するよう、
# global plannerのvelocity scaling (0.2) と同程度の保守的な速度に抑える。
CONNECTION_MAX_JOINT_VELOCITY_RAD_S = 0.5
CONNECTION_MIN_DURATION_SEC = 0.5
CONNECTION_SEGMENTS = 4


def build_connection_trajectory(
    *,
    joint_names: tuple[str, ...],
    start_positions_rad: tuple[float, ...],
    target_positions_rad: tuple[float, ...],
    max_joint_velocity_rad_s: float = CONNECTION_MAX_JOINT_VELOCITY_RAD_S,
    min_duration_sec: float = CONNECTION_MIN_DURATION_SEC,
    segments: int = CONNECTION_SEGMENTS,
) -> JointTrajectory:
    """現在状態から目標構成への線形補間joint-space軌道を作る。

    追従誤差で経路から外れた腕を、採用済みplanの終端構成へ滑らかに戻すための
    接続軌道。所要時間は最大関節差分を制限速度で割った値とし、差分が小さくても
    JTCが追従できる非ゼロ長 (min_duration_sec) を保証する。

    Args:
        joint_names: 関節名。start/targetの並びはこの順に一致していること。
        start_positions_rad: 開始関節角。
        target_positions_rad: 目標関節角 (採用済みplanの終端構成)。
        max_joint_velocity_rad_s: 関節あたりの速度上限。
        min_duration_sec: 所要時間の下限。
        segments: 補間区間数 (1以上)。

    Returns:
        開始点から目標点まで単調増加時刻で並ぶtrajectory。終端速度はゼロ。
    """
    max_delta_rad = max(
        (abs(target - start)
         for start, target in zip(start_positions_rad, target_positions_rad)),
        default=0.0,
    )
    duration_sec = max(min_duration_sec, max_delta_rad / max_joint_velocity_rad_s)
    points = [JointTrajectoryPoint(start_positions_rad, 0.0)]
    for segment in range(1, segments + 1):
        ratio = segment / segments
        positions = tuple(
            start + (target - start) * ratio
            for start, target in zip(start_positions_rad, target_positions_rad)
        )
        velocities = (
            tuple(0.0 for _ in joint_names) if segment == segments else None
        )
        points.append(JointTrajectoryPoint(
            positions_rad=positions,
            time_from_start_sec=duration_sec * ratio,
            velocities_rad_s=velocities,
        ))
    return JointTrajectory(joint_names=joint_names, points=tuple(points))


def build_local_refinement_plan(
    *,
    base_plan: HarvestMotionPlan,
    phase: HarvestTaskPhase,
    current_joint_state: JointStateSnapshot,
    now_sec: float,
    instance_id: str,
    revision: int,
) -> HarvestMotionPlan | None:
    """現在状態から採用済みplan終端への接続軌道を持つ補正planを作る。

    採用済みplanの終端関節構成は変えず、そこへ至る経路だけを現在状態起点へ
    差し替える。global plannerのIK再サンプリングを経由しないため、把持直前の
    goal構成が別IK解へ置き換わる副作用がない。

    Args:
        base_plan: 土台にする採用済み plan。補正phase以外のtrajectory契約は保持する。
        phase: 補正対象の実行中 phase。自由空間 phase 以外は補正しない。
        current_joint_state: 接続軌道の起点にする最新関節状態。
        now_sec: 生成時刻 (epoch 秒)。producer instance 間の順序付けに使う。
        instance_id: この local producer の起動単位 ID。
        revision: この producer 内で単調増加する版数 (1以上)。

    Returns:
        local producer として刻印した plan。phase が補正対象外、土台 plan に
        該当 phase の trajectory がない、または現在状態に必要な関節が
        欠けている場合は None。
    """
    trajectory = suffix_trajectory(base_plan, phase)
    if trajectory is None or not trajectory.points:
        return None
    current_by_name = dict(zip(
        current_joint_state.joint_names, current_joint_state.positions_rad
    ))
    if any(name not in current_by_name for name in trajectory.joint_names):
        return None
    start_positions = tuple(
        current_by_name[name] for name in trajectory.joint_names
    )
    connection = build_connection_trajectory(
        joint_names=trajectory.joint_names,
        start_positions_rad=start_positions,
        target_positions_rad=trajectory.points[-1].positions_rad,
    )
    final_point = connection.points[-1]
    settling_point = replace(
        final_point,
        time_from_start_sec=final_point.time_from_start_sec + 1.0,
        velocities_rad_s=tuple(0.0 for _ in trajectory.joint_names),
    )
    correction = JointTrajectory(
        joint_names=connection.joint_names,
        points=(
            *connection.points,
            settling_point,
        ),
    )
    trajectory_field = {
        HarvestTaskPhase.MOVING_TO_PREGRASP: "pregrasp_joint_trajectory",
        HarvestTaskPhase.MOVING_TO_GRASP: "grasp_joint_trajectory",
        HarvestTaskPhase.MOVING_TO_PLACE: "place_joint_trajectory",
    }[phase]
    return replace(
        base_plan,
        **{trajectory_field: correction},
        planner_name=LOCAL_PLANNER_STUB_NAME,
        plan_revision=revision,
        generated_at_sec=now_sec,
        planned_from_phase=phase,
        producer_kind=PlanProducerKind.LOCAL_PLANNER,
        producer_instance_id=instance_id,
    )


def main() -> None:
    import os
    import time
    import uuid

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    from tomato_harvest_sim.msg.topics import (
        HARVEST_MOTION_PLAN_TOPIC, HYBRID_PLANNING_EVENT_TOPIC, PHASE_TOPIC,
    )
    from tomato_harvest_sim.msg.serialization import (
        harvest_motion_plan_from_json,
        harvest_motion_plan_to_json,
    )
    from tomato_harvest_sim.robot.motion_planner.observability import metric_line
    from tomato_harvest_sim.robot.motion_planner.replan_trigger import (
        parse_suffix_injection_phases,
    )

    rclpy.init()

    class LocalPlannerStubNode(Node):  # type: ignore[misc]
        """有効化された phase へ進入したとき、一度だけ local plan を publish する。"""

        def __init__(self) -> None:
            super().__init__("local_planner_stub_node")
            # INJECT_* は外乱注入E2E (採用アサーション付き) の有効化、
            # LOCAL_PLANNER_PHASES は通常運転での補正有効化。どちらでも動く。
            self._enabled_phases = parse_suffix_injection_phases(
                os.environ.get("TOMATO_HARVEST_INJECT_LOCAL_PLAN_PHASES", "")
            ) | parse_suffix_injection_phases(
                os.environ.get("TOMATO_HARVEST_LOCAL_PLANNER_PHASES", "")
            )
            from tomato_harvest_sim.robot.motion_planner.hybrid_event import LocalEventMemory
            self._event_memory = LocalEventMemory()
            self._latest_plan: HarvestMotionPlan | None = None
            self._current_joint_state: JointStateSnapshot | None = None
            self._current_phase: HarvestTaskPhase | None = None
            self._revision = 0
            self._instance_id = uuid.uuid4().hex
            self._pub = self.create_publisher(String, HARVEST_MOTION_PLAN_TOPIC, 10)
            self.create_subscription(String, PHASE_TOPIC, self._on_phase, 10)
            self.create_subscription(String, HARVEST_MOTION_PLAN_TOPIC, self._on_plan, 10)
            self.create_subscription(String, HYBRID_PLANNING_EVENT_TOPIC, self._on_event, 10)
            from sensor_msgs.msg import JointState
            from tomato_harvest_sim.msg.topics import JOINT_STATES_TOPIC
            self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_state, 10)
            self.get_logger().info(metric_line(
                "local_planner_stub_started",
                enabled_phases=",".join(sorted(p.value for p in self._enabled_phases)),
                producer_instance_id=self._instance_id,
            ))

        def _on_plan(self, msg: String) -> None:
            plan = harvest_motion_plan_from_json(msg.data)
            # 自分の publish を土台にしない。global plan だけを補正の土台にする。
            if plan.producer_kind is not PlanProducerKind.GLOBAL_PLANNER:
                return
            if self._latest_plan is None:
                self.get_logger().info(metric_line(
                    "local_planner_stub_base_plan_captured",
                    plan_revision=plan.plan_revision,
                ))
            self._latest_plan = plan

        def _on_joint_state(self, msg: object) -> None:
            self._current_joint_state = JointStateSnapshot(
                joint_names=tuple(str(name) for name in msg.name),
                positions_rad=tuple(float(value) for value in msg.position),
            )

        def _on_phase(self, msg: String) -> None:
            try:
                phase = HarvestTaskPhase(msg.data)
            except ValueError:
                return
            self._current_phase = phase

        def _on_event(self, msg: String) -> None:
            import json
            from tomato_harvest_sim.robot.motion_planner.hybrid_event import (
                LocalEventMemory, PlannerRoute, admit_local_event,
            )
            try:
                event = json.loads(msg.data)
                route = PlannerRoute(str(event["route"]))
                phase = HarvestTaskPhase(str(event["phase"]))
                event_id = str(event["event_id"])
                event_at_sec = float(event["event_at_sec"])
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                return
            if route is not PlannerRoute.LOCAL or phase != self._current_phase:
                return
            now_sec = time.time()
            decision = admit_local_event(
                event_id=event_id, event_at_sec=event_at_sec, now_sec=now_sec,
                phase=phase, memory=self._event_memory,
            )
            if not decision.accepted:
                self.get_logger().info(metric_line(
                    "local_event_suppressed", phase=phase.value, reason=decision.reason,
                ))
                return
            self._event_memory = LocalEventMemory(now_sec, event_id)
            self._publish_local_correction(phase)

        def _publish_local_correction(self, phase: HarvestTaskPhase) -> None:
            if phase not in self._enabled_phases:
                return
            if self._latest_plan is None or self._current_joint_state is None:
                self.get_logger().info(metric_line(
                    "local_plan_skipped",
                    phase=phase.value,
                    reason=("no_global_base_plan" if self._latest_plan is None
                            else "no_current_joint_state"),
                ))
                return
            candidate = build_local_refinement_plan(
                base_plan=self._latest_plan,
                phase=phase,
                current_joint_state=self._current_joint_state,
                now_sec=time.time(),
                instance_id=self._instance_id,
                revision=self._revision + 1,
            )
            if candidate is None:
                self.get_logger().info(metric_line(
                    "local_plan_skipped",
                    phase=phase.value,
                    reason="missing_phase_trajectory",
                ))
                return
            self._revision += 1
            out = String()
            out.data = harvest_motion_plan_to_json(candidate)
            self._pub.publish(out)
            self.get_logger().info(metric_line(
                "local_plan_published",
                phase=phase.value,
                plan_revision=candidate.plan_revision,
                producer_kind=candidate.producer_kind.value,
                producer_instance_id=candidate.producer_instance_id,
            ))

    node = LocalPlannerStubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
