"""local planner stub — plan producer 複線化の最小受け皿 (Issue #13, Step 5)。

Step 6 で導入する実 local planner (MoveIt Servo / Hybrid Planning) の席を先に確保する
ダミー producer。global planner とは独立した producer_kind / instance_id / revision を
刻印した plan を同じ `harvest_motion_plan` topic へ publish し、consumer 側の
arbitration・executor 下流契約が複数 producer 混在で破綻しないことを検証する。

Step 6では、現在関節状態から採用済みglobal planの終端へ接続し直す短いjoint-space
trajectoryを生成する。終端関節構成を変えないため、把持直前のglobal suffix replanが
別IK解へ差し替わる問題を避けつつ、executor契約を維持できる。
"""
from __future__ import annotations

from dataclasses import replace

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    JointTrajectory,
    PlanProducerKind,
)
from tomato_harvest_sim.robot.motion_planner.phase_suffix_replan import (
    suffix_trajectory,
)

LOCAL_PLANNER_STUB_NAME = "joint_space_local_planner"


def build_local_refinement_plan(
    *,
    base_plan: HarvestMotionPlan,
    phase: HarvestTaskPhase,
    current_joint_state: JointStateSnapshot,
    now_sec: float,
    instance_id: str,
    revision: int,
) -> HarvestMotionPlan | None:
    """global plan を土台に、local producer として刻印した補正 plan を作る。

    Args:
        base_plan: 土台にする採用済み plan。trajectory 契約はそのまま保持する。
        phase: 補正対象の実行中 phase。自由空間 phase 以外は補正しない。
        now_sec: 生成時刻 (epoch 秒)。producer instance 間の順序付けに使う。
        instance_id: この local producer の起動単位 ID。
        revision: この producer 内で単調増加する版数 (1以上)。

    Returns:
        local producer として刻印した plan。phase が補正対象外、または土台 plan に
        該当 phase の trajectory がない場合は None。
    """
    trajectory = suffix_trajectory(base_plan, phase)
    if trajectory is None or not trajectory.points:
        return None
    current_by_name = dict(zip(
        current_joint_state.joint_names, current_joint_state.positions_rad
    ))
    if any(name not in current_by_name for name in trajectory.joint_names):
        return None
    final_point = trajectory.points[-1]
    settling_point = replace(
        final_point,
        time_from_start_sec=final_point.time_from_start_sec + 1.0,
        velocities_rad_s=tuple(0.0 for _ in trajectory.joint_names),
    )
    correction = JointTrajectory(
        joint_names=trajectory.joint_names,
        points=(
            *trajectory.points,
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
            self._enabled_phases = parse_suffix_injection_phases(
                os.environ.get("TOMATO_HARVEST_INJECT_LOCAL_PLAN_PHASES", "")
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
