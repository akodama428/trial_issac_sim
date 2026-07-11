"""local planner stub — plan producer 複線化の最小受け皿 (Issue #13, Step 5)。

Step 6 で導入する実 local planner (MoveIt Servo / Hybrid Planning) の席を先に確保する
ダミー producer。global planner とは独立した producer_kind / instance_id / revision を
刻印した plan を同じ `harvest_motion_plan` topic へ publish し、consumer 側の
arbitration・executor 下流契約が複数 producer 混在で破綻しないことを検証する。

刻印する trajectory は採用済み global plan の該当 phase 区間をそのまま使う
（no-op refinement）。実補正の実装は Step 6 の責務であり、本 stub は
「producer 複線化の配管」だけを検証対象とする。
"""
from __future__ import annotations

from dataclasses import replace

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    PlanProducerKind,
)
from tomato_harvest_sim.robot.motion_planner.phase_suffix_replan import (
    suffix_trajectory,
)

LOCAL_PLANNER_STUB_NAME = "local_planner_stub"


def build_local_refinement_plan(
    *,
    base_plan: HarvestMotionPlan,
    phase: HarvestTaskPhase,
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
    if suffix_trajectory(base_plan, phase) is None:
        return None
    return replace(
        base_plan,
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

    from tomato_harvest_sim.msg.topics import HARVEST_MOTION_PLAN_TOPIC, PHASE_TOPIC
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
            self._published_phases: frozenset[HarvestTaskPhase] = frozenset()
            self._latest_plan: HarvestMotionPlan | None = None
            self._revision = 0
            self._instance_id = uuid.uuid4().hex
            self._pub = self.create_publisher(String, HARVEST_MOTION_PLAN_TOPIC, 10)
            self.create_subscription(String, PHASE_TOPIC, self._on_phase, 10)
            self.create_subscription(String, HARVEST_MOTION_PLAN_TOPIC, self._on_plan, 10)
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

        def _on_phase(self, msg: String) -> None:
            try:
                phase = HarvestTaskPhase(msg.data)
            except ValueError:
                return
            if phase not in self._enabled_phases or phase in self._published_phases:
                return
            if self._latest_plan is None:
                self.get_logger().info(metric_line(
                    "local_plan_skipped",
                    phase=phase.value,
                    reason="no_global_base_plan",
                ))
                return
            candidate = build_local_refinement_plan(
                base_plan=self._latest_plan,
                phase=phase,
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
            self._published_phases = self._published_phases | {phase}
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
