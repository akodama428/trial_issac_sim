"""plan adoption policy — stale plan を採用しない最低限の規則 (Issue #9, Step 1)。

producer が付与する plan 契約メタデータをもとに、新着 plan を採用するか
棄却するかを判定する。producer 種別を問わない共通規則（metadata の fail-closed、
phase 整合、instance / 時刻による順序付け）だけを担い、producer 種別ごとの
裁定は plan_arbitration.py が担う (Issue #13, Step 5)。
consumer (motion_command_node) は arbitration policy を窓口として使う。
"""
from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    PlanProducerKind,
)

# 実行 phase 起点で生成された replan は、その phase の実行状態を前提にするため
# phase-bound として扱う。pre-motion phase (target_found 等) 起点の full-chain plan は
# 全 phase で利用されるため phase-bound にしない。
_PHASE_BOUND_PLANNED_PHASES = frozenset({
    HarvestTaskPhase.MOVING_TO_PREGRASP,
    HarvestTaskPhase.MOVING_TO_GRASP,
    HarvestTaskPhase.DETACHING,
    HarvestTaskPhase.MOVING_TO_PLACE,
    HarvestTaskPhase.RETURNING_HOME,
})

@dataclass(frozen=True)
class PlanAdoptionDecision:
    """新着 plan の採用可否と、その観測可能な理由。"""

    adopted: bool
    reason: str


def evaluate_plan_adoption(
    *,
    candidate: HarvestMotionPlan,
    current_plan: HarvestMotionPlan | None,
    current_phase: HarvestTaskPhase | None,
) -> PlanAdoptionDecision:
    """新着 plan を採用するか判定する。

    規則は次の順で適用する。
    1. producer 規則: 未知の producer_kind の plan は採用しない (fail-closed)。
       既知の種別間の裁定は arbitration policy の責務であり、ここでは扱わない。
    2. metadata規則: revision (1以上) と必須metadataの欠落はfail-closedで棄却する。
    3. phase整合規則: phase-bound planは現在phaseが一致する場合だけ採用する。
    4. 同一producer instanceはrevision、異なるinstanceは生成時刻で順序付ける。

    Args:
        candidate: 新しく届いた plan。
        current_plan: 採用済みの plan。未採用なら None。
        current_phase: consumer が観測している現在の harvest phase。不明なら None。

    Returns:
        採用可否と理由を持つ PlanAdoptionDecision。理由は観測イベントの
        安定した語彙として使う。
    """
    if candidate.producer_kind is PlanProducerKind.UNKNOWN:
        return PlanAdoptionDecision(adopted=False, reason="rejected_unknown_producer")

    if (
        candidate.plan_revision < 1
        or candidate.generated_at_sec is None
        or not candidate.producer_instance_id
        or candidate.planned_from_phase is None
    ):
        return PlanAdoptionDecision(adopted=False, reason="rejected_missing_plan_metadata")

    if (
        candidate.planned_from_phase in _PHASE_BOUND_PLANNED_PHASES
        and current_phase is None
    ):
        return PlanAdoptionDecision(adopted=False, reason="rejected_current_phase_unknown")

    if (
        candidate.planned_from_phase in _PHASE_BOUND_PLANNED_PHASES
        and current_phase is not candidate.planned_from_phase
    ):
        return PlanAdoptionDecision(adopted=False, reason="rejected_phase_mismatch")

    if current_plan is None:
        return PlanAdoptionDecision(adopted=True, reason="adopted_initial")

    if candidate.producer_instance_id == current_plan.producer_instance_id:
        if candidate.plan_revision <= current_plan.plan_revision:
            return PlanAdoptionDecision(adopted=False, reason="rejected_stale_revision")
        return PlanAdoptionDecision(adopted=True, reason="adopted_newer_revision")

    if (
        current_plan.generated_at_sec is None
        or candidate.generated_at_sec <= current_plan.generated_at_sec
    ):
        return PlanAdoptionDecision(
            adopted=False, reason="rejected_stale_producer_instance"
        )
    return PlanAdoptionDecision(
        adopted=True, reason="adopted_newer_producer_instance"
    )
