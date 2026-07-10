"""plan adoption policy — stale plan を採用しない最低限の規則 (Issue #9, Step 1)。

producer (trajectory_planner_node) が付与する plan 契約メタデータをもとに、
consumer (motion_command_node) が新着 plan を採用するか棄却するかを判定する。
判定は pure function とし、後続 Step の trigger policy / plan arbitration から
独立して検証できるようにする。
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

_ADOPTABLE_PRODUCER_KINDS = frozenset({
    PlanProducerKind.GLOBAL_PLANNER,
    PlanProducerKind.LOCAL_PLANNER,
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
    1. producer 規則: 未知の producer_kind の plan は採用しない。
    2. 旧契約互換: revision 0 (未版数) の plan は従来どおり常に採用する。
    3. revision 規則: 採用済み revision 以下の plan は stale として棄却する。
    4. phase 整合規則: phase-bound な replan は、生成起点 phase と現在 phase が
       一致する場合のみ採用する (phase が先へ進んだ後の巻き戻りを防ぐ)。

    Args:
        candidate: 新しく届いた plan。
        current_plan: 採用済みの plan。未採用なら None。
        current_phase: consumer が観測している現在の harvest phase。不明なら None。

    Returns:
        採用可否と理由を持つ PlanAdoptionDecision。理由は観測イベントの
        安定した語彙として使う。
    """
    if candidate.producer_kind not in _ADOPTABLE_PRODUCER_KINDS:
        return PlanAdoptionDecision(adopted=False, reason="rejected_unknown_producer")

    if candidate.plan_revision == 0:
        return PlanAdoptionDecision(adopted=True, reason="adopted_legacy_contract")

    if current_plan is not None and candidate.plan_revision <= current_plan.plan_revision:
        return PlanAdoptionDecision(adopted=False, reason="rejected_stale_revision")

    if (
        candidate.planned_from_phase in _PHASE_BOUND_PLANNED_PHASES
        and current_phase is not None
        and current_phase is not candidate.planned_from_phase
    ):
        return PlanAdoptionDecision(adopted=False, reason="rejected_phase_mismatch")

    if current_plan is None:
        return PlanAdoptionDecision(adopted=True, reason="adopted_initial")
    return PlanAdoptionDecision(adopted=True, reason="adopted_newer_revision")
