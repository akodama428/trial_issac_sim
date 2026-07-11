"""plan arbitration policy — 複数 plan producer の裁定規則 (Issue #13, Step 5)。

責務分担:
- adoption policy (plan_adoption.py): producer 種別を問わない共通の契約検証
  （metadata の fail-closed、phase 整合、instance / 時刻による順序付け）。
- arbitration policy (本module): producer 種別ごとの受け入れ条件。どの producer の
  plan を、どの前提でパイプラインに入れてよいかを裁定する。

consumer (motion_command_node) は本 module だけを窓口として使う。
"""
from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    PlanProducerKind,
)
from tomato_harvest_sim.robot.execute_manager.plan_adoption import (
    evaluate_plan_adoption,
)

# 受け入れ可能な producer 種別。新しい producer はここへ追加し、
# 必要なら種別固有の裁定規則を evaluate_plan_arbitration に足す。
_SUPPORTED_PRODUCER_KINDS = frozenset({
    PlanProducerKind.GLOBAL_PLANNER,
    PlanProducerKind.LOCAL_PLANNER,
})
_LOCAL_CONTROL_PHASES = frozenset({
    HarvestTaskPhase.MOVING_TO_PREGRASP,
    HarvestTaskPhase.MOVING_TO_GRASP,
    HarvestTaskPhase.MOVING_TO_PLACE,
})


@dataclass(frozen=True)
class PlanArbitrationDecision:
    """producer 裁定を含めた plan 採用可否と、その観測可能な理由。"""

    adopted: bool
    reason: str


def evaluate_plan_arbitration(
    *,
    candidate: HarvestMotionPlan,
    current_plan: HarvestMotionPlan | None,
    current_phase: HarvestTaskPhase | None,
) -> PlanArbitrationDecision:
    """producer 種別を裁定したうえで、新着 plan を採用するか判定する。

    規則は次の順で適用する。
    1. producer 規則: 未知の producer_kind は採用しない (fail-closed)。
    2. 共通契約規則: adoption policy の metadata / phase整合 / 順序規則。
    3. local 固有規則: local plan は「採用済み plan の実行中 phase の補正」に限る。
       - 採用済み plan がない状態では採用しない（global plan の土台が必要）。
       - planned_from_phase が現在 phase と一致しない local plan は採用しない。

    Args:
        candidate: 新しく届いた plan。
        current_plan: 採用済みの plan。未採用なら None。
        current_phase: consumer が観測している現在の harvest phase。不明なら None。

    Returns:
        採用可否と理由を持つ PlanArbitrationDecision。理由は観測イベントの
        安定した語彙として使う。
    """
    if candidate.producer_kind not in _SUPPORTED_PRODUCER_KINDS:
        return PlanArbitrationDecision(adopted=False, reason="rejected_unknown_producer")

    if (
        candidate.producer_kind is PlanProducerKind.GLOBAL_PLANNER
        and current_plan is not None
        and current_plan.producer_kind is PlanProducerKind.LOCAL_PLANNER
        and current_phase in _LOCAL_CONTROL_PHASES
        and candidate.planned_from_phase is current_phase
        and not candidate.planner_name.endswith(":abort")
    ):
        return PlanArbitrationDecision(
            adopted=False,
            reason="rejected_global_during_local_control",
        )

    adoption = evaluate_plan_adoption(
        candidate=candidate,
        current_plan=current_plan,
        current_phase=current_phase,
    )
    if not adoption.adopted:
        return PlanArbitrationDecision(adopted=False, reason=adoption.reason)

    if candidate.producer_kind is PlanProducerKind.LOCAL_PLANNER:
        if current_plan is None:
            return PlanArbitrationDecision(
                adopted=False, reason="rejected_local_without_adopted_plan"
            )
        if candidate.planned_from_phase is not current_phase:
            return PlanArbitrationDecision(
                adopted=False, reason="rejected_local_phase_mismatch"
            )

    return PlanArbitrationDecision(adopted=True, reason=adoption.reason)
