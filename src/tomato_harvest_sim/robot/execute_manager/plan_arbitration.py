"""Global plan producerの採用可否を裁定する。"""
from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.msg.contracts import HarvestMotionPlan, HarvestTaskPhase, PlanProducerKind
from tomato_harvest_sim.robot.execute_manager.plan_adoption import evaluate_plan_adoption


@dataclass(frozen=True)
class PlanArbitrationDecision:
    """planの採用可否と観測可能な理由。"""

    adopted: bool
    reason: str


def evaluate_plan_arbitration(
    *,
    candidate: HarvestMotionPlan,
    current_plan: HarvestMotionPlan | None,
    current_phase: HarvestTaskPhase | None,
) -> PlanArbitrationDecision:
    """Global planner以外をfail-closedにして共通採用規則を適用する。"""
    if candidate.producer_kind is not PlanProducerKind.GLOBAL_PLANNER:
        return PlanArbitrationDecision(False, "rejected_unknown_producer")

    adoption = evaluate_plan_adoption(
        candidate=candidate,
        current_plan=current_plan,
        current_phase=current_phase,
    )
    return PlanArbitrationDecision(adoption.adopted, adoption.reason)
