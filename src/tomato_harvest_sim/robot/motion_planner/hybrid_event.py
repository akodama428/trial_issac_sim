"""Step 7 hybrid-planning event routing and local correction admission policy."""
from __future__ import annotations

from enum import StrEnum

from tomato_harvest_sim.msg.contracts import HarvestTaskPhase
from tomato_harvest_sim.robot.motion_planner.replan_trigger import ReplanTrigger


class PlannerRoute(StrEnum):
    GLOBAL = "global"
    OBSERVE = "observe"


def route_event(trigger: ReplanTrigger, phase: HarvestTaskPhase | None) -> PlannerRoute:
    """Route one normalized event to exactly one planner responsibility."""
    if trigger is ReplanTrigger.ABORT:
        return PlannerRoute.GLOBAL
    return PlannerRoute.OBSERVE
