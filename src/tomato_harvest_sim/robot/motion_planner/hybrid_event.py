"""Step 7 hybrid-planning event routing and local correction admission policy."""
from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from tomato_harvest_sim.msg.contracts import HarvestTaskPhase
from tomato_harvest_sim.robot.motion_planner.phase_suffix_replan import SUFFIX_REPLAN_PHASES
from tomato_harvest_sim.robot.motion_planner.replan_trigger import ReplanTrigger


class PlannerRoute(StrEnum):
    GLOBAL = "global"
    LOCAL = "local"
    OBSERVE = "observe"


def route_event(trigger: ReplanTrigger, phase: HarvestTaskPhase | None) -> PlannerRoute:
    """Route one normalized event to exactly one planner responsibility."""
    if trigger is ReplanTrigger.ABORT:
        return PlannerRoute.GLOBAL
    if trigger is ReplanTrigger.TRACKING_ERROR and phase in SUFFIX_REPLAN_PHASES:
        return PlannerRoute.LOCAL
    return PlannerRoute.OBSERVE


@dataclass(frozen=True)
class LocalEventMemory:
    last_accepted_at_sec: float | None = None
    last_event_id: str | None = None


@dataclass(frozen=True)
class LocalEventDecision:
    accepted: bool
    reason: str


def admit_local_event(
    *, event_id: str, event_at_sec: float, now_sec: float,
    phase: HarvestTaskPhase | None, memory: LocalEventMemory,
    minimum_interval_sec: float = 0.25, maximum_age_sec: float = 2.0,
) -> LocalEventDecision:
    """Reject duplicate, stale, out-of-phase and overly frequent local events."""
    if phase not in SUFFIX_REPLAN_PHASES:
        return LocalEventDecision(False, "unsupported_phase")
    if event_id == memory.last_event_id:
        return LocalEventDecision(False, "duplicate_event")
    if event_at_sec > now_sec or now_sec - event_at_sec > maximum_age_sec:
        return LocalEventDecision(False, "stale_event")
    if (memory.last_accepted_at_sec is not None and
            now_sec - memory.last_accepted_at_sec < minimum_interval_sec):
        return LocalEventDecision(False, "rate_limited")
    return LocalEventDecision(True, "accepted")
