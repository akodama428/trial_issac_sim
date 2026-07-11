"""Planner 非依存の replan trigger policy (Issue #10, Step 2)。"""
from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from tomato_harvest_sim.msg.contracts import HarvestTaskPhase
from tomato_harvest_sim.robot.motion_planner.state_aggregation import PlannerState


class ReplanTrigger(StrEnum):
    TIMER = "timer"
    ABORT = "abort"
    SCENE_CHANGE = "scene_change"
    TRACKING_ERROR = "tracking_error"


@dataclass(frozen=True)
class ReplanPolicy:
    minimum_interval_sec: float = 1.0
    tracking_error_threshold_rad: float = 0.10
    timer_phases: frozenset[HarvestTaskPhase] = frozenset({
        HarvestTaskPhase.MOVING_TO_PREGRASP,
        HarvestTaskPhase.MOVING_TO_GRASP,
        HarvestTaskPhase.MOVING_TO_PLACE,
    })


@dataclass(frozen=True)
class TriggerMemory:
    last_replan_at_sec: float | None = None
    handled_abort_generation: int = 0
    handled_scene_generation: int = 0


@dataclass(frozen=True)
class TriggerDecision:
    triggered: bool
    trigger: ReplanTrigger | None
    reason: str


def evaluate_replan_trigger(
    *,
    state: PlannerState,
    memory: TriggerMemory,
    now_sec: float,
    policy: ReplanPolicy = ReplanPolicy(),
) -> TriggerDecision:
    """最新状態から、優先度順に一つの replan trigger を選ぶ。"""
    if state.phase is None or state.joint_state is None or state.target_estimate is None:
        return TriggerDecision(False, None, "suppressed_incomplete_state")

    elapsed = (
        float("inf") if memory.last_replan_at_sec is None
        else now_sec - memory.last_replan_at_sec
    )
    if elapsed < policy.minimum_interval_sec:
        return TriggerDecision(False, None, "suppressed_minimum_interval")

    if state.abort_generation > memory.handled_abort_generation:
        return TriggerDecision(True, ReplanTrigger.ABORT, "triggered_abort")
    if state.scene_generation > memory.handled_scene_generation:
        return TriggerDecision(True, ReplanTrigger.SCENE_CHANGE, "triggered_scene_change")
    if (
        state.tracking_error_rad is not None
        and state.tracking_error_rad >= policy.tracking_error_threshold_rad
    ):
        return TriggerDecision(True, ReplanTrigger.TRACKING_ERROR, "triggered_tracking_error")
    if state.phase in policy.timer_phases:
        return TriggerDecision(True, ReplanTrigger.TIMER, "triggered_timer")
    return TriggerDecision(False, None, "suppressed_phase")


def memory_after_trigger(
    *, state: PlannerState, memory: TriggerMemory, now_sec: float
) -> TriggerMemory:
    """判定に使った event generation と実行時刻を記録する。"""
    return TriggerMemory(
        last_replan_at_sec=now_sec,
        handled_abort_generation=max(
            memory.handled_abort_generation, state.abort_generation
        ),
        handled_scene_generation=max(
            memory.handled_scene_generation, state.scene_generation
        ),
    )


def trigger_starts_planner(
    trigger: ReplanTrigger, phase: HarvestTaskPhase | None
) -> bool:
    """Step 2 で既存 full-chain planner を実行してよい trigger を返す。

    timer / scene change / tracking error は Step 3 の phase-scoped suffix
    planning が入るまで観測専用とする。既存挙動の abort replan だけを維持する。
    """
    if trigger is ReplanTrigger.ABORT:
        return True
    return (
        phase is HarvestTaskPhase.MOVING_TO_PLACE
        and trigger in {ReplanTrigger.SCENE_CHANGE, ReplanTrigger.TRACKING_ERROR}
    )


def should_inject_place_replan(
    *, enabled: bool, already_injected: bool, phase: HarvestTaskPhase | None
) -> bool:
    """E2E外乱をMOVING_TO_PLACEで一度だけ注入するか返す。"""
    return (
        enabled
        and not already_injected
        and phase is HarvestTaskPhase.MOVING_TO_PLACE
    )
