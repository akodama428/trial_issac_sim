"""Planner 非依存の replan trigger policy (Issue #10, Step 2)。"""
from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from tomato_harvest_sim.msg.contracts import HarvestTaskPhase
from tomato_harvest_sim.robot.motion_planner.phase_suffix_replan import (
    SUFFIX_REPLAN_PHASES,
)
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
    """planner を実際に起動してよい trigger を返す。

    abort は従来どおり全 phase で full-chain replan を起動する。
    tracking error は自由空間phase (SUFFIX_REPLAN_PHASES) に限り suffix replan を
    起動する。接触支配の DETACHING は global replan が逆効果になり得るため
    観測専用に保つ (Issue #12 設計判断)。timer / scene change も cancel churn を
    避けるため、local planner 導入 (Step 6) まで観測専用とする。
    """
    if trigger is ReplanTrigger.ABORT:
        return True
    return phase in SUFFIX_REPLAN_PHASES and trigger is ReplanTrigger.TRACKING_ERROR


def parse_suffix_injection_phases(raw: str) -> frozenset[HarvestTaskPhase]:
    """E2E外乱注入の対象phaseを環境変数値から読み取る。

    suffix replan対象外のphase名や不正な値は無視する。

    Args:
        raw: カンマ区切りのphase値 (例: "moving_to_pregrasp, moving_to_place")。

    Returns:
        注入対象として有効なphaseの集合。
    """
    phases = set()
    for token in raw.split(","):
        name = token.strip()
        if not name:
            continue
        try:
            phase = HarvestTaskPhase(name)
        except ValueError:
            continue
        if phase in SUFFIX_REPLAN_PHASES:
            phases.add(phase)
    return frozenset(phases)


def should_inject_suffix_replan(
    *,
    enabled_phases: frozenset[HarvestTaskPhase],
    injected_phases: frozenset[HarvestTaskPhase],
    phase: HarvestTaskPhase | None,
) -> bool:
    """E2E外乱を対象phaseごとに一度だけ注入するか返す。"""
    return phase in enabled_phases and phase not in injected_phases
