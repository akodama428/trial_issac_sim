"""ROS非依存のHarvestTaskPhase状態機械。"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TypeAlias

from tomato_harvest_sim.msg.contracts import ControlCommand, HarvestTaskPhase, TomatoStatus

GRASP_SETTLE_STEPS = 30
GRASP_EVAL_TIMEOUT = 300
GRASP_DIAGNOSTIC_INTERVAL_STEPS = 10
# 搬送・設置中のFALLENはこのtick数連続した場合のみFAILEDへ落とす (Issue #54)。
# FrictionGraspStrategyの滑落watchdogは、abort/replan時の加減速でhand相対変位が
# 一瞬5mmを超えるとLOST(FALLEN)を出すが、実把持が残っていれば数physics stepで
# HELDを再確立する。一過性のFALLENで終端FAILEDへラッチすると、トマトを保持した
# ままphaseだけが死ぬ復旧不能な固着になる。実落下ならFALLENが継続するため、
# snapshot tick(約30-60Hz)で30連続 ≒ 0.5〜1.0秒の確認で判定できる。
FALLEN_CONFIRM_STEPS = 30
# DETACHINGの実行abort後、JTC結果より遅れて届く物理DETACHEDを待つ上限。
# 成果が得られない場合は有限時間でFAILEDへ落とし、replan対象外phaseでの
# 永久待機を防ぐ (Issue #58)。
DETACH_ABORT_OUTCOME_CONFIRM_STEPS = 30


@dataclass(frozen=True)
class PhaseMachineState:
    phase: HarvestTaskPhase = HarvestTaskPhase.IDLE
    running: bool = False
    settle_steps: int = 0
    eval_steps: int = 0
    fallen_steps: int = 0
    abort_pending: bool = False
    abort_reason: str | None = None
    abort_wait_steps: int = 0


@dataclass(frozen=True)
class ControlReceived:
    command: ControlCommand


@dataclass(frozen=True)
class TargetEstimateReceived:
    pass


@dataclass(frozen=True)
class PlanAdopted:
    pass


@dataclass(frozen=True)
class ExecutionSucceeded:
    pass


@dataclass(frozen=True)
class ExecutionAborted:
    reason: str | None = None


@dataclass(frozen=True)
class SnapshotTick:
    tomato_status: TomatoStatus
    place_reached: bool = False
    robot_home: bool = False


PhaseEvent: TypeAlias = (
    ControlReceived | TargetEstimateReceived | PlanAdopted | ExecutionSucceeded
    | ExecutionAborted | SnapshotTick
)


@dataclass(frozen=True)
class Transition:
    state: PhaseMachineState
    diagnostic: str | None = None
    warning: str | None = None


def _enter(state: PhaseMachineState, phase: HarvestTaskPhase, *, running: bool | None = None) -> PhaseMachineState:
    """新phaseへ入り、phaseローカルのcounterを必ず初期化する。"""
    return PhaseMachineState(
        phase=phase,
        running=state.running if running is None else running,
        settle_steps=0,
        eval_steps=0,
    )


def _confirm_fallen(state: PhaseMachineState) -> Transition:
    """FALLENを連続tickで確認し、確認完了までは現phaseへ留まる (Issue #54)。"""
    steps = state.fallen_steps + 1
    if steps >= FALLEN_CONFIRM_STEPS:
        return Transition(_enter(state, HarvestTaskPhase.FAILED))
    return Transition(replace(state, fallen_steps=steps))


def advance(state: PhaseMachineState, event: PhaseEvent) -> Transition:
    """現状態とeventから次状態とI/O shell向け要求を返す。"""
    if isinstance(event, ControlReceived):
        if event.command is ControlCommand.START:
            return Transition(_enter(state, HarvestTaskPhase.DETECTING, running=True))
        if event.command is ControlCommand.STOP:
            return Transition(_enter(state, HarvestTaskPhase.STOPPED, running=False))
        return Transition(PhaseMachineState())
    if not state.running:
        return Transition(state)
    if isinstance(event, TargetEstimateReceived) and state.phase is HarvestTaskPhase.DETECTING:
        return Transition(_enter(state, HarvestTaskPhase.TARGET_FOUND))
    if isinstance(event, PlanAdopted) and state.phase is HarvestTaskPhase.TARGET_FOUND:
        return Transition(_enter(state, HarvestTaskPhase.MOVING_TO_PREGRASP))
    if isinstance(event, ExecutionAborted):
        if event.reason == "missing_trajectory":
            return Transition(
                _enter(state, HarvestTaskPhase.FAILED),
                warning="phase_plan_contract_violation",
            )
        if state.phase is HarvestTaskPhase.DETACHING:
            if state.abort_pending:
                return Transition(state)
            return Transition(
                replace(
                    state,
                    abort_pending=True,
                    abort_reason=event.reason,
                    abort_wait_steps=0,
                ),
                warning="detaching_abort_outcome_wait",
            )
        moving = {
            HarvestTaskPhase.MOVING_TO_PREGRASP, HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.MOVING_TO_PLACE, HarvestTaskPhase.RETURNING_HOME,
        }
        warning = f"trajectory aborted at phase={state.phase.value} — waiting for replan" if state.phase in moving else None
        return Transition(state, warning=warning)
    if isinstance(event, ExecutionSucceeded):
        next_by_phase = {
            HarvestTaskPhase.MOVING_TO_PREGRASP: HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.MOVING_TO_GRASP: HarvestTaskPhase.AT_GRASP,
            HarvestTaskPhase.DETACHING: HarvestTaskPhase.MOVING_TO_PLACE,
            HarvestTaskPhase.MOVING_TO_PLACE: HarvestTaskPhase.RELEASING,
            HarvestTaskPhase.RETURNING_HOME: HarvestTaskPhase.COMPLETE,
        }
        next_phase = next_by_phase.get(state.phase)
        return Transition(_enter(state, next_phase)) if next_phase is not None else Transition(state)
    if not isinstance(event, SnapshotTick):
        return Transition(state)
    if event.tomato_status is not TomatoStatus.FALLEN and state.fallen_steps:
        state = replace(state, fallen_steps=0)
    if state.phase is HarvestTaskPhase.AT_GRASP:
        steps = state.settle_steps + 1
        if steps >= GRASP_SETTLE_STEPS:
            return Transition(_enter(state, HarvestTaskPhase.GRASP_EVALUATION))
        return Transition(
            replace(state, settle_steps=steps),
            diagnostic="entry" if steps == 1 else None,
        )
    if state.phase is HarvestTaskPhase.GRASP_EVALUATION:
        steps = state.eval_steps + 1
        terminal = event.tomato_status in {TomatoStatus.HELD, TomatoStatus.FALLEN}
        diagnostic = "terminal" if terminal else (
            "periodic" if steps == 1 or steps % GRASP_DIAGNOSTIC_INTERVAL_STEPS == 0 else None
        )
        if event.tomato_status is TomatoStatus.HELD:
            return Transition(_enter(state, HarvestTaskPhase.DETACHING), diagnostic)
        if event.tomato_status is TomatoStatus.FALLEN:
            return Transition(_enter(state, HarvestTaskPhase.FAILED), diagnostic)
        if steps >= GRASP_EVAL_TIMEOUT:
            return Transition(_enter(state, HarvestTaskPhase.FAILED), diagnostic, "GRASP_EVALUATION timeout")
        return Transition(replace(state, eval_steps=steps), diagnostic)
    if state.phase is HarvestTaskPhase.DETACHING:
        if event.tomato_status is TomatoStatus.DETACHED:
            return Transition(_enter(state, HarvestTaskPhase.MOVING_TO_PLACE))
        if event.tomato_status is TomatoStatus.FALLEN:
            return _confirm_fallen(state)
        if state.abort_pending:
            steps = state.abort_wait_steps + 1
            if steps >= DETACH_ABORT_OUTCOME_CONFIRM_STEPS:
                return Transition(
                    _enter(state, HarvestTaskPhase.FAILED),
                    warning="detaching_abort_outcome_timeout",
                )
            return Transition(replace(state, abort_wait_steps=steps))
    if state.phase is HarvestTaskPhase.MOVING_TO_PLACE:
        if event.tomato_status is TomatoStatus.FALLEN:
            return _confirm_fallen(state)
        if event.place_reached:
            return Transition(_enter(state, HarvestTaskPhase.RELEASING))
    if state.phase is HarvestTaskPhase.RELEASING:
        if event.tomato_status is TomatoStatus.PLACED:
            return Transition(_enter(state, HarvestTaskPhase.PLACED))
        if event.tomato_status is TomatoStatus.FALLEN:
            return _confirm_fallen(state)
    if state.phase is HarvestTaskPhase.PLACED:
        return Transition(_enter(state, HarvestTaskPhase.RETURNING_HOME))
    if state.phase is HarvestTaskPhase.RETURNING_HOME and event.robot_home:
        return Transition(_enter(state, HarvestTaskPhase.COMPLETE))
    return Transition(state)
