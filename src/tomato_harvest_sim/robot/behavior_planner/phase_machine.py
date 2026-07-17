"""ROS非依存のHarvestTaskPhase状態機械。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TypeAlias

from tomato_harvest_sim.msg.contracts import ControlCommand, HarvestTaskPhase, TomatoStatus

GRASP_SETTLE_STEPS = 30
GRASP_EVAL_TIMEOUT = 300
GRASP_DIAGNOSTIC_INTERVAL_STEPS = 10


@dataclass(frozen=True)
class PhaseMachineState:
    phase: HarvestTaskPhase = HarvestTaskPhase.IDLE
    running: bool = False
    settle_steps: int = 0
    eval_steps: int = 0


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
    pass


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
        moving = {
            HarvestTaskPhase.MOVING_TO_PREGRASP, HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.DETACHING, HarvestTaskPhase.MOVING_TO_PLACE,
            HarvestTaskPhase.RETURNING_HOME,
        }
        warning = f"trajectory aborted at phase={state.phase.value} — waiting for replan" if state.phase in moving else None
        return Transition(state, warning=warning)
    if isinstance(event, ExecutionSucceeded):
        next_by_phase = {
            HarvestTaskPhase.MOVING_TO_PREGRASP: HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.MOVING_TO_GRASP: HarvestTaskPhase.AT_GRASP,
            HarvestTaskPhase.DETACHING: HarvestTaskPhase.MOVING_TO_PLACE,
            HarvestTaskPhase.MOVING_TO_PLACE: HarvestTaskPhase.PLACED,
            HarvestTaskPhase.RETURNING_HOME: HarvestTaskPhase.COMPLETE,
        }
        next_phase = next_by_phase.get(state.phase)
        return Transition(_enter(state, next_phase)) if next_phase is not None else Transition(state)
    if not isinstance(event, SnapshotTick):
        return Transition(state)
    if state.phase is HarvestTaskPhase.AT_GRASP:
        steps = state.settle_steps + 1
        if steps >= GRASP_SETTLE_STEPS:
            return Transition(_enter(state, HarvestTaskPhase.GRASP_EVALUATION))
        return Transition(
            PhaseMachineState(state.phase, state.running, steps, state.eval_steps),
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
        return Transition(PhaseMachineState(state.phase, state.running, state.settle_steps, steps), diagnostic)
    if state.phase is HarvestTaskPhase.DETACHING:
        if event.tomato_status is TomatoStatus.DETACHED:
            return Transition(_enter(state, HarvestTaskPhase.MOVING_TO_PLACE))
        if event.tomato_status is TomatoStatus.FALLEN:
            return Transition(_enter(state, HarvestTaskPhase.FAILED))
    if state.phase is HarvestTaskPhase.MOVING_TO_PLACE:
        if event.tomato_status is TomatoStatus.FALLEN:
            return Transition(_enter(state, HarvestTaskPhase.FAILED))
        if event.place_reached:
            return Transition(_enter(state, HarvestTaskPhase.PLACED))
    if state.phase is HarvestTaskPhase.PLACED:
        if event.tomato_status is TomatoStatus.PLACED:
            return Transition(_enter(state, HarvestTaskPhase.RETURNING_HOME))
        if event.tomato_status is TomatoStatus.FALLEN:
            return Transition(_enter(state, HarvestTaskPhase.FAILED))
    if state.phase is HarvestTaskPhase.RETURNING_HOME and event.robot_home:
        return Transition(_enter(state, HarvestTaskPhase.COMPLETE))
    return Transition(state)
