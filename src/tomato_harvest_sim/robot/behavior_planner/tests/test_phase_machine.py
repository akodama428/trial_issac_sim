from tomato_harvest_sim.msg.contracts import ControlCommand, HarvestTaskPhase, TomatoStatus
from tomato_harvest_sim.robot.behavior_planner.phase_machine import (
    ControlReceived, ExecutionSucceeded, PhaseMachineState, SnapshotTick, advance,
)


def test_grasp_transition_table_and_entry_reset() -> None:
    state = PhaseMachineState(HarvestTaskPhase.MOVING_TO_GRASP, True, 9, 8)
    at_grasp = advance(state, ExecutionSucceeded()).state
    assert at_grasp == PhaseMachineState(HarvestTaskPhase.AT_GRASP, True, 0, 0)
    transition = None
    for _ in range(30):
        transition = advance(at_grasp, SnapshotTick(TomatoStatus.ATTACHED))
        at_grasp = transition.state
    assert transition is not None
    assert transition.state == PhaseMachineState(
        HarvestTaskPhase.GRASP_EVALUATION, True, 0, 0
    )
    detaching = advance(transition.state, SnapshotTick(TomatoStatus.HELD))
    assert detaching.state == PhaseMachineState(HarvestTaskPhase.DETACHING, True, 0, 0)
    assert detaching.diagnostic == "terminal"


def test_grasp_evaluation_timeout_fails_and_resets_counters() -> None:
    state = PhaseMachineState(HarvestTaskPhase.GRASP_EVALUATION, True, eval_steps=299)
    transition = advance(state, SnapshotTick(TomatoStatus.ATTACHED))
    assert transition.state == PhaseMachineState(HarvestTaskPhase.FAILED, True, 0, 0)
    assert transition.warning == "GRASP_EVALUATION timeout"


def test_reset_enters_idle_with_clean_state() -> None:
    state = PhaseMachineState(HarvestTaskPhase.AT_GRASP, True, 12, 4)
    assert advance(state, ControlReceived(ControlCommand.RESET)).state == PhaseMachineState()
