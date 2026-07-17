from tomato_harvest_sim.msg.contracts import ControlCommand, HarvestTaskPhase, TomatoStatus
from tomato_harvest_sim.robot.behavior_planner.phase_machine import (
    FALLEN_CONFIRM_STEPS,
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


def test_placed_phase_is_entered_only_after_physical_placement_is_confirmed() -> None:
    moving = PhaseMachineState(HarvestTaskPhase.MOVING_TO_PLACE, True)

    releasing = advance(
        moving, SnapshotTick(TomatoStatus.DETACHED, place_reached=True)
    ).state
    still_releasing = advance(
        releasing, SnapshotTick(TomatoStatus.DETACHED)
    ).state
    placed = advance(
        still_releasing, SnapshotTick(TomatoStatus.PLACED)
    ).state

    assert releasing.phase is HarvestTaskPhase.RELEASING
    assert still_releasing.phase is HarvestTaskPhase.RELEASING
    assert placed.phase is HarvestTaskPhase.PLACED


def test_transient_fallen_during_transport_does_not_fail() -> None:
    """一過性のFALLEN（摩擦把持の滑落watchdog誤検出）でサイクルを終端させない (Issue #54)。

    FrictionGraspStrategyはLOST後もHELDを再確立できるため、FALLENが1〜数tick
    見えただけでFAILEDへラッチすると、実際にはトマトを保持したままphaseだけが
    死ぬ（手動実行で観測した「リリース前の固着」）。
    """
    state = PhaseMachineState(HarvestTaskPhase.MOVING_TO_PLACE, True)
    after_one = advance(state, SnapshotTick(TomatoStatus.FALLEN)).state
    assert after_one.phase is HarvestTaskPhase.MOVING_TO_PLACE
    assert after_one.fallen_steps == 1


def test_fallen_confirmation_counter_resets_on_recovery() -> None:
    state = PhaseMachineState(
        HarvestTaskPhase.MOVING_TO_PLACE, True, fallen_steps=FALLEN_CONFIRM_STEPS - 1
    )
    recovered = advance(state, SnapshotTick(TomatoStatus.HELD)).state
    assert recovered.phase is HarvestTaskPhase.MOVING_TO_PLACE
    assert recovered.fallen_steps == 0


def test_persistent_fallen_fails_after_confirmation() -> None:
    state = PhaseMachineState(HarvestTaskPhase.MOVING_TO_PLACE, True)
    for _ in range(FALLEN_CONFIRM_STEPS):
        state = advance(state, SnapshotTick(TomatoStatus.FALLEN)).state
    assert state.phase is HarvestTaskPhase.FAILED


def test_fallen_debounce_applies_to_detaching_and_releasing() -> None:
    for phase in (HarvestTaskPhase.DETACHING, HarvestTaskPhase.RELEASING):
        state = PhaseMachineState(phase, True)
        after_one = advance(state, SnapshotTick(TomatoStatus.FALLEN)).state
        assert after_one.phase is phase, phase
        for _ in range(FALLEN_CONFIRM_STEPS - 1):
            after_one = advance(after_one, SnapshotTick(TomatoStatus.FALLEN)).state
        assert after_one.phase is HarvestTaskPhase.FAILED, phase
