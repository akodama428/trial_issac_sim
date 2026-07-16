from tomato_harvest_sim.msg.contracts import HarvestTaskPhase, MotionKind
from tomato_harvest_sim.robot.execute_manager.motion_command import PHASE_COMMAND_TABLE
from tomato_harvest_sim.robot.execute_manager.servo_execution_adapter import GripperGate


def test_every_motion_phase_declares_execution_intent() -> None:
    expected = {
        HarvestTaskPhase.MOVING_TO_PREGRASP: (MotionKind.FOLLOW_TRAJECTORY, False),
        HarvestTaskPhase.MOVING_TO_GRASP: (MotionKind.FOLLOW_TRAJECTORY, True),
        HarvestTaskPhase.AT_GRASP: (MotionKind.HOLD, True),
        HarvestTaskPhase.GRASP_EVALUATION: (MotionKind.HOLD, True),
        HarvestTaskPhase.DETACHING: (MotionKind.FOLLOW_TRAJECTORY, False),
        HarvestTaskPhase.MOVING_TO_PLACE: (MotionKind.FOLLOW_TRAJECTORY, False),
        HarvestTaskPhase.PLACED: (MotionKind.HOLD, False),
        HarvestTaskPhase.RETURNING_HOME: (MotionKind.FOLLOW_TRAJECTORY, False),
    }
    assert {
        phase: (spec.motion_kind, spec.terminal_pose_tracking)
        for phase, spec in PHASE_COMMAND_TABLE.items()
    } == expected


def test_gripper_gate_is_the_single_deduplicating_decision_owner() -> None:
    gate = GripperGate()
    assert gate.command_started(True) is True
    assert gate.terminal_reached(True) is None
    assert gate.command_started(False) is False
    assert gate.terminal_reached(False) is None
