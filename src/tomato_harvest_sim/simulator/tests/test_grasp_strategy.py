from tomato_harvest_sim.msg.contracts import Pose3D
from tomato_harvest_sim.simulator.grasp_strategy import (
    FrictionGraspConfig,
    FrictionGraspStrategy,
    GraspDecision,
)


def _pose(z: float) -> Pose3D:
    return Pose3D(0.62, 0.0, z, 0.0, 0.0, 0.0)


def test_friction_grasp_requires_continuous_bilateral_force_and_low_relative_speed() -> None:
    strategy = FrictionGraspStrategy(
        FrictionGraspConfig(required_steps=3, minimum_force_n=1.0,
                            maximum_relative_speed_m_s=0.02, maximum_slip_m=0.005)
    )

    assert strategy.observe(True, 1.2, 1.3, _pose(0.64), _pose(0.54), 1 / 60) is GraspDecision.NONE
    assert strategy.observe(True, 1.2, 1.3, _pose(0.64), _pose(0.54), 1 / 60) is GraspDecision.NONE
    assert strategy.observe(True, 1.2, 1.3, _pose(0.64), _pose(0.54), 1 / 60) is GraspDecision.HELD


def test_friction_grasp_resets_contact_count_when_one_finger_loses_force() -> None:
    strategy = FrictionGraspStrategy(FrictionGraspConfig(2, 1.0, 0.02, 0.005))

    strategy.observe(True, 2.0, 2.0, _pose(0.64), _pose(0.54), 1 / 60)
    strategy.observe(True, 2.0, 0.0, _pose(0.64), _pose(0.54), 1 / 60)

    assert strategy.observe(True, 2.0, 2.0, _pose(0.64), _pose(0.54), 1 / 60) is GraspDecision.NONE


def test_friction_grasp_rejects_fast_hand_tomato_relative_motion() -> None:
    strategy = FrictionGraspStrategy(FrictionGraspConfig(2, 1.0, 0.02, 0.005))
    strategy.observe(True, 2.0, 2.0, _pose(0.64), _pose(0.54), 1 / 60)

    decision = strategy.observe(True, 2.0, 2.0, _pose(0.66), _pose(0.54), 1 / 60)

    assert decision is GraspDecision.NONE


def test_friction_grasp_reports_lost_when_relative_displacement_exceeds_five_mm() -> None:
    strategy = FrictionGraspStrategy(FrictionGraspConfig(2, 1.0, 0.02, 0.005))
    strategy.observe(True, 2.0, 2.0, _pose(0.64), _pose(0.54), 1 / 60)
    assert strategy.observe(True, 2.0, 2.0, _pose(0.64), _pose(0.54), 1 / 60) is GraspDecision.HELD

    decision = strategy.observe(True, 2.0, 2.0, _pose(0.646), _pose(0.54), 1 / 60)

    assert decision is GraspDecision.LOST


def test_friction_grasp_does_not_treat_rigid_hand_rotation_as_slip() -> None:
    strategy = FrictionGraspStrategy(FrictionGraspConfig(1, 1.0, 1.0, 0.005))
    hand = Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    tomato = Pose3D(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert strategy.observe(True, 2.0, 2.0, hand, tomato, 1 / 60) is GraspDecision.HELD

    rotated_hand = Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 90.0)
    rotated_tomato = Pose3D(0.0, 0.1, 0.0, 0.0, 0.0, 0.0)

    assert strategy.observe(True, 2.0, 2.0, rotated_hand, rotated_tomato, 1 / 60) is GraspDecision.NONE


def test_friction_grasp_reports_released_when_gripper_opens() -> None:
    strategy = FrictionGraspStrategy(FrictionGraspConfig(1, 1.0, 0.02, 0.005))
    assert strategy.observe(True, 2.0, 2.0, _pose(0.64), _pose(0.54), 1 / 60) is GraspDecision.HELD

    assert strategy.observe(False, 0.0, 0.0, _pose(0.64), _pose(0.54), 1 / 60) is GraspDecision.RELEASED
