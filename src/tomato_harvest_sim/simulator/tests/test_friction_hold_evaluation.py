import pytest

from tomato_harvest_sim.msg.contracts import Pose3D
from tomato_harvest_sim.simulator.friction_hold_evaluation import (
    FrictionHoldEvaluation,
    FrictionHoldEvaluationConfig,
)


def _pose(x: float, y: float = 0.0, z: float = 0.0) -> Pose3D:
    return Pose3D(x, y, z, 0.0, 0.0, 0.0)


def test_hold_starts_only_after_required_lift_distance() -> None:
    evaluator = FrictionHoldEvaluation(
        FrictionHoldEvaluationConfig(minimum_lift_distance_m=0.1, required_steps=2)
    )

    result = evaluator.observe(
        stem_distance_m=0.099,
        hand_pose=_pose(0.0),
        tomato_pose=_pose(0.1),
    )

    assert not result.active
    assert not result.complete


def test_hold_completes_after_required_physics_intervals() -> None:
    evaluator = FrictionHoldEvaluation(
        FrictionHoldEvaluationConfig(minimum_lift_distance_m=0.1, required_steps=2)
    )
    first = evaluator.observe(
        stem_distance_m=0.1, hand_pose=_pose(0.0), tomato_pose=_pose(0.1)
    )
    second = evaluator.observe(
        stem_distance_m=0.1, hand_pose=_pose(0.0), tomato_pose=_pose(0.1)
    )
    third = evaluator.observe(
        stem_distance_m=0.1, hand_pose=_pose(0.0), tomato_pose=_pose(0.1)
    )

    assert first.active and first.elapsed_steps == 0
    assert second.active and second.elapsed_steps == 1
    assert third.complete and third.elapsed_steps == 2


def test_hold_slip_is_measured_in_hand_local_frame() -> None:
    evaluator = FrictionHoldEvaluation(
        FrictionHoldEvaluationConfig(minimum_lift_distance_m=0.1, required_steps=2)
    )
    evaluator.observe(
        stem_distance_m=0.1,
        hand_pose=Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        tomato_pose=Pose3D(0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
    )

    rigid_rotation = evaluator.observe(
        stem_distance_m=0.1,
        hand_pose=Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 90.0),
        tomato_pose=Pose3D(0.0, 0.1, 0.0, 0.0, 0.0, 0.0),
    )
    slipped = evaluator.observe(
        stem_distance_m=0.1,
        hand_pose=Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 90.0),
        tomato_pose=Pose3D(0.0, 0.106, 0.0, 0.0, 0.0, 0.0),
    )

    assert rigid_rotation.slip_m < 1e-9
    assert slipped.slip_m == pytest.approx(0.006)
    assert slipped.maximum_slip_m == pytest.approx(0.006)
