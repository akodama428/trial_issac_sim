from __future__ import annotations

from tomato_harvest_sim.msg.contracts import Pose3D
from tomato_harvest_sim.simulator.placement import (
    PlacementDecision,
    PlacementEvaluator,
    PlacementGeometry,
    PlacementObservation,
)
from tomato_harvest_sim.simulator.scene_config import placement_config_from_payload


def _config():
    return placement_config_from_payload({
        "scene": {
            "tomato_radius_m": 0.01,
            "tray_inner_size_m": [0.22, 0.16, 0.05],
            "tray_wall_thickness_m": 0.012,
        },
        "placement": {
            "release_pose": {"vertical_offset_m": 0.15, "hover_offset_m": 0.10},
            "release_ready": {
                "position_tolerance_m": 0.05,
                "max_joint_speed_rad_s": 0.05,
                "required_consecutive_steps": 2,
            },
            "gripper_open": {
                "measured_closed_gap_threshold_m": 0.065,
                "measured_gap_threshold_m": 0.07,
                "timeout_sec": 1.0,
            },
            "containment": {"boundary_margin_m": 0.005, "escape_margin_m": 0.03},
            "settling": {
                "max_linear_speed_m_s": 0.03,
                "max_angular_speed_rad_s": 0.5,
                "required_consecutive_steps": 3,
                "release_timeout_sec": 1.5,
                "settle_timeout_sec": 3.0,
            },
        },
    })


def _observation(
    *,
    pose: Pose3D = Pose3D(0.35, -0.35, 0.466, 0.0, 0.0, 0.0),
    speed: float = 0.01,
    contact: bool = True,
) -> PlacementObservation:
    return PlacementObservation(
        tomato_pose=pose,
        linear_speed_m_s=speed,
        angular_speed_rad_s=0.0,
        tomato_tray_contact=contact,
        dt_sec=1.0 / 120.0,
    )


def test_geometry_uses_tomato_radius_and_margin() -> None:
    geometry = PlacementGeometry(
        tray_pose=Pose3D(0.35, -0.35, 0.45, 0.0, 0.0, 0.0),
        config=_config(),
    )

    inside = geometry.evaluate(Pose3D(0.444, -0.35, 0.466, 0.0, 0.0, 0.0))
    outside = geometry.evaluate(Pose3D(0.446, -0.35, 0.466, 0.0, 0.0, 0.0))

    assert inside.contained
    assert not outside.contained


def test_geometry_evaluates_in_rotated_tray_local_frame() -> None:
    geometry = PlacementGeometry(
        tray_pose=Pose3D(0.0, 0.0, 0.45, 0.0, 0.0, 90.0),
        config=_config(),
    )

    result = geometry.evaluate(Pose3D(0.0, 0.094, 0.466, 0.0, 0.0, 0.0))

    assert result.contained
    assert abs(result.local_x_m - 0.094) < 1e-6


def test_release_does_not_succeed_before_contact_and_settling() -> None:
    evaluator = PlacementEvaluator(
        PlacementGeometry(
            tray_pose=Pose3D(0.35, -0.35, 0.45, 0.0, 0.0, 0.0),
            config=_config(),
        ),
        _config().settling,
    )
    evaluator.release_started()

    result = evaluator.observe(_observation(contact=False))

    assert result.decision is PlacementDecision.PENDING


def test_success_requires_consecutive_contained_contact_low_speed_samples() -> None:
    evaluator = PlacementEvaluator(
        PlacementGeometry(
            tray_pose=Pose3D(0.35, -0.35, 0.45, 0.0, 0.0, 0.0),
            config=_config(),
        ),
        _config().settling,
    )
    evaluator.release_started()

    assert evaluator.observe(_observation()).decision is PlacementDecision.PENDING
    assert evaluator.observe(_observation()).decision is PlacementDecision.PENDING
    assert evaluator.observe(_observation()).decision is PlacementDecision.PLACED


def test_contact_evidence_is_latched_when_physx_stops_streaming_resting_contacts() -> None:
    evaluator = PlacementEvaluator(
        PlacementGeometry(
            tray_pose=Pose3D(0.35, -0.35, 0.45, 0.0, 0.0, 0.0),
            config=_config(),
        ),
        _config().settling,
    )
    evaluator.release_started()

    assert evaluator.observe(_observation()).decision is PlacementDecision.PENDING
    assert evaluator.observe(_observation(contact=False)).decision is PlacementDecision.PENDING
    assert evaluator.observe(_observation(contact=False)).decision is PlacementDecision.PLACED


def test_high_speed_resets_settle_counter() -> None:
    evaluator = PlacementEvaluator(
        PlacementGeometry(
            tray_pose=Pose3D(0.35, -0.35, 0.45, 0.0, 0.0, 0.0),
            config=_config(),
        ),
        _config().settling,
    )
    evaluator.release_started()
    evaluator.observe(_observation())
    evaluator.observe(_observation(speed=0.2))

    assert evaluator.observe(_observation()).settle_steps == 1


def test_release_timeout_fails_with_reason() -> None:
    evaluator = PlacementEvaluator(
        PlacementGeometry(
            tray_pose=Pose3D(0.35, -0.35, 0.45, 0.0, 0.0, 0.0),
            config=_config(),
        ),
        _config().settling,
    )
    evaluator.release_started()

    result = PlacementDecision.PENDING
    for _ in range(181):
        result = evaluator.observe(_observation(contact=False)).decision

    assert result is PlacementDecision.FAILED
    assert evaluator.result.reason == "release_contact_timeout"
