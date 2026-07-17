from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from tomato_harvest_sim.simulator.scene_config import (
    load_placement_config,
    placement_config_from_payload,
)


def test_repository_placement_settings_are_loaded_from_scene_yaml() -> None:
    load_placement_config.cache_clear()

    config = load_placement_config()

    assert config.release_pose.vertical_offset_m == 0.15
    assert config.release_ready.position_tolerance_m == 0.05
    assert config.scene_geometry.tray_inner_size_m == (0.22, 0.16, 0.05)


def test_repository_yaml_has_one_placement_section() -> None:
    payload = yaml.safe_load(Path("config/scene.yaml").read_text())

    assert "placement" in payload
    assert "place_distance_m" not in payload["placement"]


def test_invalid_tomato_and_tray_geometry_is_rejected() -> None:
    payload = {
        "scene": {
            "tomato_radius_m": 0.05,
            "tray_inner_size_m": [0.10, 0.10, 0.05],
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
    }

    with pytest.raises(ValueError, match="tomato"):
        placement_config_from_payload(payload)
