from __future__ import annotations

from pathlib import Path

import yaml


CONFIG = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "franka_ros2_control"
    / "config"
    / "franka_controllers.yaml"
)


def test_jtc_recovery_starts_new_trajectory_from_measured_state() -> None:
    parameters = yaml.safe_load(CONFIG.read_text())["joint_trajectory_controller"][
        "ros__parameters"
    ]

    assert "open_loop_control" not in parameters
    assert parameters["interpolate_from_desired_state"] is False
    assert parameters["set_last_command_interface_value_as_state_on_activation"] is False


def test_jtc_completion_requires_position_and_velocity_convergence() -> None:
    constraints = yaml.safe_load(CONFIG.read_text())["joint_trajectory_controller"][
        "ros__parameters"
    ]["constraints"]

    assert constraints["goal_time"] == 5.0
    assert constraints["stopped_velocity_tolerance"] == 0.05
    for joint_index in range(1, 8):
        assert constraints[f"panda_joint{joint_index}"]["goal"] == 0.10
