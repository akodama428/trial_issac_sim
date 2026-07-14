from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/franka_ros2_control/config/moveit_servo.yaml"
LAUNCH = ROOT / "src/franka_ros2_control/launch/move_group.launch.py"
DOCKERFILE = ROOT / "docker/Dockerfile"


def test_servo_output_is_connected_to_jtc() -> None:
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))

    assert config["command_out_type"] == "trajectory_msgs/JointTrajectory"
    assert config["command_out_topic"] == (
        "/joint_trajectory_controller/joint_trajectory"
    )
    assert config["publish_joint_positions"] is True
    assert config["publish_joint_velocities"] is False
    assert config["check_collisions"] is False


def test_servo_keeps_singularity_and_joint_limit_safety_thresholds() -> None:
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))

    # CI URDF has no collision geometry; PlanningScene safety adapter remains
    # the collision observation path until production geometry is available.
    assert config["check_collisions"] is False
    assert config["lower_singularity_threshold"] == 17.0
    assert config["hard_stop_singularity_threshold"] == 30.0
    assert config["joint_limit_margins"] == [0.10]


def test_servo_launch_supports_only_off_and_jtc_modes() -> None:
    launch_source = LAUNCH.read_text(encoding="utf-8")

    assert '"servo_mode"' in launch_source
    assert 'default_value="off"' in launch_source
    assert 'choices=["off", "jtc"]' in launch_source
    assert "shadow" not in launch_source
    assert 'package="moveit_servo"' in launch_source


def test_jtc_mode_disables_follow_joint_trajectory_executor() -> None:
    runner = (ROOT / "scripts/run_ros2_components.sh").read_text(encoding="utf-8")

    assert 'TOMATO_HARVEST_SERVO_MODE:-off' in runner
    assert '"${SERVO_MODE}" != "jtc"' in runner
    assert 'python3 -m tomato_harvest_sim.robot.execute_manager.servo_execution_adapter' in runner
    assert '"${SERVO_MODE}" != "jtc"' in runner


def test_ci_image_installs_servo_explicitly() -> None:
    dockerfile = DOCKERFILE.read_text(encoding="utf-8")

    assert "ros-${ROS_DISTRO}-moveit-servo" in dockerfile
