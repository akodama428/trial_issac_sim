import os
from pathlib import Path

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("franka_ros2_control")

    arm_command_mode = os.environ.get(
        "TOMATO_HARVEST_ARM_COMMAND_MODE", "position_velocity"
    )
    if arm_command_mode not in {"position_velocity", "effort"}:
        raise RuntimeError(f"unsupported arm command mode: {arm_command_mode}")
    controller_file = (
        "franka_controllers_effort.yaml"
        if arm_command_mode == "effort"
        else "franka_controllers.yaml"
    )
    controllers_yaml = PathJoinSubstitution([pkg_share, "config", controller_file])

    robot_description_text = open(
        Path(__file__).parent.parent / "config" / "franka_ros2_control.urdf"
    ).read().replace("__ARM_COMMAND_MODE__", arm_command_mode)
    robot_description = {"robot_description": robot_description_text}

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ])
