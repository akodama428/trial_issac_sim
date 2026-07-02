"""franka_controllers.launch.py

franka_ros2_control の C++ ノード群を起動する。

- ros2_control_node (controller_manager)
  - IsaacSimHardwareInterface plugin
- joint_state_broadcaster spawner
- joint_trajectory_controller spawner

このランチは harvest_sim.launch.py から include されるか、
単独で使用できる。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_franka = get_package_share_directory("franka_ros2_control")

    urdf_file = os.path.join(pkg_franka, "config", "franka_ros2_control.urdf")
    controllers_config = os.path.join(pkg_franka, "config", "franka_controllers.yaml")

    with open(urdf_file, encoding="utf-8") as f:
        robot_description = f.read()

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_config,
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    joint_trajectory_controller_spawner = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_trajectory_controller",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            ros2_control_node,
            TimerAction(
                period=2.0,
                actions=[joint_state_broadcaster_spawner],
            ),
            TimerAction(
                period=2.5,
                actions=[joint_trajectory_controller_spawner],
            ),
        ]
    )
