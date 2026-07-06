"""robot_nodes.launch.py — tomato_harvest_robot パッケージの 6 ノードを一括起動する。"""
from __future__ import annotations

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package = "tomato_harvest_robot"

    tomato_detector = Node(
        package=package,
        executable="tomato_detector_node",
        name="tomato_detector_node",
        output="screen",
    )

    behavior_planner = Node(
        package=package,
        executable="behavior_planner_node",
        name="behavior_planner_node",
        output="screen",
    )

    trajectory_planner = Node(
        package=package,
        executable="trajectory_planner_node",
        name="trajectory_planner_node",
        output="screen",
    )

    trajectory_monitor = Node(
        package=package,
        executable="trajectory_monitor_node",
        name="trajectory_monitor_node",
        output="screen",
    )

    motion_command = Node(
        package=package,
        executable="motion_command_node",
        name="motion_command_node",
        output="screen",
    )

    motion_command_executor = Node(
        package=package,
        executable="motion_command_executor_node",
        name="motion_command_executor_node",
        output="screen",
    )

    return LaunchDescription([
        tomato_detector,
        behavior_planner,
        trajectory_planner,
        trajectory_monitor,
        motion_command,
        motion_command_executor,
    ])
