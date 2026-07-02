"""move_group.launch.py

MoveIt2 move_group ノードを franka_ros2_control パッケージの設定で起動する。

起動方法:
    ros2 launch franka_ros2_control move_group.launch.py

このランチは run_ros2_components.sh から --moveit フラグで自動起動されるか、
単独で手動起動できる。

move_group が提供するサービス:
    /plan_kinematic_path (moveit_msgs/srv/GetMotionPlan)
    /apply_planning_scene (moveit_msgs/srv/ApplyPlanningScene)
"""
from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_text(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


def _load_yaml(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("franka_ros2_control")

    urdf_content = _load_text(os.path.join(pkg, "config", "franka_ros2_control.urdf"))
    srdf_content = _load_text(os.path.join(pkg, "config", "panda.srdf"))
    kinematics = _load_yaml(os.path.join(pkg, "config", "kinematics.yaml"))
    ompl = _load_yaml(os.path.join(pkg, "config", "ompl_planning.yaml"))
    joint_limits = _load_yaml(os.path.join(pkg, "config", "joint_limits.yaml"))

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            {"robot_description": urdf_content},
            {"robot_description_semantic": srdf_content},
            {"robot_description_kinematics": kinematics},
            # Planning pipeline: OMPL
            # Jazzy 形式: pipeline_names は dict ではなく直接リスト
            {"planning_pipelines": ["ompl"]},
            {"default_planning_pipeline": "ompl"},
            {"ompl": ompl},
            # Joint limits with acceleration (required by AddTimeOptimalParameterization)
            joint_limits,
            # Planning scene monitor
            {
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([move_group_node])
