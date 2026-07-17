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
import xml.etree.ElementTree as ET

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


def _safety_observation_urdf(kinematic_urdf: str) -> str:
    """安全観測node専用の保守的なprimitive collision modelを付与する。"""
    root = ET.fromstring(kinematic_urdf)
    sphere_radii = {
        "panda_link0": 0.09, "panda_link1": 0.075, "panda_link2": 0.075,
        "panda_link3": 0.07, "panda_link4": 0.07, "panda_link5": 0.065,
        "panda_link6": 0.06, "panda_link7": 0.055, "panda_link8": 0.055,
        "panda_hand": 0.07,
    }
    for link in root.findall("link"):
        radius = sphere_radii.get(link.attrib.get("name", ""))
        if radius is not None:
            geometry = ET.SubElement(ET.SubElement(link, "collision"), "geometry")
            ET.SubElement(geometry, "sphere", {"radius": str(radius)})
        elif link.attrib.get("name") in {"panda_leftfinger", "panda_rightfinger"}:
            collision = ET.SubElement(link, "collision")
            ET.SubElement(collision, "origin", {"xyz": "0 0 0.025"})
            geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(geometry, "box", {"size": "0.018 0.018 0.05"})
    return ET.tostring(root, encoding="unicode")


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("franka_ros2_control")

    urdf_content = _load_text(os.path.join(pkg, "config", "franka_ros2_control.urdf"))
    srdf_content = _load_text(os.path.join(pkg, "config", "panda.srdf"))
    kinematics = _load_yaml(os.path.join(pkg, "config", "kinematics.yaml"))
    ompl = _load_yaml(os.path.join(pkg, "config", "ompl_planning.yaml"))
    joint_limits = _load_yaml(os.path.join(pkg, "config", "joint_limits.yaml"))
    servo = _load_yaml(os.path.join(pkg, "config", "moveit_servo.yaml"))

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

    safety_observation_node = Node(
        package="franka_ros2_control",
        executable="local_safety_observation_node",
        name="local_safety_observation_node",
        output="screen",
        parameters=[
            {"robot_description": _safety_observation_urdf(urdf_content)},
            {"robot_description_semantic": srdf_content},
            {"robot_description_kinematics": kinematics},
            {"move_group_name": "panda_arm"},
            {"tip_link_name": "panda_hand"},
            {"publish_rate_hz": 20.0},
            {"use_sim_time": False},
        ],
    )

    servo_common_parameters = [
            {"moveit_servo": servo},
            {"robot_description": _safety_observation_urdf(urdf_content)},
            {"robot_description_semantic": srdf_content},
            {"robot_description_kinematics": kinematics},
            {"use_sim_time": False},
    ]
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        output="screen",
        parameters=servo_common_parameters,
    )

    return LaunchDescription([
        move_group_node,
        safety_observation_node,
        servo_node,
    ])
