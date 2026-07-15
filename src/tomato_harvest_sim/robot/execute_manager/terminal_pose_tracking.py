"""Servo終端pose trackingで共有する座標変換。"""
from __future__ import annotations

import math

from tomato_harvest_sim.msg.contracts import Pose3D

MOVEIT_LINK_TO_RUNTIME_TOOL_OFFSET_M = (0.0, 0.0, 0.0584)


def moveit_link_pose(runtime_tool_pose: Pose3D) -> Pose3D:
    """runtime tool目標をServoが制御するpanda_link8目標へ変換する。"""
    roll = math.radians(runtime_tool_pose.roll)
    pitch = math.radians(runtime_tool_pose.pitch)
    yaw = math.radians(runtime_tool_pose.yaw)
    x, y, z = tuple(-value for value in MOVEIT_LINK_TO_RUNTIME_TOOL_OFFSET_M)
    rotated_x = (
        math.cos(yaw) * math.cos(pitch) * x
        + (math.cos(yaw) * math.sin(pitch) * math.sin(roll) - math.sin(yaw) * math.cos(roll)) * y
        + (math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll)) * z
    )
    rotated_y = (
        math.sin(yaw) * math.cos(pitch) * x
        + (math.sin(yaw) * math.sin(pitch) * math.sin(roll) + math.cos(yaw) * math.cos(roll)) * y
        + (math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll)) * z
    )
    rotated_z = (
        -math.sin(pitch) * x
        + math.cos(pitch) * math.sin(roll) * y
        + math.cos(pitch) * math.cos(roll) * z
    )
    return Pose3D(
        round(runtime_tool_pose.x + rotated_x, 6),
        round(runtime_tool_pose.y + rotated_y, 6),
        round(runtime_tool_pose.z + rotated_z, 6),
        runtime_tool_pose.roll, runtime_tool_pose.pitch, runtime_tool_pose.yaw,
    )


def quaternion_from_pose(pose: Pose3D) -> tuple[float, float, float, float]:
    """degree Euler姿勢をROS quaternionへ変換する。"""
    half_roll = math.radians(pose.roll) * 0.5
    half_pitch = math.radians(pose.pitch) * 0.5
    half_yaw = math.radians(pose.yaw) * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def pose_from_transform(transform: object) -> Pose3D:
    """ROS TransformStamped相当値をPose3Dへ変換する。"""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    x, y, z, w = float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return Pose3D(
        float(translation.x), float(translation.y), float(translation.z),
        math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
    )
