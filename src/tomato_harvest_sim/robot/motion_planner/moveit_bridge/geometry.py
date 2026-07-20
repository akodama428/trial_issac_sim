from __future__ import annotations

import math

from tomato_harvest_sim.msg.contracts import Pose3D


def moveit_link_target_pose_from_runtime_tool_pose(
    runtime_tool_pose: Pose3D,
    *,
    link_to_tool_offset_m: tuple[float, float, float],
) -> Pose3D:
    inverse_offset_m = tuple(-value for value in link_to_tool_offset_m)
    return shift_pose_by_local_offset(runtime_tool_pose, inverse_offset_m)


def shift_pose_by_local_offset(
    pose: Pose3D,
    local_offset_m: tuple[float, float, float],
) -> Pose3D:
    offset_x, offset_y, offset_z = rotate_local_offset(local_offset_m, pose)
    return Pose3D(
        x=round(pose.x + offset_x, 6),
        y=round(pose.y + offset_y, 6),
        z=round(pose.z + offset_z, 6),
        roll=pose.roll,
        pitch=pose.pitch,
        yaw=pose.yaw,
    )


def rotate_local_offset(
    local_offset_m: tuple[float, float, float],
    pose: Pose3D,
) -> tuple[float, float, float]:
    x, y, z = local_offset_m
    roll = math.radians(pose.roll)
    pitch = math.radians(pose.pitch)
    yaw = math.radians(pose.yaw)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cy * cp * x + (cy * sp * sr - sy * cr) * y
        + (cy * sp * cr + sy * sr) * z,
        sy * cp * x + (sy * sp * sr + cy * cr) * y
        + (sy * sp * cr - cy * sr) * z,
        -sp * x + cp * sr * y + cp * cr * z,
    )


def quaternion_from_pose(pose: Pose3D) -> object:
    from geometry_msgs.msg import Quaternion

    roll = math.radians(pose.roll)
    pitch = math.radians(pose.pitch)
    yaw = math.radians(pose.yaw)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    quaternion = Quaternion()
    quaternion.w = cr * cp * cy + sr * sp * sy
    quaternion.x = sr * cp * cy - cr * sp * sy
    quaternion.y = cr * sp * cy + sr * cp * sy
    quaternion.z = cr * cp * sy - sr * sp * cy
    return quaternion
