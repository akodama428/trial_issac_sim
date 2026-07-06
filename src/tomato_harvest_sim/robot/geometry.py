from __future__ import annotations

import math

from tomato_harvest_sim.msg.contracts import Pose3D


def world_point_to_local(world_pose: Pose3D, frame_pose: Pose3D) -> Pose3D:
    dx = world_pose.x - frame_pose.x
    dy = world_pose.y - frame_pose.y
    dz = world_pose.z - frame_pose.z

    rotation = _rotation_matrix_xyz_deg(frame_pose.roll, frame_pose.pitch, frame_pose.yaw)
    local_x = rotation[0][0] * dx + rotation[1][0] * dy + rotation[2][0] * dz
    local_y = rotation[0][1] * dx + rotation[1][1] * dy + rotation[2][1] * dz
    local_z = rotation[0][2] * dx + rotation[1][2] * dy + rotation[2][2] * dz
    return Pose3D(local_x, local_y, local_z, 0.0, 0.0, 0.0)


def _rotation_matrix_xyz_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[tuple[float, ...], ...]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)

    rx = (
        (1.0, 0.0, 0.0),
        (0.0, cx, -sx),
        (0.0, sx, cx),
    )
    ry = (
        (cy, 0.0, sy),
        (0.0, 1.0, 0.0),
        (-sy, 0.0, cy),
    )
    rz = (
        (cz, -sz, 0.0),
        (sz, cz, 0.0),
        (0.0, 0.0, 1.0),
    )
    return _matmul(_matmul(rx, ry), rz)


def _matmul(left: tuple[tuple[float, ...], ...], right: tuple[tuple[float, ...], ...]) -> tuple[tuple[float, ...], ...]:
    rows = []
    for row_index in range(3):
        row = []
        for col_index in range(3):
            row.append(sum(left[row_index][k] * right[k][col_index] for k in range(3)))
        rows.append(tuple(row))
    return tuple(rows)
