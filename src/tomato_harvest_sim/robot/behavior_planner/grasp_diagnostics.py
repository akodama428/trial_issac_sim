"""Pure calculations for grasp-evaluation observability."""
from __future__ import annotations

import math
from dataclasses import dataclass

from tomato_harvest_sim.msg.contracts import Pose3D, SceneSnapshot


@dataclass(frozen=True)
class GraspPoseError:
    position_xyz_m: tuple[float, float, float]
    position_norm_m: float
    orientation_rpy_rad: tuple[float, float, float]


def _wrapped_delta(target: float, actual: float) -> float:
    return (target - actual + math.pi) % (2.0 * math.pi) - math.pi


def calculate_pose_error(actual: Pose3D, target: Pose3D) -> GraspPoseError:
    position = (target.x - actual.x, target.y - actual.y, target.z - actual.z)
    orientation = (
        _wrapped_delta(target.roll, actual.roll),
        _wrapped_delta(target.pitch, actual.pitch),
        _wrapped_delta(target.yaw, actual.yaw),
    )
    return GraspPoseError(position, math.sqrt(sum(value * value for value in position)), orientation)


def metric_payload(snapshot: SceneSnapshot, *, phase: str, sample_kind: str,
                   target_pose: Pose3D | None = None) -> dict[str, object] | None:
    target = target_pose or snapshot.target_tool_pose
    if target is None:
        return None
    error = calculate_pose_error(snapshot.robot_tool_pose, target)
    return {
        "event": "grasp_evaluation_diagnostic",
        "phase": phase,
        "sample_kind": sample_kind,
        "position_error_x_m": error.position_xyz_m[0],
        "position_error_y_m": error.position_xyz_m[1],
        "position_error_z_m": error.position_xyz_m[2],
        "position_error_norm_m": error.position_norm_m,
        "orientation_error_roll_rad": error.orientation_rpy_rad[0],
        "orientation_error_pitch_rad": error.orientation_rpy_rad[1],
        "orientation_error_yaw_rad": error.orientation_rpy_rad[2],
        "left_finger_contact": snapshot.left_finger_contact,
        "right_finger_contact": snapshot.right_finger_contact,
        "left_finger_force_n": snapshot.left_finger_force_n,
        "right_finger_force_n": snapshot.right_finger_force_n,
        "gripper_closed": snapshot.gripper_closed,
        "tomato_status": snapshot.tomato_status.value,
        "grasp_result_reason": snapshot.grasp_result_reason,
    }
