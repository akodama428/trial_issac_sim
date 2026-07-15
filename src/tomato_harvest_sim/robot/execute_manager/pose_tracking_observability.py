"""Servo終端Pose Trackingを同一sequenceで追跡する観測フィールド。"""
from __future__ import annotations

from tomato_harvest_sim.msg.contracts import Pose3D


def _pose_xyz(pose: Pose3D) -> list[float]:
    return [pose.x, pose.y, pose.z]


def _pose_rpy_deg(pose: Pose3D) -> list[float]:
    return [pose.roll, pose.pitch, pose.yaw]


def pose_tracking_metric_fields(
    *,
    sequence_id: int,
    published_count: int,
    planning_frame: str,
    end_effector_frame: str,
    target: Pose3D,
    current: Pose3D,
    position_error_m: float,
    orientation_error_rad: float,
    reached: bool,
    stable_samples: int,
    servo_status: int | None,
    tf_success_count: int,
    tf_failure_count: int,
) -> dict[str, object]:
    """command、TF、6D誤差、Servo statusを1 sampleへ束ねる。"""
    return {
        "sequence_id": sequence_id,
        "published_count": published_count,
        "planning_frame": planning_frame,
        "end_effector_frame": end_effector_frame,
        "target_xyz_m": _pose_xyz(target),
        "target_rpy_deg": _pose_rpy_deg(target),
        "current_xyz_m": _pose_xyz(current),
        "current_rpy_deg": _pose_rpy_deg(current),
        "position_error_m": position_error_m,
        "orientation_error_rad": orientation_error_rad,
        "reached": reached,
        "stable_samples": stable_samples,
        "servo_status": servo_status,
        "tf_lookup_succeeded": True,
        "tf_success_count": tf_success_count,
        "tf_failure_count": tf_failure_count,
    }


def tf_lookup_failure_metric_fields(
    *,
    sequence_id: int,
    published_count: int,
    planning_frame: str,
    end_effector_frame: str,
    error: str,
    servo_status: int | None,
    tf_success_count: int,
    tf_failure_count: int,
    last_success_age_sec: float | None,
) -> dict[str, object]:
    """TF失敗を握り潰さず、command sequenceと累積状態を記録する。"""
    return {
        "sequence_id": sequence_id,
        "published_count": published_count,
        "planning_frame": planning_frame,
        "end_effector_frame": end_effector_frame,
        "servo_status": servo_status,
        "tf_lookup_succeeded": False,
        "tf_error": error,
        "tf_success_count": tf_success_count,
        "tf_failure_count": tf_failure_count,
        "last_tf_success_age_sec": last_success_age_sec,
    }
