from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class CameraViewMode(str, Enum):
    FIXED = "fixed"
    HAND = "hand"


class HarvestPhase(str, Enum):
    READY = "Ready"
    SEARCHING = "Searching"
    TARGET_FOUND = "Target Found"
    APPROACHING = "Approaching"
    GRASPING = "Grasping"
    PLACING = "Placing"
    COMPLETE = "Complete"
    STOPPED = "Stopped"
    FAILED = "Failed"


@dataclass(frozen=True)
class ScanPose:
    label: str
    dof_positions: tuple[float, ...]


@dataclass(frozen=True)
class HarvestScenarioPlan:
    home_dof_positions: tuple[float, ...]
    top_down_reference_dof_positions: tuple[float, ...]
    scan_poses: tuple[ScanPose, ...]
    grasp_pre_offset_m: tuple[float, float, float]
    grasp_offset_m: tuple[float, float, float]
    grasp_center_local_offset_m: tuple[float, float, float]
    place_pre_offset_m: tuple[float, float, float]
    place_retreat_offset_m: tuple[float, float, float]
    place_position_m: tuple[float, float, float]
    branch_center_world_m: tuple[float, float, float]
    branch_scale_m: tuple[float, float, float]
    stem_center_world_m: tuple[float, float, float]
    stem_scale_m: tuple[float, float, float]
    tomato_initial_world_m: tuple[float, float, float]
    tomato_radius_m: float
    tomato_mass_kg: float
    hand_camera_local_offset_m: tuple[float, float, float]
    hand_camera_local_rotation_deg: tuple[float, float, float]
    hand_camera_xy_limit_m: float
    hand_camera_min_depth_m: float
    hand_camera_max_depth_m: float
    search_height_tolerance_m: float
    pull_offset_m: tuple[float, float, float]
    tray_inner_size_m: tuple[float, float, float]
    tray_wall_thickness_m: float
    stem_break_force_n: float
    stem_break_torque_nm: float
    finger_contact_force_threshold_n: float
    grasp_hold_frame_count: int
    settle_linear_speed_threshold_mps: float
    settle_angular_speed_threshold_radps: float
    settle_frame_count: int
    settle_timeout_frames: int


def build_harvest_scenario_plan() -> HarvestScenarioPlan:
    home = (0.00, -0.70, 0.00, -2.20, 0.00, 1.60, 0.78, 0.040, 0.040)
    top_down_reference = (0.14, -0.40, -0.10, -1.90, 0.06, 1.80, 0.92, 0.040, 0.040)
    return HarvestScenarioPlan(
        home_dof_positions=home,
        top_down_reference_dof_positions=top_down_reference,
        scan_poses=(
            ScanPose("scan_back_left", (-2.20, -0.45, 0.05, -1.95, 0.10, 1.65, 0.55, 0.040, 0.040)),
            ScanPose("scan_left", (-1.50, -0.42, -0.05, -1.90, 0.06, 1.78, 0.82, 0.040, 0.040)),
            ScanPose("scan_front_left", (-0.75, -0.40, -0.10, -1.86, 0.06, 1.84, 0.95, 0.040, 0.040)),
            ScanPose("scan_front", (0.08, -0.36, -0.14, -1.82, 0.08, 1.90, 1.00, 0.040, 0.040)),
            ScanPose("scan_front_right", (0.42, 0.02, -0.46, -1.60, 0.03, 1.82, 0.88, 0.040, 0.040)),
            ScanPose("scan_right", (1.20, -0.10, -0.20, -1.72, 0.02, 1.74, 0.62, 0.040, 0.040)),
            ScanPose("scan_back_right", (2.05, -0.24, 0.15, -1.88, -0.04, 1.60, 0.40, 0.040, 0.040)),
        ),
        grasp_pre_offset_m=(0.000, 0.000, 0.120),
        grasp_offset_m=(0.000, 0.000, 0.000),
        grasp_center_local_offset_m=(0.000, 0.000, 0.1034),
        place_pre_offset_m=(0.000, 0.000, 0.120),
        place_retreat_offset_m=(0.000, 0.000, 0.160),
        place_position_m=(0.35, -0.45, 0.385),
        branch_center_world_m=(0.50, 0.00, 0.57),
        branch_scale_m=(0.22, 0.03, 0.03),
        stem_center_world_m=(0.50, 0.00, 0.435),
        stem_scale_m=(0.012, 0.012, 0.05),
        tomato_initial_world_m=(0.50, 0.00, 0.42),
        tomato_radius_m=0.01,
        tomato_mass_kg=0.015,
        hand_camera_local_offset_m=(0.0, 0.0, 0.10),
        hand_camera_local_rotation_deg=(0.0, 180.0, 0.0),
        hand_camera_xy_limit_m=0.60,
        hand_camera_min_depth_m=0.03,
        hand_camera_max_depth_m=1.50,
        search_height_tolerance_m=0.25,
        pull_offset_m=(0.0, 0.0, 0.10),
        tray_inner_size_m=(0.12, 0.18, 0.06),
        tray_wall_thickness_m=0.01,
        stem_break_force_n=3.5,
        stem_break_torque_nm=0.35,
        finger_contact_force_threshold_n=1.8,
        grasp_hold_frame_count=12,
        settle_linear_speed_threshold_mps=0.02,
        settle_angular_speed_threshold_radps=0.35,
        settle_frame_count=18,
        settle_timeout_frames=240,
    )


def is_target_visible(
    camera_point_m: tuple[float, float, float],
    world_point_m: tuple[float, float, float],
    *,
    expected_height_m: float,
    xy_limit_m: float,
    min_depth_m: float,
    max_depth_m: float,
    height_tolerance_m: float,
) -> bool:
    depth_m = abs(camera_point_m[2])
    if depth_m < min_depth_m or depth_m > max_depth_m:
        return False
    if abs(camera_point_m[0]) > xy_limit_m or abs(camera_point_m[1]) > xy_limit_m:
        return False
    if abs(world_point_m[2] - expected_height_m) > height_tolerance_m:
        return False
    return True


def format_xyz(point_m: tuple[float, float, float]) -> str:
    return f"({point_m[0]:.4f}, {point_m[1]:.4f}, {point_m[2]:.4f})"


def _vector_norm(vector: tuple[float, float, float]) -> float:
    return (vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2) ** 0.5


def has_dual_finger_contact(
    contact_forces_n: tuple[tuple[float, float, float], tuple[float, float, float]],
    *,
    force_threshold_n: float,
) -> bool:
    left_force, right_force = contact_forces_n
    return _vector_norm(left_force) >= force_threshold_n and _vector_norm(right_force) >= force_threshold_n


def is_object_settled(
    linear_velocity_mps: tuple[float, float, float],
    angular_velocity_radps: tuple[float, float, float],
    *,
    linear_speed_threshold_mps: float,
    angular_speed_threshold_radps: float,
) -> bool:
    return _vector_norm(linear_velocity_mps) <= linear_speed_threshold_mps and _vector_norm(
        angular_velocity_radps
    ) <= angular_speed_threshold_radps


def is_point_in_box_xy(
    point_m: tuple[float, float, float],
    center_m: tuple[float, float, float],
    size_m: tuple[float, float, float],
    *,
    margin_m: float = 0.0,
) -> bool:
    half_x = max(size_m[0] * 0.5 - margin_m, 0.0)
    half_y = max(size_m[1] * 0.5 - margin_m, 0.0)
    return abs(point_m[0] - center_m[0]) <= half_x and abs(point_m[1] - center_m[1]) <= half_y


def build_target_found_messages(
    camera_point_m: tuple[float, float, float],
    world_point_m: tuple[float, float, float],
) -> tuple[str, ...]:
    return (
        "Target is Found!",
        f"Tomato camera xyz: {format_xyz(camera_point_m)}",
        f"Tomato world xyz: {format_xyz(world_point_m)}",
    )
