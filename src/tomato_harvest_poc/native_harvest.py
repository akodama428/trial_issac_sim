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
    tomato_initial_world_m: tuple[float, float, float]
    tomato_radius_m: float
    hand_camera_local_offset_m: tuple[float, float, float]
    hand_camera_local_rotation_deg: tuple[float, float, float]
    hand_camera_xy_limit_m: float
    hand_camera_min_depth_m: float
    hand_camera_max_depth_m: float
    search_height_tolerance_m: float


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
        tomato_initial_world_m=(0.50, 0.00, 0.42),
        tomato_radius_m=0.01,
        hand_camera_local_offset_m=(0.0, 0.0, 0.10),
        hand_camera_local_rotation_deg=(0.0, 180.0, 0.0),
        hand_camera_xy_limit_m=0.20,
        hand_camera_min_depth_m=0.03,
        hand_camera_max_depth_m=0.80,
        search_height_tolerance_m=0.08,
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


def build_target_found_messages(
    camera_point_m: tuple[float, float, float],
    world_point_m: tuple[float, float, float],
) -> tuple[str, ...]:
    return (
        "Target is Found!",
        f"Tomato camera xyz: {format_xyz(camera_point_m)}",
        f"Tomato world xyz: {format_xyz(world_point_m)}",
    )
