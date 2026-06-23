from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class FrankaMotionStep:
    label: str
    dof_positions: tuple[float, ...]


@dataclass(frozen=True)
class StageLightSpec:
    prim_path: str
    kind: str
    intensity: float
    translate_m: tuple[float, float, float] | None = None
    rotate_deg: tuple[float, float, float] | None = None
    radius_m: float | None = None
    color_rgb: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class FrankaSmokePlan:
    robot_prim_path: str
    target_tomato_prim_path: str
    debug_camera_prim_path: str
    hand_camera_prim_name: str
    container_command: str
    franka_asset_relative_path: str
    franka_urdf_relative_path: str
    frames_per_step: int
    simulation_dt_s: float
    motion_steps: tuple[FrankaMotionStep, ...]
    tomato_radius_m: float
    tomato_highlight_radius_m: float
    debug_camera_position_m: tuple[float, float, float]
    debug_camera_rotation_deg: tuple[float, float, float]
    hand_camera_local_offset_m: tuple[float, float, float]
    hand_camera_local_rotation_deg: tuple[float, float, float]
    centering_preferred_depth_m: float
    centering_position_tolerance_m: float
    centering_orientation_tolerance_rad: float
    centering_interpolation_frames: int


def build_franka_smoke_plan() -> FrankaSmokePlan:
    return FrankaSmokePlan(
        robot_prim_path="/World/FrankaPanda",
        target_tomato_prim_path="/World/TargetTomato",
        debug_camera_prim_path="/World/Camera_Debug",
        hand_camera_prim_name="HandCamera",
        container_command="/isaac-sim/python.sh scripts/isaac_franka_smoke.py",
        franka_asset_relative_path="/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
        franka_urdf_relative_path=(
            "/exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description/robots/panda_arm_hand.urdf"
        ),
        frames_per_step=120,
        simulation_dt_s=1.0 / 60.0,
        motion_steps=(
            FrankaMotionStep(
                label="home_open",
                dof_positions=(0.00, -0.70, 0.00, -2.20, 0.00, 1.60, 0.78, 0.040, 0.040),
            ),
            FrankaMotionStep(
                label="pre_grasp_open",
                dof_positions=(0.14, -0.40, -0.10, -1.90, 0.06, 1.80, 0.92, 0.040, 0.040),
            ),
            FrankaMotionStep(
                label="grasp_closed",
                dof_positions=(0.18, -0.32, -0.12, -1.82, 0.08, 1.87, 0.96, 0.010, 0.010),
            ),
            FrankaMotionStep(
                label="pull_closed",
                dof_positions=(0.28, -0.18, -0.16, -1.55, 0.14, 1.98, 1.04, 0.010, 0.010),
            ),
        ),
        tomato_radius_m=0.01,
        tomato_highlight_radius_m=0.013,
        debug_camera_position_m=(2.2, -2.2, 1.6),
        debug_camera_rotation_deg=(60.0, 0.0, 45.0),
        hand_camera_local_offset_m=(0.0, 0.0, 0.06),
        hand_camera_local_rotation_deg=(0.0, 180.0, 0.0),
        centering_preferred_depth_m=0.12,
        centering_position_tolerance_m=0.01,
        centering_orientation_tolerance_rad=0.15,
        centering_interpolation_frames=90,
    )


def select_motion_step(*, frame: int, frames_per_step: int, motion_step_count: int) -> int:
    if frames_per_step <= 0:
        raise ValueError("frames_per_step must be positive")
    if motion_step_count <= 0:
        raise ValueError("motion_step_count must be positive")
    return (frame // frames_per_step) % motion_step_count


def interpolate_dof_positions(
    start: tuple[float, ...],
    end: tuple[float, ...],
    *,
    progress: float,
) -> tuple[float, ...]:
    clamped_progress = min(max(progress, 0.0), 1.0)
    return tuple(
        start[index] + (end[index] - start[index]) * clamped_progress
        for index in range(len(start))
    )


def build_default_light_specs() -> tuple[StageLightSpec, ...]:
    return (
        StageLightSpec(
            prim_path="/World/KeyLight",
            kind="distant",
            intensity=900.0,
            rotate_deg=(-35.0, 0.0, 25.0),
            color_rgb=(1.0, 0.98, 0.95),
        ),
        StageLightSpec(
            prim_path="/World/FillLight",
            kind="sphere",
            intensity=35000.0,
            translate_m=(1.8, -1.6, 1.6),
            radius_m=0.25,
            color_rgb=(0.85, 0.90, 1.0),
        ),
        StageLightSpec(
            prim_path="/World/BackLight",
            kind="sphere",
            intensity=12000.0,
            translate_m=(-1.2, 1.0, 1.8),
            radius_m=0.18,
            color_rgb=(1.0, 0.95, 0.90),
        ),
    )


def compute_look_at_rotate_xyz_deg(
    camera_position_m: tuple[float, float, float],
    target_position_m: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = target_position_m[0] - camera_position_m[0]
    dy = target_position_m[1] - camera_position_m[1]
    dz = target_position_m[2] - camera_position_m[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length == 0.0:
        raise ValueError("camera_position_m and target_position_m must differ")

    nx = dx / length
    ny = dy / length
    nz = dz / length

    rotate_x_deg = math.degrees(math.asin(max(-1.0, min(1.0, ny))))
    rotate_y_deg = math.degrees(math.atan2(-nx, -nz))
    return (rotate_x_deg, rotate_y_deg, 0.0)


def build_camera_look_at_rows(
    camera_position_m: tuple[float, float, float],
    target_position_m: tuple[float, float, float],
    *,
    world_up_m: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> tuple[tuple[float, float, float, float], ...]:
    eye_x, eye_y, eye_z = camera_position_m
    target_x, target_y, target_z = target_position_m
    up_x, up_y, up_z = world_up_m

    forward = _normalize((target_x - eye_x, target_y - eye_y, target_z - eye_z))
    right = _normalize(_cross(forward, (up_x, up_y, up_z)))
    corrected_up = _cross(right, forward)

    return (
        (right[0], right[1], right[2], 0.0),
        (corrected_up[0], corrected_up[1], corrected_up[2], 0.0),
        (-forward[0], -forward[1], -forward[2], 0.0),
        (eye_x, eye_y, eye_z, 1.0),
    )


def compute_camera_center_error(camera_point_m: tuple[float, float, float]) -> float:
    return math.hypot(camera_point_m[0], camera_point_m[1])


def compute_centering_camera_position(
    current_camera_position_m: tuple[float, float, float],
    target_world_position_m: tuple[float, float, float],
    *,
    preferred_depth_m: float,
) -> tuple[float, float, float]:
    direction_x = target_world_position_m[0] - current_camera_position_m[0]
    direction_y = target_world_position_m[1] - current_camera_position_m[1]
    direction_z = target_world_position_m[2] - current_camera_position_m[2]
    direction_length = math.sqrt(direction_x * direction_x + direction_y * direction_y + direction_z * direction_z)
    if direction_length == 0.0:
        raise ValueError("current_camera_position_m and target_world_position_m must differ")

    unit_x = direction_x / direction_length
    unit_y = direction_y / direction_length
    unit_z = direction_z / direction_length
    return (
        target_world_position_m[0] - unit_x * preferred_depth_m,
        target_world_position_m[1] - unit_y * preferred_depth_m,
        target_world_position_m[2] - unit_z * preferred_depth_m,
    )


def compute_tomato_centering_score(
    camera_point_m: tuple[float, float, float],
    *,
    preferred_depth_m: float = 0.25,
) -> float:
    score = compute_camera_center_error(camera_point_m)
    if camera_point_m[2] >= -0.01:
        score += 10.0 + camera_point_m[2]
    score += 0.05 * abs(abs(camera_point_m[2]) - preferred_depth_m)
    return score


def _cross(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(component * component for component in vector))
    if length == 0.0:
        raise ValueError("vector must be non-zero")
    return tuple(component / length for component in vector)
