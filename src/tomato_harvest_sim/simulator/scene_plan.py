from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.api.contracts import Pose3D
from tomato_harvest_sim.simulator.scene_config import load_scene_layout_config
from tomato_harvest_sim.simulator.scene_runtime import IsaacSceneRuntime


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
class ReviewScenePlan:
    robot_prim_path: str
    fixed_camera_prim_path: str
    hand_camera_mount_prim_suffix: str
    hand_camera_prim_name: str
    branch_prim_path: str
    stem_prim_path: str
    tomato_prim_path: str
    tray_prim_path: str
    robot_tool_proxy_prim_path: str
    pregrasp_marker_prim_path: str
    ground_prim_path: str
    world_prim_path: str
    robot_base_pose: Pose3D
    fixed_camera_pose: Pose3D
    hand_camera_pose: Pose3D
    branch_pose: Pose3D
    stem_pose: Pose3D
    tomato_pose: Pose3D
    tray_pose: Pose3D
    robot_tool_pose: Pose3D
    pregrasp_pose: Pose3D | None
    tomato_radius_m: float
    tray_inner_size_m: tuple[float, float, float]
    tray_wall_thickness_m: float
    branch_size_m: tuple[float, float, float]
    stem_height_m: float
    stem_radius_m: float
    ground_size_m: tuple[float, float, float]
    fixed_camera_focal_length_mm: float
    fixed_camera_clipping_range_m: tuple[float, float]
    hand_camera_focal_length_mm: float
    hand_camera_clipping_range_m: tuple[float, float]
    required_prim_paths: tuple[str, ...]
    light_specs: tuple[StageLightSpec, ...]


def build_review_scene_plan() -> ReviewScenePlan:
    layout = load_scene_layout_config()
    snapshot = IsaacSceneRuntime().boot()
    return ReviewScenePlan(
        robot_prim_path="/World/FrankaPanda",
        fixed_camera_prim_path="/World/Camera_Fixed",
        hand_camera_mount_prim_suffix="panda_hand",
        hand_camera_prim_name="HandCamera",
        branch_prim_path="/World/TomatoBranch",
        stem_prim_path="/World/TomatoStem",
        tomato_prim_path="/World/TargetTomato",
        tray_prim_path="/World/PlaceTray",
        robot_tool_proxy_prim_path="/World/RobotToolProxy",
        pregrasp_marker_prim_path="/World/PreGraspMarker",
        ground_prim_path="/World/GroundPlane",
        world_prim_path="/World",
        robot_base_pose=snapshot.robot_base_pose,
        fixed_camera_pose=snapshot.fixed_camera_pose,
        hand_camera_pose=snapshot.hand_camera_pose,
        branch_pose=snapshot.branch_pose,
        stem_pose=snapshot.stem_pose,
        tomato_pose=snapshot.tomato_pose,
        tray_pose=snapshot.tray_pose,
        robot_tool_pose=snapshot.robot_tool_pose,
        pregrasp_pose=snapshot.pregrasp_pose,
        tomato_radius_m=layout.tomato_radius_m,
        tray_inner_size_m=layout.tray_inner_size_m,
        tray_wall_thickness_m=layout.tray_wall_thickness_m,
        branch_size_m=layout.branch_size_m,
        stem_height_m=layout.stem_height_m,
        stem_radius_m=layout.stem_radius_m,
        ground_size_m=layout.ground_size_m,
        fixed_camera_focal_length_mm=layout.fixed_camera_focal_length_mm,
        fixed_camera_clipping_range_m=layout.fixed_camera_clipping_range_m,
        hand_camera_focal_length_mm=layout.hand_camera_focal_length_mm,
        hand_camera_clipping_range_m=layout.hand_camera_clipping_range_m,
        required_prim_paths=(
            "/World",
            "/World/GroundPlane",
            "/World/FrankaPanda",
            "/World/Camera_Fixed",
            "/World/TomatoBranch",
            "/World/TomatoStem",
            "/World/TargetTomato",
            "/World/PlaceTray",
            "/World/RobotToolProxy",
        ),
        light_specs=build_default_light_specs(),
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
