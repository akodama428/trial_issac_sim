from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import yaml

from tomato_harvest_sim.msg.contracts import Pose3D


@dataclass(frozen=True)
class SceneLayoutConfig:
    robot_base_pose: Pose3D
    fixed_camera_pose: Pose3D
    hand_camera_pose: Pose3D
    branch_pose: Pose3D
    stem_pose: Pose3D
    tomato_pose: Pose3D
    tray_pose: Pose3D
    home_tool_pose: Pose3D
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


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _scene_config_path() -> Path:
    return _repo_root() / "config" / "scene.yaml"


def _pose3d_from_dict(data: dict[str, object]) -> Pose3D:
    return Pose3D(
        x=float(data["x"]),
        y=float(data["y"]),
        z=float(data["z"]),
        roll=float(data["roll"]),
        pitch=float(data["pitch"]),
        yaw=float(data["yaw"]),
    )


def _float_tuple(data: list[object], *, expected_len: int) -> tuple[float, ...]:
    values = tuple(float(value) for value in data)
    if len(values) != expected_len:
        raise ValueError(f"Expected tuple length {expected_len}, got {len(values)}")
    return values


@lru_cache(maxsize=1)
def load_scene_layout_config() -> SceneLayoutConfig:
    payload = yaml.safe_load(_scene_config_path().read_text(encoding="utf-8"))
    scene = payload.get("scene", {}) if isinstance(payload, dict) else {}
    if not isinstance(scene, dict):
        raise ValueError("scene config must contain a top-level 'scene' mapping")

    return SceneLayoutConfig(
        robot_base_pose=_pose3d_from_dict(scene["robot_base_pose"]),
        fixed_camera_pose=_pose3d_from_dict(scene["fixed_camera_pose"]),
        hand_camera_pose=_pose3d_from_dict(scene["hand_camera_pose"]),
        branch_pose=_pose3d_from_dict(scene["branch_pose"]),
        stem_pose=_pose3d_from_dict(scene["stem_pose"]),
        tomato_pose=_pose3d_from_dict(scene["tomato_pose"]),
        tray_pose=_pose3d_from_dict(scene["tray_pose"]),
        home_tool_pose=_pose3d_from_dict(scene["home_tool_pose"]),
        tomato_radius_m=float(scene["tomato_radius_m"]),
        tray_inner_size_m=_float_tuple(scene["tray_inner_size_m"], expected_len=3),
        tray_wall_thickness_m=float(scene["tray_wall_thickness_m"]),
        branch_size_m=_float_tuple(scene["branch_size_m"], expected_len=3),
        stem_height_m=float(scene["stem_height_m"]),
        stem_radius_m=float(scene["stem_radius_m"]),
        ground_size_m=_float_tuple(scene["ground_size_m"], expected_len=3),
        fixed_camera_focal_length_mm=float(scene["fixed_camera_focal_length_mm"]),
        fixed_camera_clipping_range_m=_float_tuple(scene["fixed_camera_clipping_range_m"], expected_len=2),
        hand_camera_focal_length_mm=float(scene["hand_camera_focal_length_mm"]),
        hand_camera_clipping_range_m=_float_tuple(scene["hand_camera_clipping_range_m"], expected_len=2),
    )
