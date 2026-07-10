from __future__ import annotations

import os
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import yaml

from tomato_harvest_sim.msg.contracts import Pose3D

# PhysX が受け付ける combine mode（PhysxSchema physxMaterial:*CombineMode の許容値）
_VALID_COMBINE_MODES = ("average", "min", "multiply", "max")


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


@dataclass(frozen=True)
class PhysicsMaterialConfig:
    """摩擦・反発の物理マテリアル設定。"""

    static_friction: float
    dynamic_friction: float
    restitution: float


@dataclass(frozen=True)
class PhysicsTuningConfig:
    """Step 1 で導入した物理チューニング一式（scene.yaml physics セクション）。

    enabled=False のとき、physics_harvest は従来どおり何も適用しない。
    環境変数 TOMATO_HARVEST_PHYSICS_TUNING=0 で A/B 比較用に強制無効化できる。
    """

    enabled: bool
    tomato_material: PhysicsMaterialConfig
    gripper_material: PhysicsMaterialConfig
    container_material: PhysicsMaterialConfig
    friction_combine_mode: str
    restitution_combine_mode: str
    tomato_contact_offset_m: float
    tomato_rest_offset_m: float
    tomato_torsional_patch_radius_m: float
    tomato_min_torsional_patch_radius_m: float
    tomato_solver_position_iterations: int
    tomato_solver_velocity_iterations: int
    # Step 2: finger drive の力制限。max_force_n=0 は「drive へ適用しない」を意味する。
    finger_drive_stiffness: float
    finger_drive_damping: float
    finger_drive_max_force_n: float


_DISABLED_MATERIAL = PhysicsMaterialConfig(
    static_friction=0.0, dynamic_friction=0.0, restitution=0.0
)

_DISABLED_TUNING = PhysicsTuningConfig(
    enabled=False,
    tomato_material=_DISABLED_MATERIAL,
    gripper_material=_DISABLED_MATERIAL,
    container_material=_DISABLED_MATERIAL,
    friction_combine_mode="average",
    restitution_combine_mode="average",
    tomato_contact_offset_m=0.0,
    tomato_rest_offset_m=0.0,
    tomato_torsional_patch_radius_m=0.0,
    tomato_min_torsional_patch_radius_m=0.0,
    tomato_solver_position_iterations=0,
    tomato_solver_velocity_iterations=0,
    finger_drive_stiffness=0.0,
    finger_drive_damping=0.0,
    finger_drive_max_force_n=0.0,
)


def _material_from_dict(data: dict[str, object]) -> PhysicsMaterialConfig:
    return PhysicsMaterialConfig(
        static_friction=float(data["static_friction"]),
        dynamic_friction=float(data["dynamic_friction"]),
        restitution=float(data["restitution"]),
    )


def _validated_combine_mode(value: object) -> str:
    mode = str(value)
    if mode not in _VALID_COMBINE_MODES:
        raise ValueError(
            f"combine mode must be one of {_VALID_COMBINE_MODES}, got {mode!r}"
        )
    return mode


def physics_tuning_from_payload(payload: dict[str, object]) -> PhysicsTuningConfig:
    """yaml payload から物理チューニング設定を組み立てる。

    physics セクションが無い場合、および環境変数キルスイッチが立っている場合は
    「適用しない」設定を返す（従来挙動の維持）。

    Raises:
        ValueError: combine mode が PhysX の許容値でない場合。
    """
    physics = payload.get("physics") if isinstance(payload, dict) else None
    if not isinstance(physics, dict):
        return _DISABLED_TUNING
    kill_switch = os.environ.get("TOMATO_HARVEST_PHYSICS_TUNING", "").strip()
    enabled = bool(physics.get("enabled", True)) and kill_switch not in {"0", "false", "False"}

    collision = physics["tomato_collision"]
    solver = physics["tomato_solver"]
    finger_drive = physics.get("finger_drive", {})
    if not isinstance(finger_drive, dict):
        finger_drive = {}
    return PhysicsTuningConfig(
        enabled=enabled,
        tomato_material=_material_from_dict(physics["tomato_material"]),
        gripper_material=_material_from_dict(physics["gripper_material"]),
        container_material=_material_from_dict(physics["container_material"]),
        friction_combine_mode=_validated_combine_mode(physics["friction_combine_mode"]),
        restitution_combine_mode=_validated_combine_mode(physics["restitution_combine_mode"]),
        tomato_contact_offset_m=float(collision["contact_offset_m"]),
        tomato_rest_offset_m=float(collision["rest_offset_m"]),
        tomato_torsional_patch_radius_m=float(collision["torsional_patch_radius_m"]),
        tomato_min_torsional_patch_radius_m=float(collision["min_torsional_patch_radius_m"]),
        tomato_solver_position_iterations=int(solver["position_iterations"]),
        tomato_solver_velocity_iterations=int(solver["velocity_iterations"]),
        finger_drive_stiffness=float(finger_drive.get("stiffness", 0.0)),
        finger_drive_damping=float(finger_drive.get("damping", 0.0)),
        finger_drive_max_force_n=float(finger_drive.get("max_force_n", 0.0)),
    )


@lru_cache(maxsize=1)
def load_physics_tuning_config() -> PhysicsTuningConfig:
    payload = yaml.safe_load(_scene_config_path().read_text(encoding="utf-8"))
    return physics_tuning_from_payload(payload if isinstance(payload, dict) else {})


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
