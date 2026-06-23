from __future__ import annotations

from dataclasses import dataclass

from .config import CameraPose, RuntimeConfig


@dataclass(frozen=True)
class IsaacSmokeScenePlan:
    camera_prim_path: str
    camera_position_m: tuple[float, float, float]
    camera_rotation_deg: tuple[float, float, float]
    required_prim_paths: tuple[str, ...]
    container_command: str
    simulation_steps: int


def build_smoke_scene_plan(config: RuntimeConfig | None = None) -> IsaacSmokeScenePlan:
    runtime_config = config or RuntimeConfig()
    camera_pose = runtime_config.scene.camera_pose
    return IsaacSmokeScenePlan(
        camera_prim_path="/World/Camera_EyeToHand",
        camera_position_m=(camera_pose.x_m, camera_pose.y_m, camera_pose.z_m),
        camera_rotation_deg=_to_isaac_rotation(camera_pose),
        required_prim_paths=(
            "/World/GroundPlane",
            "/World/SmokeCube",
            "/World/TargetTomato",
            "/World/Camera_EyeToHand",
        ),
        container_command="/isaac-sim/python.sh scripts/isaac_viewport_smoke.py",
        simulation_steps=180,
    )


def _to_isaac_rotation(camera_pose: CameraPose) -> tuple[float, float, float]:
    return (camera_pose.pitch_deg, camera_pose.roll_deg, camera_pose.yaw_deg)

