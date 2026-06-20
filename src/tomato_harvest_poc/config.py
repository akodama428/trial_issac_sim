from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class CameraPose:
    x_m: float = 0.80
    y_m: float = 0.00
    z_m: float = 1.35
    pitch_deg: float = -30.0
    yaw_deg: float = 180.0
    roll_deg: float = 0.0


@dataclass(frozen=True)
class MotionDurations:
    loading_s: float = 0.05
    approach_s: float = 0.05
    grasp_s: float = 0.05
    pull_s: float = 0.05


@dataclass(frozen=True)
class SceneConfig:
    camera_pose: CameraPose = field(default_factory=CameraPose)
    stage_items: tuple[str, ...] = (
        "/World",
        "/World/FrankaPanda",
        "/World/GreenhouseShell",
        "/World/TomatoPlant/Branch",
        "/World/TomatoPlant/Stem",
        "/World/TomatoPlant/Fruit",
        "/World/Camera_EyeToHand",
    )
    target_label: str = "Target Tomato"


@dataclass(frozen=True)
class RuntimeConfig:
    scene: SceneConfig = field(default_factory=SceneConfig)
    durations: MotionDurations = field(default_factory=MotionDurations)
    detach_break_force: float = 7.5
    pull_force: float = 9.0
    ui_host: str = "0.0.0.0"
    ui_port: int = 8080
    static_dir: Path = field(
        default_factory=lambda: Path(__file__).resolve().parents[2] / "ui"
    )

