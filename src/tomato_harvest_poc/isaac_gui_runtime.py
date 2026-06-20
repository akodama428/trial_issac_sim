from __future__ import annotations

import os
import time
from dataclasses import dataclass
from pathlib import Path

from .config import RuntimeConfig
from .model import SimulationStatus, Snapshot
from .service import HarvestSimulationService

ISAAC_SIM_ROOT = Path(os.environ.get("ISAAC_SIM_ROOT", "/isaac-sim"))


@dataclass(frozen=True)
class SceneVisuals:
    tool_position: tuple[float, float, float]
    fruit_position: tuple[float, float, float]
    stem_anchor_position: tuple[float, float, float]
    left_finger_position: tuple[float, float, float]
    right_finger_position: tuple[float, float, float]
    gripper_closed: bool
    fruit_color: tuple[float, float, float]
    tool_color: tuple[float, float, float]


def build_appframework_argv(*, headless: bool = False) -> list[str]:
    argv = [
        "--empty",
        "--ext-folder",
        str(ISAAC_SIM_ROOT / "exts"),
        "--ext-folder",
        str(ISAAC_SIM_ROOT / "extscache"),
        "--ext-folder",
        str(ISAAC_SIM_ROOT / "apps"),
        "--/app/asyncRendering=False",
        "--/app/fastShutdown=True",
        "--/app/hangDetector/timeout=300",
        "--/app/file/ignoreUnsavedStage=true",
        "--/renderer/asyncInit=true",
        "--/persistent/renderer/startupMessageDisplayed=true",
        "--enable",
        "omni.usd",
        "--enable",
        "omni.kit.uiapp",
        "--enable",
        "omni.kit.mainwindow",
        "--enable",
        "omni.kit.viewport.window",
        "--enable",
        "omni.hydra.usdrt_delegate",
        "--enable",
        "omni.hydra.rtx",
    ]
    if headless:
        argv.append("--no-window")
    return argv


def compute_scene_visuals(snapshot: Snapshot) -> SceneVisuals:
    rest_tool = (0.18, -0.42, 0.36)
    approach_tool = (0.46, -0.06, 0.55)
    pull_tool = (0.70, -0.18, 0.68)
    stem_anchor = (0.55, 0.00, 0.55)

    progress = min(max(snapshot.visual.arm_progress, 0.0), 1.0)
    if snapshot.status in {SimulationStatus.APPROACHING, SimulationStatus.GRASPING}:
        tool_position = _lerp_point(rest_tool, approach_tool, progress)
    elif snapshot.status in {SimulationStatus.PULLING, SimulationStatus.DETACHED, SimulationStatus.FAILED}:
        tool_position = _lerp_point(approach_tool, pull_tool, progress)
    else:
        tool_position = rest_tool

    fruit_position = stem_anchor
    if snapshot.visual.tomato_detached:
        fruit_position = (
            tool_position[0] + 0.04,
            tool_position[1],
            tool_position[2] - 0.03,
        )

    finger_gap = 0.018 if snapshot.visual.gripper_closed else 0.055
    left_finger = (tool_position[0] + 0.02, tool_position[1] - finger_gap, tool_position[2])
    right_finger = (tool_position[0] + 0.02, tool_position[1] + finger_gap, tool_position[2])

    tool_color = (0.15, 0.39, 0.92)
    if snapshot.status == SimulationStatus.FAILED:
        tool_color = (0.86, 0.16, 0.16)
    if snapshot.status == SimulationStatus.DETACHED:
        tool_color = (0.09, 0.50, 0.25)

    fruit_color = (0.86, 0.31, 0.22) if snapshot.visual.target_highlighted else (0.72, 0.22, 0.16)
    return SceneVisuals(
        tool_position=tool_position,
        fruit_position=fruit_position,
        stem_anchor_position=stem_anchor,
        left_finger_position=left_finger,
        right_finger_position=right_finger,
        gripper_closed=snapshot.visual.gripper_closed,
        fruit_color=fruit_color,
        tool_color=tool_color,
    )


class IsaacGuiRuntime:
    def __init__(self, config: RuntimeConfig, service: HarvestSimulationService) -> None:
        self._config = config
        self._service = service
        self._last_snapshot: Snapshot | None = None
        self._app = None
        self._context = None
        self._stage = None
        self._fruit_translate = None
        self._stem_translate = None
        self._tool_translate = None
        self._finger_left_translate = None
        self._finger_right_translate = None
        self._fruit_display_color = None
        self._tool_display_color = None
        self._highlight_scale = None

    def run(self) -> None:
        from isaacsim import AppFramework
        import omni.kit.app

        self._app = AppFramework("tomato_harvest_poc_gui", build_appframework_argv())
        self._build_scene()
        kit_app = omni.kit.app.get_app()
        while kit_app.is_running():
            self._app.update()
            self._sync_snapshot()
            time.sleep(0.01)

    def close(self) -> None:
        if self._app is not None:
            self._app.close()
            self._app = None

    def _build_scene(self) -> None:
        import omni.usd
        from pxr import Gf, UsdGeom

        self._context = omni.usd.get_context()
        self._context.new_stage()
        self._stage = self._context.get_stage()
        stage = self._stage

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        _define_xform(stage, "/World")
        _define_xform(stage, "/World/TomatoPlant")
        _add_ground_plane(stage)
        _add_lighting(stage)
        _add_franka_if_available(stage)
        self._stem_translate = _add_stem(stage, "/World/TomatoPlant/Stem")
        _add_branch(stage, "/World/TomatoPlant/Branch")
        self._fruit_translate, self._fruit_display_color = _add_fruit(stage, "/World/TomatoPlant/Fruit")
        self._highlight_scale = _add_target_highlight(stage, "/World/TomatoPlant/FruitHighlight")
        self._tool_translate, self._tool_display_color = _add_tool(stage, "/World/ToolCenter")
        self._finger_left_translate = _add_finger(stage, "/World/ToolCenter/LeftFinger")
        self._finger_right_translate = _add_finger(stage, "/World/ToolCenter/RightFinger")
        _add_camera(stage, self._config)
        self._sync_snapshot(force=True)

    def _sync_snapshot(self, *, force: bool = False) -> None:
        snapshot = self._service.get_snapshot()
        if not force and snapshot == self._last_snapshot:
            return

        from pxr import Gf

        visuals = compute_scene_visuals(snapshot)
        self._stem_translate.Set(Gf.Vec3d(*visuals.stem_anchor_position))
        self._fruit_translate.Set(Gf.Vec3d(*visuals.fruit_position))
        self._tool_translate.Set(Gf.Vec3d(*visuals.tool_position))
        self._finger_left_translate.Set(Gf.Vec3d(*visuals.left_finger_position))
        self._finger_right_translate.Set(Gf.Vec3d(*visuals.right_finger_position))
        self._fruit_display_color.Set([visuals.fruit_color])
        self._tool_display_color.Set([visuals.tool_color])
        highlight_scale = 1.4 if snapshot.visual.target_highlighted else 0.0
        self._highlight_scale.Set(Gf.Vec3f(highlight_scale, highlight_scale, highlight_scale))
        self._last_snapshot = snapshot


def _lerp_point(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    progress: float,
) -> tuple[float, float, float]:
    return tuple(start[index] + (end[index] - start[index]) * progress for index in range(3))


def _define_xform(stage: object, prim_path: str) -> object:
    from pxr import UsdGeom

    return UsdGeom.Xform.Define(stage, prim_path)


def _add_ground_plane(stage: object) -> None:
    from pxr import Gf, UsdGeom

    ground = UsdGeom.Cube.Define(stage, "/World/GroundPlane")
    ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.025))
    ground.AddScaleOp().Set(Gf.Vec3f(2.4, 2.4, 0.025))
    ground.CreateDisplayColorAttr([(0.55, 0.60, 0.55)])


def _add_lighting(stage: object) -> None:
    from pxr import Gf, UsdLux

    key = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    key.CreateIntensityAttr(500.0)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 35.0))

    fill = UsdLux.SphereLight.Define(stage, "/World/FillLight")
    fill.CreateIntensityAttr(1800.0)
    fill.CreateRadiusAttr(0.25)
    fill.AddTranslateOp().Set(Gf.Vec3d(0.2, -0.7, 1.2))


def _add_franka_if_available(stage: object) -> None:
    from pxr import Gf, UsdGeom

    asset_root = _find_franka_asset_root()
    franka = UsdGeom.Xform.Define(stage, "/World/FrankaPanda")
    franka.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.25, 0.0))
    if asset_root is not None:
        franka.GetPrim().GetReferences().AddReference(str(asset_root))


def _find_franka_asset_root() -> Path | None:
    candidates = [
        ISAAC_SIM_ROOT / "assets" / "Isaac" / "Robots" / "Franka" / "franka.usd",
        ISAAC_SIM_ROOT / "extscache" / "isaacsim.asset.browser-0.1.7+lx64" / "data" / "Isaac" / "Robots" / "Franka" / "franka.usd",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def _add_branch(stage: object, prim_path: str) -> None:
    from pxr import Gf, UsdGeom

    branch = UsdGeom.Cube.Define(stage, prim_path)
    branch.AddTranslateOp().Set(Gf.Vec3d(0.55, 0.0, 0.72))
    branch.AddScaleOp().Set(Gf.Vec3f(0.24, 0.03, 0.03))
    branch.CreateDisplayColorAttr([(0.34, 0.52, 0.22)])


def _add_stem(stage: object, prim_path: str) -> object:
    from pxr import Gf, UsdGeom

    stem = UsdGeom.Cylinder.Define(stage, prim_path)
    stem.GetHeightAttr().Set(0.18)
    stem.GetRadiusAttr().Set(0.012)
    stem.CreateAxisAttr("Z")
    translate = stem.AddTranslateOp()
    translate.Set(Gf.Vec3d(0.55, 0.0, 0.55))
    stem.CreateDisplayColorAttr([(0.36, 0.59, 0.19)])
    return translate


def _add_fruit(stage: object, prim_path: str) -> tuple[object, object]:
    from pxr import Gf, UsdGeom

    fruit = UsdGeom.Sphere.Define(stage, prim_path)
    fruit.GetRadiusAttr().Set(0.08)
    translate = fruit.AddTranslateOp()
    translate.Set(Gf.Vec3d(0.55, 0.0, 0.55))
    display_color = fruit.CreateDisplayColorAttr([(0.86, 0.31, 0.22)])
    return translate, display_color


def _add_target_highlight(stage: object, prim_path: str) -> object:
    from pxr import Gf, UsdGeom

    highlight = UsdGeom.Sphere.Define(stage, prim_path)
    highlight.GetRadiusAttr().Set(0.09)
    highlight.AddTranslateOp().Set(Gf.Vec3d(0.55, 0.0, 0.55))
    scale = highlight.AddScaleOp()
    scale.Set(Gf.Vec3f(1.4, 1.4, 1.4))
    highlight.CreateDisplayColorAttr([(0.98, 0.82, 0.12)])
    return scale


def _add_tool(stage: object, prim_path: str) -> tuple[object, object]:
    from pxr import Gf, UsdGeom

    tool = UsdGeom.Sphere.Define(stage, prim_path)
    tool.GetRadiusAttr().Set(0.03)
    translate = tool.AddTranslateOp()
    translate.Set(Gf.Vec3d(0.18, -0.42, 0.36))
    display_color = tool.CreateDisplayColorAttr([(0.15, 0.39, 0.92)])
    return translate, display_color


def _add_finger(stage: object, prim_path: str) -> object:
    from pxr import Gf, UsdGeom

    finger = UsdGeom.Cube.Define(stage, prim_path)
    translate = finger.AddTranslateOp()
    translate.Set(Gf.Vec3d(0.18, -0.47, 0.36))
    finger.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.04))
    finger.CreateDisplayColorAttr([(0.08, 0.11, 0.16)])
    return translate


def _add_camera(stage: object, config: RuntimeConfig) -> None:
    from pxr import Gf, UsdGeom

    pose = config.scene.camera_pose
    camera = UsdGeom.Camera.Define(stage, "/World/Camera_EyeToHand")
    camera.AddTranslateOp().Set(Gf.Vec3d(pose.x_m, pose.y_m, pose.z_m))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(pose.pitch_deg, pose.yaw_deg, pose.roll_deg))
    camera.GetFocalLengthAttr().Set(24.0)
