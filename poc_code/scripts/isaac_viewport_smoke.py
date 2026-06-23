#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from tomato_harvest_poc.isaac_smoke import build_smoke_scene_plan  # noqa: E402

ISAAC_SIM_ROOT = Path(os.environ.get("ISAAC_SIM_ROOT", "/isaac-sim"))


@dataclass(frozen=True)
class ProxyRobotPose:
    wrist_position: tuple[float, float, float]
    left_finger_position: tuple[float, float, float]
    right_finger_position: tuple[float, float, float]
    shoulder_rotation_deg: tuple[float, float, float]
    forearm_rotation_deg: tuple[float, float, float]


@dataclass
class ProxyRobotBindings:
    shoulder_rotate: object
    forearm_rotate: object
    wrist_translate: object
    left_finger_translate: object
    right_finger_translate: object


@dataclass
class SmokeSceneBindings:
    camera_prim_path: str
    proxy_robot: ProxyRobotBindings | None


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Isaac Sim viewport smoke test")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without a native Isaac Sim window.",
    )
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=0.0,
        help="In GUI mode, close automatically after the given seconds. 0 means keep running.",
    )
    parser.add_argument(
        "--with-franka",
        action="store_true",
        help="Also load the Franka USD asset. Leave disabled to verify the local viewport first.",
    )
    parser.add_argument(
        "--enable-livestream",
        action="store_true",
        help="Enable livestream extensions when SimulationApp is used in headless mode.",
    )
    parser.add_argument(
        "--use-eye-to-hand-camera",
        action="store_true",
        help="Switch the active 3D viewport to the authored eye-to-hand camera.",
    )
    return parser.parse_args(argv)


def build_appframework_argv(*, headless: bool) -> list[str]:
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
        "--/app/viewport/defaultCamPos/x=1.6",
        "--/app/viewport/defaultCamPos/y=-1.6",
        "--/app/viewport/defaultCamPos/z=1.1",
        "--/renderer/asyncInit=true",
        "--/persistent/renderer/startupMessageDisplayed=true",
        "--enable",
        "omni.usd",
        "--enable",
        "omni.kit.uiapp",
        "--enable",
        "omni.kit.mainwindow",
        "--enable",
        "omni.kit.manipulator.camera",
        "--enable",
        "omni.kit.manipulator.prim",
        "--enable",
        "omni.kit.manipulator.selection",
        "--enable",
        "omni.kit.viewport.actions",
        "--enable",
        "omni.kit.viewport.legacy_gizmos",
        "--enable",
        "omni.kit.viewport.window",
        "--enable",
        "omni.kit.viewport.utility",
        "--enable",
        "omni.kit.window.status_bar",
        "--enable",
        "omni.kit.window.toolbar",
        "--enable",
        "omni.hydra.usdrt_delegate",
        "--enable",
        "omni.hydra.rtx",
    ]
    if headless:
        argv.append("--no-window")
    return argv


def choose_robot_visual(*, with_franka_requested: bool, franka_asset_available: bool) -> tuple[bool, str]:
    if with_franka_requested and franka_asset_available:
        return True, "Loading Franka USD asset."
    if with_franka_requested and not franka_asset_available:
        return False, "Franka asset root is unavailable; showing robot proxy instead."
    return False, "Skipping Franka asset lookup for fast local viewport startup; showing robot proxy."


def compute_proxy_robot_pose(elapsed_seconds: float) -> ProxyRobotPose:
    reach_phase = 0.5 + 0.5 * math.sin(elapsed_seconds * 1.2)
    gripper_phase = 0.5 + 0.5 * math.sin(elapsed_seconds * 2.4)

    wrist_x = 0.42 + 0.11 * reach_phase
    wrist_y = -0.05 + 0.06 * math.sin(elapsed_seconds * 0.6)
    wrist_z = 0.50 + 0.08 * math.cos(elapsed_seconds * 1.2)

    finger_gap = 0.018 + 0.038 * gripper_phase
    wrist_position = (wrist_x, wrist_y, wrist_z)
    return ProxyRobotPose(
        wrist_position=wrist_position,
        left_finger_position=(wrist_x + 0.07, wrist_y - finger_gap, wrist_z - 0.01),
        right_finger_position=(wrist_x + 0.07, wrist_y + finger_gap, wrist_z - 0.01),
        shoulder_rotation_deg=(0.0, 25.0 + 18.0 * reach_phase, -34.0 + 8.0 * math.sin(elapsed_seconds * 0.8)),
        forearm_rotation_deg=(10.0, 44.0 + 24.0 * reach_phase, -28.0 + 12.0 * math.cos(elapsed_seconds * 0.9)),
    )


def main() -> int:
    args = parse_args()
    if not args.headless and not os.environ.get("DISPLAY"):
        print(
            "DISPLAY is not set. Recreate the debug container with X11 access or run with --headless.",
            file=sys.stderr,
        )
        return 2

    print("Launching Isaac Sim smoke test...")
    print(f"  headless={args.headless}")
    print(f"  with_franka={args.with_franka}")
    print(f"  enable_livestream={args.enable_livestream}")
    print(f"  use_eye_to_hand_camera={args.use_eye_to_hand_camera}")

    if args.headless:
        return _run_headless_smoke(args)
    return _run_gui_smoke(args)


def _run_gui_smoke(args: argparse.Namespace) -> int:
    from isaacsim import AppFramework

    app = AppFramework("tomato_harvest_viewport_smoke", build_appframework_argv(headless=False))
    print("AppFramework initialized.")
    print("Initial RTX shader warm-up can take 2-3 minutes on the first GUI launch.")

    try:
        import omni.kit.app

        scene = _build_scene(with_franka=args.with_franka, use_viewport_api=False)
        _wait_for_first_frame(max_frames=240)
        if args.use_eye_to_hand_camera:
            _try_set_active_camera(scene.camera_prim_path)
        else:
            print("Using free viewport camera. Press F after selecting /World/TargetTomato if you want to frame the target.")
        if scene.proxy_robot is not None:
            print("Robot proxy idle motion is enabled.")
        print("Viewport is running. Close the Isaac Sim window or press Ctrl+C in this terminal to exit.")
        if args.timeout_seconds > 0:
            print(f"GUI will close automatically after {args.timeout_seconds:.1f} seconds.")

        deadline = time.time() + args.timeout_seconds if args.timeout_seconds > 0 else None
        kit_app = omni.kit.app.get_app()
        animation_start = time.monotonic()
        while kit_app.is_running():
            app.update()
            if scene.proxy_robot is not None:
                _update_proxy_robot(scene.proxy_robot, elapsed_seconds=time.monotonic() - animation_start)
            if deadline is not None and time.time() >= deadline:
                print("Timeout reached. Closing viewport smoke test.")
                break
    except KeyboardInterrupt:
        print("Interrupted by user. Closing viewport smoke test.")
    finally:
        app.close()
    return 0


def _run_headless_smoke(args: argparse.Namespace) -> int:
    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": True,
            "renderer": "MinimalRendering",
            "sync_loads": False,
            "anti_aliasing": 0,
        }
    )

    try:
        print("SimulationApp initialized.")
        _enable_livestream_if_requested(args.enable_livestream)
        _build_scene(with_franka=args.with_franka, use_viewport_api=False)
        _pump_updates(simulation_app.update, frame_count=build_smoke_scene_plan().simulation_steps)
    finally:
        simulation_app.close()
    return 0


def _enable_livestream_if_requested(enabled: bool) -> None:
    if not enabled:
        return

    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    for extension_name in (
        "omni.kit.livestream.webrtc",
        "omni.services.streamclient.webrtc",
    ):
        if manager.get_extension_id_by_module(extension_name):
            manager.set_extension_enabled_immediate(extension_name, True)


def _build_scene(*, with_franka: bool, use_viewport_api: bool) -> SmokeSceneBindings:
    import omni.usd
    from pxr import Gf, UsdGeom

    plan = build_smoke_scene_plan()
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    _ensure_world_xform(stage)
    _add_ground_plane(stage)
    _add_lighting(stage)
    franka_asset_root = _find_franka_asset_root()
    load_franka, robot_message = choose_robot_visual(
        with_franka_requested=with_franka,
        franka_asset_available=franka_asset_root is not None,
    )
    print(robot_message)
    proxy_robot = None
    if load_franka:
        _add_franka_reference(stage, franka_asset_root)
    else:
        proxy_robot = _add_robot_proxy(stage)

    _add_colored_cube(
        stage=stage,
        prim_path="/World/SmokeCube",
        position=(0.35, -0.35, 0.08),
        scale=(0.16, 0.16, 0.16),
        color=(0.1, 0.35, 0.9),
    )
    _add_colored_sphere(
        stage=stage,
        prim_path="/World/TargetTomato",
        position=(0.55, 0.0, 0.55),
        radius=0.08,
        color=(0.9, 0.1, 0.05),
    )

    camera = UsdGeom.Camera.Define(stage, plan.camera_prim_path)
    camera.AddTranslateOp().Set(Gf.Vec3d(*plan.camera_position_m))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(*plan.camera_rotation_deg))
    camera.GetFocalLengthAttr().Set(24.0)
    print("Isaac viewport smoke scene is ready.")
    print("Required prims:")
    for prim_path in plan.required_prim_paths:
        print(f"  - {prim_path}: {'ok' if stage.GetPrimAtPath(prim_path).IsValid() else 'missing'}")
    return SmokeSceneBindings(
        camera_prim_path=plan.camera_prim_path,
        proxy_robot=proxy_robot,
    )


def _ensure_world_xform(stage: object) -> None:
    from pxr import UsdGeom

    UsdGeom.Xform.Define(stage, "/World")


def _add_ground_plane(stage: object) -> None:
    from pxr import Gf, UsdGeom

    ground = UsdGeom.Cube.Define(stage, "/World/GroundPlane")
    ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.025))
    ground.AddScaleOp().Set(Gf.Vec3f(2.0, 2.0, 0.025))
    ground.CreateDisplayColorAttr([(0.55, 0.6, 0.55)])


def _add_colored_cube(
    *,
    stage: object,
    prim_path: str,
    position: tuple[float, float, float],
    scale: tuple[float, float, float],
    color: tuple[float, float, float],
) -> None:
    from pxr import Gf, UsdGeom

    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    cube.AddScaleOp().Set(Gf.Vec3f(*scale))
    cube.CreateDisplayColorAttr([color])


def _add_colored_sphere(
    *,
    stage: object,
    prim_path: str,
    position: tuple[float, float, float],
    radius: float,
    color: tuple[float, float, float],
) -> None:
    from pxr import Gf, UsdGeom

    sphere = UsdGeom.Sphere.Define(stage, prim_path)
    sphere.AddTranslateOp().Set(Gf.Vec3d(*position))
    sphere.GetRadiusAttr().Set(radius)
    sphere.CreateDisplayColorAttr([color])


def _add_lighting(stage: object) -> None:
    from pxr import Gf, UsdLux

    light = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    light.CreateIntensityAttr(500.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 35.0))


def _add_franka_reference(stage: object, asset_root: Path | None) -> None:
    from pxr import UsdGeom

    if asset_root is None:
        return

    franka_prim = UsdGeom.Xform.Define(stage, "/World/FrankaPanda").GetPrim()
    franka_prim.GetReferences().AddReference(str(asset_root))
    print(f"Added Franka reference: {asset_root}")


def _find_franka_asset_root() -> Path | None:
    candidates = [
        ISAAC_SIM_ROOT / "assets" / "Isaac" / "Robots" / "Franka" / "franka.usd",
        ISAAC_SIM_ROOT / "extscache" / "isaacsim.asset.browser-0.1.7+lx64" / "data" / "Isaac" / "Robots" / "Franka" / "franka.usd",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def _add_robot_proxy(stage: object) -> ProxyRobotBindings:
    from pxr import Gf, UsdGeom

    base = UsdGeom.Cube.Define(stage, "/World/RobotProxy/Base")
    base.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.25, 0.10))
    base.AddScaleOp().Set(Gf.Vec3f(0.12, 0.12, 0.10))
    base.CreateDisplayColorAttr([(0.18, 0.26, 0.38)])

    shoulder = UsdGeom.Cylinder.Define(stage, "/World/RobotProxy/Shoulder")
    shoulder.CreateAxisAttr("Z")
    shoulder.GetHeightAttr().Set(0.38)
    shoulder.GetRadiusAttr().Set(0.03)
    shoulder.AddTranslateOp().Set(Gf.Vec3d(0.12, -0.18, 0.34))
    shoulder_rotate = shoulder.AddRotateXYZOp()
    shoulder_rotate.Set(Gf.Vec3f(0.0, 35.0, -30.0))
    shoulder.CreateDisplayColorAttr([(0.12, 0.37, 0.82)])

    forearm = UsdGeom.Cylinder.Define(stage, "/World/RobotProxy/Forearm")
    forearm.CreateAxisAttr("Z")
    forearm.GetHeightAttr().Set(0.32)
    forearm.GetRadiusAttr().Set(0.025)
    forearm.AddTranslateOp().Set(Gf.Vec3d(0.32, -0.08, 0.50))
    forearm_rotate = forearm.AddRotateXYZOp()
    forearm_rotate.Set(Gf.Vec3f(10.0, 55.0, -25.0))
    forearm.CreateDisplayColorAttr([(0.09, 0.31, 0.73)])

    wrist = UsdGeom.Sphere.Define(stage, "/World/RobotProxy/Wrist")
    wrist.GetRadiusAttr().Set(0.035)
    wrist_translate = wrist.AddTranslateOp()
    wrist_translate.Set(Gf.Vec3d(0.47, -0.02, 0.58))
    wrist.CreateDisplayColorAttr([(0.92, 0.92, 0.95)])

    left_finger = UsdGeom.Cube.Define(stage, "/World/RobotProxy/LeftFinger")
    left_finger_translate = left_finger.AddTranslateOp()
    left_finger_translate.Set(Gf.Vec3d(0.54, -0.03, 0.56))
    left_finger.AddScaleOp().Set(Gf.Vec3f(0.012, 0.01, 0.05))
    left_finger.CreateDisplayColorAttr([(0.10, 0.10, 0.12)])

    right_finger = UsdGeom.Cube.Define(stage, "/World/RobotProxy/RightFinger")
    right_finger_translate = right_finger.AddTranslateOp()
    right_finger_translate.Set(Gf.Vec3d(0.54, 0.03, 0.56))
    right_finger.AddScaleOp().Set(Gf.Vec3f(0.012, 0.01, 0.05))
    right_finger.CreateDisplayColorAttr([(0.10, 0.10, 0.12)])
    return ProxyRobotBindings(
        shoulder_rotate=shoulder_rotate,
        forearm_rotate=forearm_rotate,
        wrist_translate=wrist_translate,
        left_finger_translate=left_finger_translate,
        right_finger_translate=right_finger_translate,
    )


def _try_set_active_camera(camera_prim_path: str) -> None:
    import time as _time

    try:
        import omni.kit.app
        import omni.kit.viewport.utility

        app = omni.kit.app.get_app()
        for _ in range(60):
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport is not None:
                viewport.camera_path = camera_prim_path
                print(f"Active viewport camera set to {camera_prim_path}.")
                return
            app.update()
            _time.sleep(0.01)
    except Exception:
        pass
    print("Viewport camera override is unavailable; continuing with the default viewport camera.")


def _update_proxy_robot(bindings: ProxyRobotBindings, *, elapsed_seconds: float) -> None:
    from pxr import Gf

    pose = compute_proxy_robot_pose(elapsed_seconds)
    bindings.shoulder_rotate.Set(Gf.Vec3f(*pose.shoulder_rotation_deg))
    bindings.forearm_rotate.Set(Gf.Vec3f(*pose.forearm_rotation_deg))
    bindings.wrist_translate.Set(Gf.Vec3d(*pose.wrist_position))
    bindings.left_finger_translate.Set(Gf.Vec3d(*pose.left_finger_position))
    bindings.right_finger_translate.Set(Gf.Vec3d(*pose.right_finger_position))


def _wait_for_first_frame(max_frames: int) -> None:
    import carb.eventdispatcher
    import omni.kit.app
    import omni.usd

    app = omni.kit.app.get_app()
    callback_called = False

    def _on_event(event: object) -> None:
        nonlocal callback_called
        callback_called = True

    dispatcher = carb.eventdispatcher.get_eventdispatcher()
    usd_context = omni.usd.get_context()
    observer = dispatcher.observe_event(
        event_name=usd_context.stage_rendering_event_name(omni.usd.StageRenderingEventType.NEW_FRAME, True),
        on_event=_on_event,
        observer_name="tomato_harvest_viewport_smoke.first_frame",
        order=0,
    )
    try:
        for _ in range(max_frames):
            app.update()
            if callback_called:
                print("First viewport frame rendered.")
                return
            time.sleep(0.01)
        print("Viewport frame callback did not arrive within the warm-up budget.")
    finally:
        observer = None


def _pump_updates(update_fn: object, *, frame_count: int) -> None:
    for _ in range(frame_count):
        update_fn()


if __name__ == "__main__":
    raise SystemExit(main())
