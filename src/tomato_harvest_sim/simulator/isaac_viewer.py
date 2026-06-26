from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from tomato_harvest_sim.app.application import create_tomato_harvest_application
from tomato_harvest_sim.simulator.control_panel import ControlPanelController, IsaacControlPanelWindow
from tomato_harvest_sim.simulator.franka_motion import IsaacFrankaMotionExecutor
from tomato_harvest_sim.simulator.physics_harvest import IsaacPhysicsHarvestBridge, PhysicsHarvestScenePaths
from tomato_harvest_sim.simulator.scene_plan import ReviewScenePlan, build_review_scene_plan
from tomato_harvest_sim.simulator.scene_runtime_view import (
    SceneRuntimeDisplay,
    build_scene_runtime_display,
    sync_scene_runtime_display,
)

ISAAC_SIM_ROOT = Path(os.environ.get("ISAAC_SIM_ROOT", "/isaac-sim"))
ISAAC_SIM_EXPERIENCE = ISAAC_SIM_ROOT / "apps" / "isaacsim.exp.base.python.kit"
OFFICIAL_FRANKA_ASSET_RELATIVE_PATH = "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"


@dataclass(frozen=True)
class CameraPrimPaths:
    fixed_camera_prim_path: str
    hand_camera_prim_path: str


@dataclass(frozen=True)
class SceneBuildArtifacts:
    camera_paths: CameraPrimPaths
    runtime_display: SceneRuntimeDisplay
    physics_bridge: IsaacPhysicsHarvestBridge | None


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tomato harvest Isaac Sim 3DView review")
    parser.add_argument("--headless", action="store_true", help="Run without a native Isaac Sim window.")
    parser.add_argument("--auto-start", action="store_true", help="Automatically press Start after boot.")
    parser.add_argument(
        "--headless-steps",
        type=int,
        default=64,
        help="Number of runtime steps to execute in headless mode.",
    )
    parser.add_argument(
        "--camera-view",
        choices=("fixed", "hand"),
        default="fixed",
        help="Initial active camera for the 3DView.",
    )
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=0.0,
        help="Automatically close after the given seconds in GUI mode. 0 keeps the window open.",
    )
    parser.add_argument(
        "--grasp-mode",
        choices=("success", "failure"),
        default="success",
        help="Choose the stable grasp or failed grasp demo path.",
    )
    parser.add_argument(
        "--transport",
        choices=("in_memory", "ros2", "auto"),
        default=os.environ.get("TOMATO_HARVEST_VIEWER_TRANSPORT", "in_memory"),
        help="Transport used between simulator and robot runtime inside the review viewer.",
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


def build_simulation_app_config(*, headless: bool) -> dict[str, object]:
    extra_args = build_appframework_argv(headless=headless)
    extra_args.extend(
        [
            "--/app/hangDetector/timeout=300",
            "--/persistent/renderer/startupMessageDisplayed=true",
        ]
    )
    return {
        "headless": headless,
        "renderer": "MinimalRendering" if headless else "RaytracedLighting",
        "anti_aliasing": 0 if headless else 3,
        "sync_loads": False,
        "fast_shutdown": True,
        "create_new_stage": False,
        "disable_viewport_updates": headless,
        "extra_args": extra_args,
    }


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.headless and not os.environ.get("DISPLAY"):
        print(
            "DISPLAY is not set. Run inside the X11-enabled container or pass --headless.",
            file=sys.stderr,
        )
        return 2

    try:
        from isaacsim import SimulationApp
    except ImportError:
        print(
            "Isaac Sim Python modules are not available in this environment. Run inside the Isaac Sim container.",
            file=sys.stderr,
        )
        return 2

    print("Launching tomato harvest Isaac viewer...", flush=True)
    print(f"  headless={args.headless}", flush=True)
    print(f"  camera_view={args.camera_view}", flush=True)
    print(f"  grasp_mode={args.grasp_mode}", flush=True)
    print(f"  transport={args.transport}", flush=True)
    simulation_app = SimulationApp(
        build_simulation_app_config(headless=args.headless),
        experience=str(ISAAC_SIM_EXPERIENCE),
    )
    try:
        plan = build_review_scene_plan()
        print("SimulationApp initialized.", flush=True)
        use_physx_harvest = args.grasp_mode == "success"
        artifacts = _build_scene(plan, use_physx_harvest=use_physx_harvest)
        _start_timeline_playback()
        _pump_updates(simulation_app.update, frame_count=4)
        _wait_for_first_frame(simulation_app=simulation_app, max_frames=240)
        franka_executor = IsaacFrankaMotionExecutor(robot_prim_path=plan.robot_prim_path)
        control_controller = _build_control_panel_controller(
            artifacts.camera_paths,
            initial_camera_view=args.camera_view,
            grasp_mode=args.grasp_mode,
            physics_grasp_enabled=use_physx_harvest,
            transport=args.transport,
        )
        if args.auto_start:
            control_controller.start()
        control_window = None
        if not args.headless:
            control_window = IsaacControlPanelWindow(control_controller)
        _sync_runtime_visuals(artifacts.runtime_display, control_controller)
        _print_review_summary(plan, camera_paths=artifacts.camera_paths, camera_view=args.camera_view)

        if args.headless:
            for _ in range(args.headless_steps):
                _sync_executor_joint_state_to_runtime(franka_executor, control_controller)
                control_controller.step_runtime()
                _sync_runtime_visuals(artifacts.runtime_display, control_controller)
                executor_log = _step_franka_executor(franka_executor, control_controller)
                if executor_log is not None:
                    print(executor_log, flush=True)
                _sync_executor_pose_to_runtime(franka_executor, control_controller)
                if artifacts.physics_bridge is not None:
                    artifacts.physics_bridge.begin_physics_step()
                simulation_app.update()
                _log_executor_post_update_debug(franka_executor)
                if artifacts.physics_bridge is not None:
                    artifacts.physics_bridge.finalize_physics_step(control_controller)
                _sync_runtime_visuals(artifacts.runtime_display, control_controller)
            _pump_updates(simulation_app.update, frame_count=30)
            print("Headless scene runtime setup completed.", flush=True)
            return 0

        deadline = time.time() + args.timeout_seconds if args.timeout_seconds > 0 else None
        while simulation_app.is_running():
            _sync_executor_joint_state_to_runtime(franka_executor, control_controller)
            status = control_controller.step_runtime()
            _sync_runtime_visuals(artifacts.runtime_display, control_controller)
            executor_log = _step_franka_executor(franka_executor, control_controller)
            if executor_log is not None:
                print(executor_log, flush=True)
            _sync_executor_pose_to_runtime(franka_executor, control_controller)
            if control_window is not None:
                control_window.refresh_status(status)
            if artifacts.physics_bridge is not None:
                artifacts.physics_bridge.begin_physics_step()
            simulation_app.update()
            _log_executor_post_update_debug(franka_executor)
            if artifacts.physics_bridge is not None:
                artifacts.physics_bridge.finalize_physics_step(control_controller)
            _sync_runtime_visuals(artifacts.runtime_display, control_controller)
            if deadline is not None and time.time() >= deadline:
                print("Timeout reached. Closing Isaac review scene.", flush=True)
                break
    except KeyboardInterrupt:
        print("Interrupted by user. Closing Isaac review scene.", flush=True)
    finally:
        try:
            if "control_controller" in locals():
                control_controller.close()
        finally:
            simulation_app.close()
    return 0


def _build_scene(plan: ReviewScenePlan, *, use_physx_harvest: bool) -> SceneBuildArtifacts:
    import omni.usd
    from pxr import Gf, UsdGeom, UsdLux

    print("Scene build step: create stage", flush=True)
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, plan.world_prim_path)
    _add_physics_scene(stage)

    print("Scene build step: ground", flush=True)
    ground = UsdGeom.Cube.Define(stage, plan.ground_prim_path)
    ground.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.01))
    ground.AddScaleOp().Set(Gf.Vec3f(*plan.ground_size_m))
    ground.CreateDisplayColorAttr([(0.32, 0.36, 0.32)])

    print("Scene build step: lighting", flush=True)
    for light_spec in plan.light_specs:
        if light_spec.kind == "distant":
            light = UsdLux.DistantLight.Define(stage, light_spec.prim_path)
            light.CreateIntensityAttr(light_spec.intensity)
            if light_spec.color_rgb is not None:
                light.CreateColorAttr(Gf.Vec3f(*light_spec.color_rgb))
            if light_spec.rotate_deg is not None:
                light.AddRotateXYZOp().Set(Gf.Vec3f(*light_spec.rotate_deg))
        elif light_spec.kind == "sphere":
            light = UsdLux.SphereLight.Define(stage, light_spec.prim_path)
            light.CreateIntensityAttr(light_spec.intensity)
            if light_spec.radius_m is not None:
                light.CreateRadiusAttr(light_spec.radius_m)
            if light_spec.color_rgb is not None:
                light.CreateColorAttr(Gf.Vec3f(*light_spec.color_rgb))
            if light_spec.translate_m is not None:
                light.AddTranslateOp().Set(Gf.Vec3d(*light_spec.translate_m))

    print("Scene build step: franka", flush=True)
    _add_franka_reference(stage, plan)
    hand_mount_prim_path = _resolve_hand_mount_prim_path(stage, hand_mount_prim_suffix=plan.hand_camera_mount_prim_suffix)
    hand_camera_prim_path = f"{hand_mount_prim_path}/{plan.hand_camera_prim_name}"
    print(f"Resolved hand mount prim: {hand_mount_prim_path}", flush=True)
    print("Scene build step: branch", flush=True)
    _add_branch(stage, plan)
    print("Scene build step: stem", flush=True)
    _add_stem(stage, plan)
    print("Scene build step: tomato", flush=True)
    _add_tomato(stage, plan)
    print("Scene build step: tray", flush=True)
    _add_tray(stage, plan)
    print("Scene build step: runtime display", flush=True)
    runtime_display = build_scene_runtime_display(stage, plan, tomato_driven_by_physics=use_physx_harvest)
    print("Scene build step: fixed camera", flush=True)
    _add_fixed_camera(stage, plan)
    print("Scene build step: hand camera", flush=True)
    _add_hand_camera(stage, plan, hand_camera_prim_path=hand_camera_prim_path)
    physics_bridge = None
    if use_physx_harvest:
        physics_bridge = IsaacPhysicsHarvestBridge(
            stage=stage,
            scene_paths=PhysicsHarvestScenePaths(
                ground_prim_path=plan.ground_prim_path,
                tray_prim_path=plan.tray_prim_path,
                tomato_prim_path=plan.tomato_prim_path,
                stem_anchor_prim_path="/World/TomatoStemAnchor",
                stem_joint_prim_path="/World/TomatoStemJoint",
                grasp_joint_prim_path="/World/TomatoGraspJoint",
                hand_mount_prim_path=hand_mount_prim_path,
            ),
            initial_tomato_pose=plan.tomato_pose,
        )
        physics_bridge.prepare_scene()

    print("Isaac review scene is ready.", flush=True)
    for prim_path in plan.required_prim_paths:
        print(f"  - {prim_path}: {'ok' if stage.GetPrimAtPath(prim_path).IsValid() else 'missing'}", flush=True)
    print(f"  - {hand_camera_prim_path}: {'ok' if stage.GetPrimAtPath(hand_camera_prim_path).IsValid() else 'missing'}", flush=True)
    return SceneBuildArtifacts(
        camera_paths=CameraPrimPaths(
            fixed_camera_prim_path=plan.fixed_camera_prim_path,
            hand_camera_prim_path=hand_camera_prim_path,
        ),
        runtime_display=runtime_display,
        physics_bridge=physics_bridge,
    )


def _add_franka_reference(stage: object, plan: ReviewScenePlan) -> None:
    from pxr import Gf, UsdGeom

    asset_path = _resolve_official_franka_asset_path()
    print(f"Resolved Franka asset: {asset_path}", flush=True)
    franka_prim = UsdGeom.Xform.Define(stage, plan.robot_prim_path).GetPrim()
    franka_prim.GetReferences().AddReference(asset_path)
    UsdGeom.XformCommonAPI(franka_prim).SetTranslate(
        Gf.Vec3d(plan.robot_base_pose.x, plan.robot_base_pose.y, plan.robot_base_pose.z)
    )
    print("Added Franka reference to stage.", flush=True)


def _add_physics_scene(stage: object) -> None:
    from pxr import Gf, PhysxSchema, UsdPhysics

    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(physics_scene.GetPrim())
    physx_scene.CreateEnableCCDAttr(True)


def build_official_franka_asset_path(assets_root: str) -> str:
    return f"{assets_root.rstrip('/')}/{OFFICIAL_FRANKA_ASSET_RELATIVE_PATH}"


def _resolve_official_franka_asset_path() -> str:
    import omni.client
    from isaacsim.storage.native import get_assets_root_path

    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError("Isaac Sim assets root is not available.")

    asset_path = build_official_franka_asset_path(assets_root)
    result, _ = omni.client.stat(asset_path)
    if result != omni.client.Result.OK:
        raise FileNotFoundError(
            "Official Franka USD was not found. "
            f"path={asset_path} result={result}"
        )
    return asset_path


def select_hand_mount_prim_path(
    prim_paths: Sequence[str],
    *,
    hand_mount_prim_suffix: str,
) -> str:
    all_prim_paths = tuple(prim_paths)
    preferred_paths: list[str] = []
    fallback_paths: list[str] = []
    for prim_path in all_prim_paths:
        if not prim_path.endswith(f"/{hand_mount_prim_suffix}"):
            continue
        if "/Geometry/" in prim_path:
            preferred_paths.append(prim_path)
        else:
            fallback_paths.append(prim_path)

    if preferred_paths:
        return preferred_paths[0]
    if fallback_paths:
        return fallback_paths[0]
    raise RuntimeError(
        f"Could not resolve {hand_mount_prim_suffix} prim path. "
        f"Available prim count: {len(all_prim_paths)}"
    )


def _resolve_hand_mount_prim_path(stage: object, *, hand_mount_prim_suffix: str) -> str:
    return select_hand_mount_prim_path(
        tuple(prim.GetPath().pathString for prim in stage.Traverse()),
        hand_mount_prim_suffix=hand_mount_prim_suffix,
    )


def _add_branch(stage: object, plan: ReviewScenePlan) -> None:
    from pxr import Gf, UsdGeom

    branch = UsdGeom.Cube.Define(stage, plan.branch_prim_path)
    branch.AddTranslateOp().Set(Gf.Vec3d(plan.branch_pose.x, plan.branch_pose.y, plan.branch_pose.z))
    branch.AddScaleOp().Set(Gf.Vec3f(*plan.branch_size_m))
    branch.CreateDisplayColorAttr([(0.45, 0.34, 0.20)])


def _add_stem(stage: object, plan: ReviewScenePlan) -> None:
    from pxr import Gf, UsdGeom

    stem = UsdGeom.Cylinder.Define(stage, plan.stem_prim_path)
    stem.AddTranslateOp().Set(Gf.Vec3d(plan.stem_pose.x, plan.stem_pose.y, plan.stem_pose.z))
    stem.GetHeightAttr().Set(plan.stem_height_m)
    stem.GetRadiusAttr().Set(plan.stem_radius_m)
    stem.CreateDisplayColorAttr([(0.33, 0.62, 0.24)])


def _add_tomato(stage: object, plan: ReviewScenePlan) -> None:
    from pxr import Gf, UsdGeom

    tomato_root = UsdGeom.Xform.Define(stage, plan.tomato_prim_path)
    tomato_root.AddTranslateOp().Set(Gf.Vec3d(plan.tomato_pose.x, plan.tomato_pose.y, plan.tomato_pose.z))

    tomato_visual = UsdGeom.Sphere.Define(stage, f"{plan.tomato_prim_path}/Geometry")
    tomato_visual.GetRadiusAttr().Set(plan.tomato_radius_m)
    tomato_visual.CreateDisplayColorAttr([(0.93, 0.77, 0.17)])


def _add_tray(stage: object, plan: ReviewScenePlan) -> None:
    from pxr import Gf, UsdGeom

    tray_color = (0.36, 0.25, 0.14)
    tray_root = UsdGeom.Xform.Define(stage, plan.tray_prim_path)
    tray_root.AddTranslateOp().Set(Gf.Vec3d(plan.tray_pose.x, plan.tray_pose.y, plan.tray_pose.z))

    inner_x, inner_y, inner_z = plan.tray_inner_size_m
    wall = plan.tray_wall_thickness_m
    outer_x = inner_x + 2.0 * wall
    outer_y = inner_y + 2.0 * wall
    base_height = wall
    wall_height = inner_z
    wall_center_z = base_height * 0.5 + wall_height * 0.5

    _define_colored_cube(
        stage,
        f"{plan.tray_prim_path}/Base",
        translate=(0.0, 0.0, 0.0),
        scale=(outer_x, outer_y, base_height),
        color_rgb=tray_color,
    )
    _define_colored_cube(
        stage,
        f"{plan.tray_prim_path}/WallFront",
        translate=(0.0, outer_y * 0.5 - wall * 0.5, wall_center_z),
        scale=(outer_x, wall, wall_height),
        color_rgb=tray_color,
    )
    _define_colored_cube(
        stage,
        f"{plan.tray_prim_path}/WallBack",
        translate=(0.0, -outer_y * 0.5 + wall * 0.5, wall_center_z),
        scale=(outer_x, wall, wall_height),
        color_rgb=tray_color,
    )
    _define_colored_cube(
        stage,
        f"{plan.tray_prim_path}/WallLeft",
        translate=(-outer_x * 0.5 + wall * 0.5, 0.0, wall_center_z),
        scale=(wall, inner_y, wall_height),
        color_rgb=tray_color,
    )
    _define_colored_cube(
        stage,
        f"{plan.tray_prim_path}/WallRight",
        translate=(outer_x * 0.5 - wall * 0.5, 0.0, wall_center_z),
        scale=(wall, inner_y, wall_height),
        color_rgb=tray_color,
    )


def _define_colored_cube(
    stage: object,
    prim_path: str,
    *,
    translate: tuple[float, float, float],
    scale: tuple[float, float, float],
    color_rgb: tuple[float, float, float],
) -> None:
    from pxr import Gf, UsdGeom

    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.AddTranslateOp().Set(Gf.Vec3d(*translate))
    cube.AddScaleOp().Set(Gf.Vec3f(*scale))
    cube.CreateDisplayColorAttr([color_rgb])


def _add_fixed_camera(stage: object, plan: ReviewScenePlan) -> None:
    from pxr import Gf, UsdGeom

    camera = UsdGeom.Camera.Define(stage, plan.fixed_camera_prim_path)
    camera.AddTranslateOp().Set(Gf.Vec3d(plan.fixed_camera_pose.x, plan.fixed_camera_pose.y, plan.fixed_camera_pose.z))
    camera.AddRotateXYZOp().Set(
        Gf.Vec3f(plan.fixed_camera_pose.roll, plan.fixed_camera_pose.pitch, plan.fixed_camera_pose.yaw)
    )
    camera.GetFocalLengthAttr().Set(plan.fixed_camera_focal_length_mm)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(*plan.fixed_camera_clipping_range_m))


def _add_hand_camera(stage: object, plan: ReviewScenePlan, *, hand_camera_prim_path: str) -> None:
    from pxr import Gf, UsdGeom

    camera = UsdGeom.Camera.Define(stage, hand_camera_prim_path)
    camera.AddTranslateOp().Set(Gf.Vec3d(plan.hand_camera_pose.x, plan.hand_camera_pose.y, plan.hand_camera_pose.z))
    camera.AddRotateXYZOp().Set(
        Gf.Vec3f(plan.hand_camera_pose.roll, plan.hand_camera_pose.pitch, plan.hand_camera_pose.yaw)
    )
    camera.GetFocalLengthAttr().Set(plan.hand_camera_focal_length_mm)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(*plan.hand_camera_clipping_range_m))


def _set_active_camera(camera_paths: CameraPrimPaths, *, camera_view: str) -> None:
    try:
        import omni.kit.viewport.utility

        viewport = omni.kit.viewport.utility.get_active_viewport()
        if viewport is None:
            print("Active viewport is unavailable; keeping the default camera.", flush=True)
            return
        viewport.camera_path = (
            camera_paths.fixed_camera_prim_path if camera_view == "fixed" else camera_paths.hand_camera_prim_path
        )
        print(f"Active viewport camera set to {viewport.camera_path}.", flush=True)
    except Exception:
        print("Viewport camera override is unavailable; continuing with the default viewport camera.", flush=True)


def _build_control_panel_controller(
    camera_paths: CameraPrimPaths,
    *,
    initial_camera_view: str,
    grasp_mode: str,
    physics_grasp_enabled: bool,
    transport: str,
) -> ControlPanelController:
    system = create_tomato_harvest_application(
        grasp_mode=grasp_mode,
        physics_grasp_enabled=physics_grasp_enabled,
        physics_soft_fallback_enabled=False,
        transport=transport,
    )
    controller = ControlPanelController(
        system=system,
        set_viewport_camera=lambda camera_name: _set_active_camera(
            camera_paths,
            camera_view="hand" if camera_name == "hand_camera" else "fixed",
        ),
        log_fn=lambda message: print(message, flush=True),
    )
    controller.boot(initial_camera_name="hand_camera" if initial_camera_view == "hand" else "fixed_camera")
    return controller


def _sync_runtime_visuals(runtime_display: SceneRuntimeDisplay, controller: ControlPanelController) -> None:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return
    sync_scene_runtime_display(stage, runtime_display, controller.current_scene_snapshot())


def _step_franka_executor(
    executor: IsaacFrankaMotionExecutor,
    controller: ControlPanelController,
) -> str | None:
    try:
        executor.sync_with_snapshot(controller.current_scene_snapshot())
        return executor.step()
    except Exception as exc:
        return f"[Simulator] Franka executor error: {exc}"


def _sync_executor_pose_to_runtime(
    executor: IsaacFrankaMotionExecutor,
    controller: ControlPanelController,
) -> None:
    pose = executor.current_end_effector_pose()
    if pose is None:
        return
    controller.sync_robot_tool_pose(pose)


def _sync_executor_joint_state_to_runtime(
    executor: IsaacFrankaMotionExecutor,
    controller: ControlPanelController,
) -> None:
    joint_state = executor.current_joint_state_snapshot()
    if joint_state is None:
        return
    controller.sync_robot_joint_state(joint_state)


def _log_executor_post_update_debug(executor: IsaacFrankaMotionExecutor) -> None:
    executor.log_post_update_debug_snapshot()


def _wait_for_first_frame(*, simulation_app: object, max_frames: int) -> None:
    try:
        import omni.kit.viewport.utility
    except Exception:
        return

    viewport = omni.kit.viewport.utility.get_active_viewport()
    if viewport is None:
        return

    for _ in range(max_frames):
        simulation_app.update()
        try:
            viewport.get_render_product_path()
            print("First viewport frame rendered.", flush=True)
            return
        except Exception:
            continue


def _print_review_summary(_: ReviewScenePlan, *, camera_paths: CameraPrimPaths, camera_view: str) -> None:
    print("Tomato harvest review scene is running.", flush=True)
    print("Expected review points:", flush=True)
    print("  - Franka Panda is visible in the 3DView.", flush=True)
    print("  - Tomato is attached below TomatoStem before Start.", flush=True)
    print("  - Branch and tray are visible.", flush=True)
    print("  - Camera view can start from fixed or hand.", flush=True)
    print(f"Fixed camera: {camera_paths.fixed_camera_prim_path}", flush=True)
    print(f"Hand camera: {camera_paths.hand_camera_prim_path}", flush=True)
    print(f"Initial camera view: {camera_view}", flush=True)


def _pump_updates(update_fn: object, *, frame_count: int) -> None:
    for _ in range(frame_count):
        update_fn()


def _start_timeline_playback() -> None:
    import omni.timeline

    omni.timeline.get_timeline_interface().play()
