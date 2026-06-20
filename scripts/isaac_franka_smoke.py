#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import random
import sys
import tempfile
import time
import traceback
from pathlib import Path
from typing import Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
ISAAC_SIM_ROOT = Path(os.environ.get("ISAAC_SIM_ROOT", "/isaac-sim"))
ISAAC_SIM_EXPERIENCE = ISAAC_SIM_ROOT / "apps" / "isaacsim.exp.base.python.kit"

if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from tomato_harvest_poc.franka_smoke import (
    build_camera_look_at_rows,
    build_default_light_specs,
    build_franka_smoke_plan,
    compute_centering_camera_position,
    compute_camera_center_error,
    compute_tomato_centering_score,
    interpolate_dof_positions,
    select_motion_step,
)

_FRANKA_ARM_JOINT_BOUNDS_RAD = (
    (-2.8, 2.8),
    (-1.7, 1.7),
    (-2.8, 2.8),
    (-3.0, -0.1),
    (-2.8, 2.8),
    (0.0, 3.6),
    (-2.8, 2.8),
)

def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Spawn a Franka Panda in Isaac Sim and move its joints.")
    parser.add_argument("--headless", action="store_true", help="Run without a native Isaac Sim window.")
    parser.add_argument("--test", action="store_true", help="Exit after one motion cycle for automated checks.")
    parser.add_argument(
        "--frames-per-step",
        type=int,
        default=0,
        help="Override the default number of simulation frames used for each motion step.",
    )
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=0.0,
        help="Stop after the given wall-clock timeout. 0 means no timeout.",
    )
    parser.add_argument(
        "--use-hand-camera",
        dest="use_hand_camera",
        action="store_true",
        help="Switch the active viewport to the hand-mounted camera instead of the debug overview camera.",
    )
    parser.add_argument(
        "--use-eye-to-hand-camera",
        dest="use_hand_camera",
        action="store_true",
        help="Deprecated alias for --use-hand-camera.",
    )
    parser.add_argument(
        "--center-tomato",
        action="store_true",
        help="Move the Franka hand until the tomato is centered in the hand camera view.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.headless and not os.environ.get("DISPLAY"):
        _emit("DISPLAY is not set. Run inside the X11-enabled debug container or pass --headless.", error=True)
        return 2

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": args.headless,
            "renderer": "MinimalRendering" if args.headless else "RaytracedLighting",
            "anti_aliasing": 0 if args.headless else 3,
            "sync_loads": False,
            "fast_shutdown": True,
            "create_new_stage": False,
            "disable_viewport_updates": args.headless,
            "extra_args": [
                "--/app/hangDetector/timeout=300",
                "--/persistent/renderer/startupMessageDisplayed=true",
            ],
        },
        experience=str(ISAAC_SIM_EXPERIENCE),
    )
    try:
        return _run(simulation_app, args)
    finally:
        simulation_app.close()


def _run(simulation_app: object, args: argparse.Namespace) -> int:
    import isaacsim.core.experimental.utils.app as app_utils
    from isaacsim.core.experimental.prims import Articulation

    plan = build_franka_smoke_plan()
    frames_per_step = args.frames_per_step or plan.frames_per_step

    try:
        _emit("Launching Franka smoke test...")
        _emit(f"  headless={args.headless}")
        _emit(f"  test={args.test}")
        _emit(f"  frames_per_step={frames_per_step}")
        _emit(f"  experience={ISAAC_SIM_EXPERIENCE}")
        _emit(f"  use_hand_camera={args.use_hand_camera}")
        _emit(f"  center_tomato={args.center_tomato}")

        stage = _open_local_franka_stage()
        _emit("Opened imported Franka stage.")
        robot_prim_path = _resolve_robot_prim_path(stage, plan)
        _emit(f"Resolved robot prim path: {robot_prim_path}")
        hand_prim_path = _resolve_hand_prim_path(stage)
        _emit(f"Resolved hand prim path: {hand_prim_path}")
        articulation = Articulation(robot_prim_path)
        _emit("Created Articulation wrapper.")
        articulation.set_default_state(dof_positions=list(plan.motion_steps[0].dof_positions))
        _emit("Applied default DOF state.")

        hand_camera_prim_path = _resolve_hand_camera_prim_path(hand_prim_path, plan)
        _add_scene_markers(stage, plan, hand_camera_prim_path)
        _emit("Added scene markers and cameras.")
        simulation_app.update()
        simulation_app.update()
        _emit("Completed initial app updates.")

        defer_hand_camera_switch = not args.headless and args.center_tomato and args.use_hand_camera
        if not args.headless:
            camera_path = plan.debug_camera_prim_path if defer_hand_camera_switch else (
                hand_camera_prim_path if args.use_hand_camera else plan.debug_camera_prim_path
            )
            _try_set_active_camera(camera_path)

        _print_scene_summary(stage, plan, robot_prim_path, hand_camera_prim_path)

        timeout_deadline = time.monotonic() + args.timeout_seconds if args.timeout_seconds > 0 else None

        if args.center_tomato:
            app_utils.play()
            _settle_simulation(simulation_app, steps=20)
            _emit("Simulation is playing for tomato centering.")
            _center_tomato_in_hand_camera(
                simulation_app=simulation_app,
                stage=stage,
                articulation=articulation,
                plan=plan,
                robot_prim_path=robot_prim_path,
                hand_prim_path=hand_prim_path,
                hand_camera_prim_path=hand_camera_prim_path,
                activate_hand_camera_before_animation=defer_hand_camera_switch,
            )
            if args.test:
                _emit(_format_joint_positions(articulation, prefix="Final"))
                return 0
            _emit("Tomato centered. Holding the current pose.")
            while simulation_app.is_running():
                simulation_app.update()
                if timeout_deadline is not None and time.monotonic() >= timeout_deadline:
                    _emit("Timeout reached.")
                    break
            _emit(_format_joint_positions(articulation, prefix="Final"))
            return 0

        app_utils.play()
        simulation_app.update()
        _emit("Simulation is playing.")

        motion_cycle_frames = frames_per_step * len(plan.motion_steps)
        current_step_index = -1
        frame = 0
        while simulation_app.is_running():
            step_index = select_motion_step(
                frame=frame,
                frames_per_step=frames_per_step,
                motion_step_count=len(plan.motion_steps),
            )
            next_step_index = (step_index + 1) % len(plan.motion_steps)
            phase = (frame % frames_per_step) / float(frames_per_step)
            positions = interpolate_dof_positions(
                plan.motion_steps[step_index].dof_positions,
                plan.motion_steps[next_step_index].dof_positions,
                progress=phase,
            )
            if step_index != current_step_index:
                current_step_index = step_index
                current_step = plan.motion_steps[step_index]
                _emit(f"Motion step {step_index + 1}/{len(plan.motion_steps)}: {current_step.label}")
            articulation.set_dof_positions(list(positions))

            simulation_app.update()
            frame += 1

            if frame % frames_per_step == 0:
                _emit(_format_joint_positions(articulation))

            if args.test and frame >= motion_cycle_frames:
                _emit("Completed one motion cycle in test mode.")
                break
            if timeout_deadline is not None and time.monotonic() >= timeout_deadline:
                _emit("Timeout reached.")
                break

        _emit(_format_joint_positions(articulation, prefix="Final"))
        return 0
    except Exception:
        _emit("Franka smoke failed with an exception:", error=True)
        _emit(traceback.format_exc(), error=True)
        return 1


def _open_local_franka_stage() -> object:
    import omni.kit.app
    import omni.usd
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    plan = build_franka_smoke_plan()
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    extension_manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)

    urdf_path = ISAAC_SIM_ROOT / plan.franka_urdf_relative_path.lstrip("/")
    output_dir = Path(tempfile.mkdtemp(prefix="franka_smoke_usd_"))

    importer = URDFImporter()
    config = URDFImporterConfig()
    config.urdf_path = str(urdf_path)
    config.usd_path = str(output_dir)
    config.merge_mesh = False
    config.collision_from_visuals = False
    config.debug_mode = False
    importer.config = config

    _emit(f"Importing local Franka URDF: {urdf_path}")
    output_path = importer.import_urdf()
    _emit(f"Imported Franka USD: {output_path}")

    omni.usd.get_context().open_stage(output_path)
    stage = omni.usd.get_context().get_stage()
    _emit("Opened imported USD stage.")
    robot_prim = stage.GetPrimAtPath("/panda_arm_hand")
    if robot_prim.IsValid():
        robot_prim.GetVariantSet("Physics").SetVariantSelection("physx")
        _emit("Selected physx physics variant for /panda_arm_hand.")
    return stage


def _resolve_robot_prim_path(stage: object, plan: object) -> str:
    preferred_paths = (plan.robot_prim_path, "/panda_arm_hand")
    for prim_path in preferred_paths:
        if stage.GetPrimAtPath(prim_path).IsValid():
            return prim_path

    root_prims = [prim.GetPath().pathString for prim in stage.GetPseudoRoot().GetChildren()]
    for prim_path in root_prims:
        if "panda" in prim_path.lower():
            return prim_path
    raise RuntimeError(f"Could not resolve Franka robot prim path. Root prims: {root_prims}")


def _resolve_hand_prim_path(stage: object) -> str:
    preferred_paths: list[str] = []
    fallback_paths: list[str] = []
    for prim in stage.Traverse():
        prim_path = prim.GetPath().pathString
        if prim_path.endswith("/panda_hand"):
            if "/Geometry/" in prim_path:
                preferred_paths.append(prim_path)
            else:
                fallback_paths.append(prim_path)
    if preferred_paths:
        return preferred_paths[0]
    if fallback_paths:
        return fallback_paths[0]
    raise RuntimeError("Could not resolve panda_hand prim path in imported Franka stage.")


def _resolve_hand_camera_prim_path(hand_prim_path: str, plan: object) -> str:
    return f"{hand_prim_path}/{plan.hand_camera_prim_name}"


def _add_scene_markers(stage: object, plan: object, hand_camera_prim_path: str) -> None:
    from pxr import Gf, UsdGeom, UsdLux

    UsdGeom.Xform.Define(stage, "/World")

    for light_spec in build_default_light_specs():
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

    branch = UsdGeom.Cube.Define(stage, "/World/TomatoBranch")
    branch.AddTranslateOp().Set(Gf.Vec3d(0.62, 0.0, 0.72))
    branch.AddScaleOp().Set(Gf.Vec3f(0.28, 0.03, 0.03))
    branch.CreateDisplayColorAttr([(0.34, 0.52, 0.22)])

    tomato = UsdGeom.Sphere.Define(stage, plan.target_tomato_prim_path)
    tomato.AddTranslateOp().Set(Gf.Vec3d(0.64, 0.0, 0.55))
    tomato.GetRadiusAttr().Set(plan.tomato_radius_m)
    tomato.CreateDisplayColorAttr([(0.88, 0.16, 0.12)])

    highlight = UsdGeom.Sphere.Define(stage, "/World/TargetTomatoHighlight")
    highlight.AddTranslateOp().Set(Gf.Vec3d(0.64, 0.0, 0.55))
    highlight.AddScaleOp().Set(Gf.Vec3f(1.3, 1.3, 1.3))
    highlight.GetRadiusAttr().Set(plan.tomato_highlight_radius_m)
    highlight.CreateDisplayColorAttr([(0.98, 0.82, 0.12)])

    debug_camera = UsdGeom.Camera.Define(stage, plan.debug_camera_prim_path)
    debug_camera.AddTranslateOp().Set(Gf.Vec3d(*plan.debug_camera_position_m))
    debug_camera.AddRotateXYZOp().Set(Gf.Vec3f(*plan.debug_camera_rotation_deg))
    debug_camera.GetFocalLengthAttr().Set(24.0)

    hand_camera = UsdGeom.Camera.Define(stage, hand_camera_prim_path)
    hand_camera.AddTranslateOp().Set(Gf.Vec3d(*plan.hand_camera_local_offset_m))
    hand_camera.AddRotateXYZOp().Set(Gf.Vec3f(*plan.hand_camera_local_rotation_deg))
    hand_camera.GetFocalLengthAttr().Set(18.0)
    hand_camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))


def _try_set_active_camera(camera_prim_path: str) -> None:
    try:
        import omni.kit.app
        import omni.kit.viewport.utility
    except Exception:
        _emit("Viewport camera utility is unavailable; continuing with the current camera.")
        return

    app = omni.kit.app.get_app()
    for _ in range(120):
        viewport = omni.kit.viewport.utility.get_active_viewport()
        if viewport is not None:
            viewport.camera_path = camera_prim_path
            _emit(f"Active viewport camera set to {camera_prim_path}.")
            return
        app.update()
        time.sleep(0.01)
    _emit("Active viewport camera could not be switched; continuing with the current camera.")


def _print_scene_summary(
    stage: object,
    plan: object,
    robot_prim_path: str,
    hand_camera_prim_path: str,
) -> None:
    required_prims = (
        robot_prim_path,
        "/World",
        plan.target_tomato_prim_path,
        plan.debug_camera_prim_path,
        hand_camera_prim_path,
        "/World/KeyLight",
    )
    _emit("Franka smoke scene is ready.")
    for prim_path in required_prims:
        status = "ok" if stage.GetPrimAtPath(prim_path).IsValid() else "missing"
        _emit(f"  - {prim_path}: {status}")


def _format_joint_positions(articulation: object, prefix: str = "Measured") -> str:
    positions = articulation.get_dof_positions()
    if hasattr(positions, "numpy"):
        positions = positions.numpy()
    if hasattr(positions, "tolist"):
        positions = positions.tolist()
    if positions and isinstance(positions[0], (list, tuple)):
        positions = positions[0]
    rounded = ", ".join(f"{float(value):.3f}" for value in positions)
    return f"{prefix} joint positions: [{rounded}]"


def _center_tomato_in_hand_camera(
    *,
    simulation_app: object,
    stage: object,
    articulation: object,
    plan: object,
    robot_prim_path: str,
    hand_prim_path: str,
    hand_camera_prim_path: str,
    activate_hand_camera_before_animation: bool,
) -> None:
    current_positions = np.array(plan.motion_steps[1].dof_positions, dtype=float)
    _set_articulation_positions(articulation, current_positions)
    _settle_simulation(simulation_app, steps=12)

    _emit("Starting tomato centering in the hand camera view.")
    _emit_tomato_coordinates(stage, plan, hand_camera_prim_path, prefix="Initial")

    target_positions = _search_centering_joint_positions(
        simulation_app=simulation_app,
        stage=stage,
        plan=plan,
        articulation=articulation,
        hand_camera_prim_path=hand_camera_prim_path,
    )
    _emit(f"Centering target joint positions: {_format_joint_positions_from_values(target_positions)}")
    if activate_hand_camera_before_animation:
        _try_set_active_camera(hand_camera_prim_path)
    _animate_articulation_to_joint_positions(
        simulation_app=simulation_app,
        articulation=articulation,
        start_positions=current_positions,
        target_positions=target_positions,
        frames=plan.centering_interpolation_frames,
    )

    final_camera_point, _ = _read_tomato_positions(
        stage=stage,
        plan=plan,
        hand_camera_prim_path=hand_camera_prim_path,
    )
    _emit_tomato_coordinates(stage, plan, hand_camera_prim_path, prefix="Final")
    _emit(f"Final centering error: {compute_camera_center_error(final_camera_point):.6f} m")


def _search_centering_joint_positions(
    *,
    simulation_app: object,
    stage: object,
    plan: object,
    articulation: object,
    hand_camera_prim_path: str,
) -> np.ndarray:
    current_positions = _get_articulation_positions(articulation)
    candidate_positions = [current_positions.copy()]
    candidate_positions.extend(np.array(step.dof_positions, dtype=float) for step in plan.motion_steps)

    rng = random.Random(7)
    open_finger = float(plan.motion_steps[1].dof_positions[7])
    for _ in range(24):
        sample = []
        for low, high in _FRANKA_ARM_JOINT_BOUNDS_RAD:
            sample.append(rng.uniform(low, high))
        sample.extend([open_finger, open_finger])
        candidate_positions.append(np.array(sample, dtype=float))

    best_positions = current_positions.copy()
    best_camera_point = None
    best_label = "pre_grasp_open"
    best_score = float("inf")
    for candidate_index, candidate in enumerate(candidate_positions):
        _set_articulation_positions(articulation, candidate)
        _settle_simulation(simulation_app, steps=3)
        trial_camera_point, _ = _read_tomato_positions(
            stage=stage,
            plan=plan,
            hand_camera_prim_path=hand_camera_prim_path,
        )
        trial_score = compute_tomato_centering_score(trial_camera_point)
        if best_camera_point is None or trial_score < best_score:
            best_positions = candidate.copy()
            best_camera_point = trial_camera_point
            best_score = trial_score
            best_label = f"candidate_{candidate_index}"

    _set_articulation_positions(articulation, best_positions)
    _settle_simulation(simulation_app, steps=6)
    _emit(
        "Best hidden seed pose: "
        f"label={best_label} "
        f"camera_xyz={_format_xyz(best_camera_point)} "
        f"score={best_score:.6f}"
    )

    current_positions = best_positions.copy()
    current_camera_point, _ = _read_tomato_positions(
        stage=stage,
        plan=plan,
        hand_camera_prim_path=hand_camera_prim_path,
    )
    best_camera_point = current_camera_point
    best_score = compute_tomato_centering_score(best_camera_point)
    step_rad = 0.08
    joint_indices = range(7)

    for iteration in range(18):
        current_camera_point, current_world_point = _read_tomato_positions(
            stage=stage,
            plan=plan,
            hand_camera_prim_path=hand_camera_prim_path,
        )
        current_error = compute_camera_center_error(current_camera_point)
        _emit(
            "Centering search "
            f"{iteration + 1}/18: "
            f"camera_xyz={_format_xyz(current_camera_point)} "
            f"world_xyz={_format_xyz(current_world_point)} "
            f"xy_error={current_error:.6f}"
        )
        if current_error <= plan.centering_position_tolerance_m:
            break

        candidate_positions = None
        candidate_score = compute_tomato_centering_score(current_camera_point)
        for joint_index in joint_indices:
            for direction in (-1.0, 1.0):
                trial_positions = current_positions.copy()
                trial_positions[joint_index] += direction * step_rad
                low, high = _FRANKA_ARM_JOINT_BOUNDS_RAD[joint_index]
                trial_positions[joint_index] = min(max(trial_positions[joint_index], low), high)
                _set_articulation_positions(articulation, trial_positions)
                _settle_simulation(simulation_app, steps=4)
                trial_camera_point, _ = _read_tomato_positions(
                    stage=stage,
                    plan=plan,
                    hand_camera_prim_path=hand_camera_prim_path,
                )
                trial_score = compute_tomato_centering_score(trial_camera_point)
                if trial_score < candidate_score:
                    candidate_score = trial_score
                    candidate_positions = trial_positions.copy()

        if candidate_positions is None:
            step_rad *= 0.5
            _set_articulation_positions(articulation, current_positions)
            _settle_simulation(simulation_app, steps=3)
            _emit(f"No improving hidden search step found. Reducing joint step to {step_rad:.6f} rad.")
            if step_rad < 0.005:
                break
            continue

        current_positions = candidate_positions
        _set_articulation_positions(articulation, current_positions)
        _settle_simulation(simulation_app, steps=6)
        current_camera_point, _ = _read_tomato_positions(
            stage=stage,
            plan=plan,
            hand_camera_prim_path=hand_camera_prim_path,
        )
        current_score = compute_tomato_centering_score(current_camera_point)
        if current_score < best_score:
            best_score = current_score
            best_positions = current_positions.copy()

    _set_articulation_positions(articulation, plan.motion_steps[1].dof_positions)
    _settle_simulation(simulation_app, steps=3)
    return best_positions


def _build_centering_hand_target_pose_candidates(
    *,
    stage: object,
    plan: object,
    hand_prim_path: str,
    hand_camera_prim_path: str,
) -> list[tuple[str, np.ndarray, np.ndarray, np.ndarray]]:
    from isaacsim.core.utils.numpy.rotations import rot_matrices_to_quats
    from pxr import Gf, UsdGeom

    hand_camera_prim = stage.GetPrimAtPath(hand_camera_prim_path)
    tomato_camera_point, tomato_world_point = _read_tomato_positions(
        stage=stage,
        plan=plan,
        hand_camera_prim_path=hand_camera_prim_path,
    )
    current_camera_world_matrix = UsdGeom.Xformable(hand_camera_prim).ComputeLocalToWorldTransform(0.0)
    camera_local_matrix = UsdGeom.Xformable(hand_camera_prim).GetLocalTransformation()

    current_camera_position = current_camera_world_matrix.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
    desired_camera_position = compute_centering_camera_position(
        (current_camera_position[0], current_camera_position[1], current_camera_position[2]),
        tomato_world_point,
        preferred_depth_m=plan.centering_preferred_depth_m,
    )
    desired_camera_world = _build_matrix4d_from_rows(
        build_camera_look_at_rows(desired_camera_position, tomato_world_point)
    )

    _emit(
        "Initial camera alignment: "
        f"camera_xyz={_format_xyz(tomato_camera_point)} "
        f"world_xyz={_format_xyz(tomato_world_point)}"
    )
    candidate_world_matrices = (
        ("inverse_local_left", camera_local_matrix.GetInverse() * desired_camera_world),
        ("inverse_local_right", desired_camera_world * camera_local_matrix.GetInverse()),
    )
    candidates: list[tuple[str, np.ndarray, np.ndarray, np.ndarray]] = []
    for label, desired_hand_world in candidate_world_matrices:
        desired_hand_position = desired_hand_world.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
        desired_hand_rotation = np.array(
            [
                [desired_hand_world[row_index][column_index] for column_index in range(3)]
                for row_index in range(3)
            ],
            dtype=float,
        )
        desired_hand_orientation = rot_matrices_to_quats(desired_hand_rotation[np.newaxis, :, :])[0]
        transposed_orientation = rot_matrices_to_quats(desired_hand_rotation.T[np.newaxis, :, :])[0]
        target_position = np.array([desired_hand_position[0], desired_hand_position[1], desired_hand_position[2]], dtype=float)
        target_camera_position = np.array(desired_camera_position, dtype=float)
        candidates.append((f"{label}_rot", target_position, np.array(desired_hand_orientation, dtype=float), target_camera_position))
        candidates.append((f"{label}_rot_t", target_position, np.array(transposed_orientation, dtype=float), target_camera_position))
    return candidates


def _build_matrix4d_from_rows(rows: tuple[tuple[float, float, float, float], ...]) -> object:
    from pxr import Gf

    matrix = Gf.Matrix4d(1.0)
    for row_index, row_values in enumerate(rows):
        matrix.SetRow(row_index, Gf.Vec4d(*row_values))
    return matrix


def _merge_action_joint_positions(articulation: object, action: object) -> np.ndarray:
    current_positions = _get_articulation_positions(articulation)
    target_positions = current_positions.copy()
    action_positions = np.array(action.joint_positions, dtype=float)
    if action.joint_indices is None:
        target_positions[: len(action_positions)] = action_positions
        return target_positions

    action_indices = np.array(action.joint_indices, dtype=int)
    target_positions[action_indices] = action_positions
    return target_positions


def _animate_articulation_to_joint_positions(
    *,
    simulation_app: object,
    articulation: object,
    start_positions: np.ndarray,
    target_positions: np.ndarray,
    frames: int,
) -> None:
    for frame_index in range(max(frames, 1)):
        progress = (frame_index + 1) / float(max(frames, 1))
        positions = interpolate_dof_positions(
            tuple(float(value) for value in start_positions),
            tuple(float(value) for value in target_positions),
            progress=progress,
        )
        _set_articulation_positions(articulation, positions)
        simulation_app.update()


def _set_articulation_positions(articulation: object, positions: Sequence[float]) -> None:
    if hasattr(articulation, "set_dof_positions"):
        articulation.set_dof_positions(list(positions))
        return
    if hasattr(articulation, "set_joint_positions"):
        articulation.set_joint_positions(np.array(positions, dtype=float))
        return
    raise TypeError(f"Unsupported articulation wrapper: {type(articulation)!r}")


def _get_articulation_positions(articulation: object) -> np.ndarray:
    if hasattr(articulation, "get_joint_positions"):
        return np.array(articulation.get_joint_positions(), dtype=float)
    if hasattr(articulation, "get_dof_positions"):
        positions = articulation.get_dof_positions()
        if hasattr(positions, "numpy"):
            positions = positions.numpy()
        if hasattr(positions, "tolist"):
            positions = positions.tolist()
        if positions and isinstance(positions[0], (list, tuple)):
            positions = positions[0]
        return np.array(positions, dtype=float)
    raise TypeError(f"Unsupported articulation wrapper: {type(articulation)!r}")


def _enable_extension(extension_name: str) -> None:
    import omni.kit.app

    extension_manager = omni.kit.app.get_app().get_extension_manager()
    extension_manager.set_extension_enabled_immediate(extension_name, True)


def _read_tomato_positions(
    *,
    stage: object,
    plan: object,
    hand_camera_prim_path: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    from pxr import Gf, UsdGeom

    tomato_prim = stage.GetPrimAtPath(plan.target_tomato_prim_path)
    hand_camera_prim = stage.GetPrimAtPath(hand_camera_prim_path)

    tomato_world_matrix = UsdGeom.Xformable(tomato_prim).ComputeLocalToWorldTransform(0.0)
    hand_camera_world_matrix = UsdGeom.Xformable(hand_camera_prim).ComputeLocalToWorldTransform(0.0)

    tomato_world = tomato_world_matrix.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
    camera_to_world_inverse = hand_camera_world_matrix.GetInverse()
    tomato_camera = camera_to_world_inverse.Transform(tomato_world)

    return (
        (tomato_camera[0], tomato_camera[1], tomato_camera[2]),
        (tomato_world[0], tomato_world[1], tomato_world[2]),
    )


def _emit_tomato_coordinates(
    stage: object,
    plan: object,
    hand_camera_prim_path: str,
    *,
    prefix: str,
) -> None:
    tomato_camera_point, tomato_world_point = _read_tomato_positions(
        stage=stage,
        plan=plan,
        hand_camera_prim_path=hand_camera_prim_path,
    )
    _emit(f"{prefix} tomato world xyz: {_format_xyz(tomato_world_point)}")
    _emit(f"{prefix} tomato camera xyz: {_format_xyz(tomato_camera_point)}")


def _settle_simulation(simulation_app: object, *, steps: int) -> None:
    for _ in range(steps):
        simulation_app.update()


def _format_xyz(position_xyz: tuple[float, float, float]) -> str:
    return f"({position_xyz[0]:.4f}, {position_xyz[1]:.4f}, {position_xyz[2]:.4f})"


def _format_joint_positions_from_values(positions: Sequence[float]) -> str:
    rounded = ", ".join(f"{float(value):.3f}" for value in positions)
    return f"[{rounded}]"


def _emit(message: str, *, error: bool = False) -> None:
    stream = sys.stderr if error else sys.stdout
    print(message, file=stream, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
