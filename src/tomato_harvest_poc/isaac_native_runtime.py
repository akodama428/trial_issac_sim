from __future__ import annotations

import os
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

import numpy as np

from .native_harvest import (
    CameraViewMode,
    HarvestPhase,
    HarvestScenarioPlan,
    build_harvest_scenario_plan,
    build_target_found_messages,
    has_dual_finger_contact,
    is_object_settled,
    is_point_in_box_xy,
    is_target_visible,
)

ISAAC_SIM_ROOT = Path(os.environ.get("ISAAC_SIM_ROOT", "/isaac-sim"))
ISAAC_SIM_EXPERIENCE = ISAAC_SIM_ROOT / "apps" / "isaacsim.exp.base.python.kit"


@dataclass
class JointAnimation:
    start_positions: np.ndarray
    target_positions: np.ndarray
    frames: int
    on_complete: Callable[[], None] | None = None
    frame_index: int = 0


class IsaacNativeRuntime:
    def __init__(
        self,
        *,
        headless: bool = False,
        test_mode: bool = False,
        initial_camera_view: CameraViewMode = CameraViewMode.FIXED,
    ) -> None:
        self._headless = headless
        self._test_mode = test_mode
        self._initial_camera_view = initial_camera_view
        self._plan = build_harvest_scenario_plan()
        self._simulation_app = None
        self._app = None
        self._timeline = None
        self._stage = None
        self._articulation = None
        self._kinematics_solver = None
        self._articulation_kinematics_solver = None
        self._top_down_hand_orientation = None
        self._top_down_grasp_center_offset_world = None
        self._phase = HarvestPhase.READY
        self._status_message = "Ready"
        self._active_camera_view = initial_camera_view
        self._fixed_camera_path = "/World/Camera_Fixed"
        self._hand_camera_path = ""
        self._hand_prim_path = ""
        self._robot_prim_path = ""
        self._left_finger_prim_path = ""
        self._right_finger_prim_path = ""
        self._tomato_prim_path = "/World/TargetTomato"
        self._stem_prim_path = "/World/TomatoStem"
        self._stem_anchor_prim_path = "/World/TomatoStemAnchor"
        self._branch_prim_path = "/World/TomatoBranch"
        self._fruit_hand_joint_path = "/World/Joints/FruitHandJoint"
        self._fruit_stem_joint_path = "/World/Joints/FruitStemJoint"
        self._tray_root_path = "/World/PlaceTray"
        self._tray_base_path = f"{self._tray_root_path}/Base"
        self._tray_contact_paths: tuple[str, ...] = ()
        self._found_camera_point: tuple[float, float, float] | None = None
        self._found_world_point: tuple[float, float, float] | None = None
        self._stop_requested = False
        self._scenario_active = False
        self._animation: JointAnimation | None = None
        self._tomato_body = None
        self._tomato_contact_view = None
        self._grasp_check_frames_remaining = 0
        self._grasp_contact_stable_frames = 0
        self._settle_monitor_active = False
        self._settle_stable_frames = 0
        self._settle_elapsed_frames = 0
        self._fruit_hand_joint_active = False
        self._fruit_stem_joint_active = False
        self._status_label = None
        self._phase_label = None
        self._camera_label = None
        self._control_window = None
        self._pending_ui_action: Callable[[], None] | None = None
        self._test_completed = False
        self._test_exit_code = 0

    def run(self) -> int:
        if not self._headless and not os.environ.get("DISPLAY"):
            print("DISPLAY is not set. Launch this runtime from a GUI-capable container or pass --headless.")
            return 2

        from isaacsim import SimulationApp

        self._simulation_app = SimulationApp(
            {
                "headless": self._headless,
                "renderer": "MinimalRendering" if self._headless else "RaytracedLighting",
                "anti_aliasing": 0 if self._headless else 3,
                "sync_loads": False,
                "fast_shutdown": True,
                "create_new_stage": False,
                "disable_viewport_updates": self._headless,
                "extra_args": [
                    "--/app/hangDetector/timeout=300",
                    "--/persistent/renderer/startupMessageDisplayed=true",
                ],
            },
            experience=str(ISAAC_SIM_EXPERIENCE),
        )
        try:
            self._setup_runtime()
            if self._test_mode:
                self._start_scenario()
            while self._simulation_app.is_running():
                self._simulation_app.update()
                self._update_runtime()
                if self._test_mode and self._test_completed:
                    return self._test_exit_code
            return self._test_exit_code
        finally:
            self.close()

    def close(self) -> None:
        if self._simulation_app is not None:
            self._simulation_app.close()
            self._simulation_app = None

    def _setup_runtime(self) -> None:
        import omni.timeline
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.extensions import get_extension_path_from_name
        from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver

        self._timeline = omni.timeline.get_timeline_interface()
        self._open_local_franka_stage()
        self._robot_prim_path = self._resolve_robot_prim_path()
        self._hand_prim_path = self._resolve_hand_prim_path()
        self._left_finger_prim_path = self._resolve_finger_prim_path("left")
        self._right_finger_prim_path = self._resolve_finger_prim_path("right")
        self._hand_camera_path = f"{self._hand_prim_path}/HandCamera"
        self._add_scene_markers()

        self._timeline.play()
        self._pump_updates(4)

        self._articulation = SingleArticulation(self._robot_prim_path)
        self._articulation.initialize()
        self._initialize_physics_handles()
        self._reset_physics_scene_state()
        self._set_articulation_positions(self._plan.home_dof_positions)
        self._pump_updates(6)
        self._top_down_hand_orientation, self._top_down_grasp_center_offset_world = self._capture_reference_hand_pose_data(
            self._plan.top_down_reference_dof_positions
        )

        motion_generation_root = Path(get_extension_path_from_name("isaacsim.robot_motion.motion_generation"))
        self._kinematics_solver = LulaKinematicsSolver(
            robot_description_path=str(motion_generation_root / "motion_policy_configs" / "franka" / "rmpflow" / "robot_descriptor.yaml"),
            urdf_path=str(motion_generation_root / "motion_policy_configs" / "franka" / "lula_franka_gen.urdf"),
        )
        self._articulation_kinematics_solver = ArticulationKinematicsSolver(
            self._articulation,
            self._kinematics_solver,
            "panda_hand",
        )

        if not self._headless:
            self._build_ui_window()
        self._set_camera_view(self._initial_camera_view)
        self._set_phase(HarvestPhase.READY, "Ready. Press Start to run the harvest scenario.")

    def _open_local_franka_stage(self) -> None:
        import omni.kit.app
        import omni.usd
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        extension_manager = omni.kit.app.get_app().get_extension_manager()
        extension_manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
        extension_manager.set_extension_enabled_immediate("isaacsim.robot_motion.motion_generation", True)

        urdf_path = ISAAC_SIM_ROOT / "exts/isaacsim.asset.importer.urdf/data/urdf/robots/franka_description/robots/panda_arm_hand.urdf"
        output_dir = Path(tempfile.mkdtemp(prefix="native_harvest_franka_"))

        importer = URDFImporter()
        config = URDFImporterConfig()
        config.urdf_path = str(urdf_path)
        config.usd_path = str(output_dir)
        config.merge_mesh = False
        config.collision_from_visuals = False
        importer.config = config
        output_path = importer.import_urdf()

        omni.usd.get_context().open_stage(output_path)
        self._stage = omni.usd.get_context().get_stage()
        robot_prim = self._stage.GetPrimAtPath("/panda_arm_hand")
        if robot_prim.IsValid():
            robot_prim.GetVariantSet("Physics").SetVariantSelection("physx")

    def _resolve_robot_prim_path(self) -> str:
        for prim_path in ("/panda", "/panda_arm_hand"):
            if self._stage.GetPrimAtPath(prim_path).IsValid():
                return prim_path
        raise RuntimeError("Could not resolve Franka robot prim path.")

    def _resolve_hand_prim_path(self) -> str:
        preferred_paths: list[str] = []
        fallback_paths: list[str] = []
        for prim in self._stage.Traverse():
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
        raise RuntimeError("Could not resolve panda_hand prim path.")

    def _resolve_finger_prim_path(self, side: str) -> str:
        preferred_paths: list[str] = []
        fallback_paths: list[str] = []
        suffix = f"/panda_{side}finger"
        for prim in self._stage.Traverse():
            prim_path = prim.GetPath().pathString
            if prim_path.endswith(suffix):
                if "/Geometry/" in prim_path:
                    preferred_paths.append(prim_path)
                else:
                    fallback_paths.append(prim_path)
        if preferred_paths:
            return preferred_paths[0]
        if fallback_paths:
            return fallback_paths[0]
        raise RuntimeError(f"Could not resolve panda_{side}finger prim path.")

    def _add_scene_markers(self) -> None:
        from isaacsim.core.api.materials import PhysicsMaterial
        from isaacsim.core.api.objects import DynamicSphere, FixedCuboid
        from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

        UsdGeom.Xform.Define(self._stage, "/World")
        UsdPhysics.Scene.Define(self._stage, "/World/PhysicsScene")

        fruit_material = PhysicsMaterial(
            prim_path="/World/PhysicsMaterials/Fruit",
            dynamic_friction=1.2,
            static_friction=1.4,
            restitution=0.02,
        )
        support_material = PhysicsMaterial(
            prim_path="/World/PhysicsMaterials/Support",
            dynamic_friction=0.9,
            static_friction=1.1,
            restitution=0.01,
        )

        FixedCuboid(
            prim_path="/World/GroundPlane",
            name="ground_plane",
            position=np.array([0.55, 0.0, -0.02], dtype=float),
            scale=np.array([2.0, 2.0, 0.04], dtype=float),
            size=1.0,
            color=np.array([0.22, 0.24, 0.26], dtype=float),
            physics_material=support_material,
        )

        key = UsdLux.DistantLight.Define(self._stage, "/World/KeyLight")
        key.CreateIntensityAttr(900.0)
        key.AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 0.0, 25.0))

        fill = UsdLux.SphereLight.Define(self._stage, "/World/FillLight")
        fill.CreateIntensityAttr(35000.0)
        fill.CreateRadiusAttr(0.25)
        fill.AddTranslateOp().Set(Gf.Vec3d(1.8, -1.6, 1.6))

        FixedCuboid(
            prim_path=self._branch_prim_path,
            name="tomato_branch",
            position=np.array(self._plan.branch_center_world_m, dtype=float),
            scale=np.array(self._plan.branch_scale_m, dtype=float),
            size=1.0,
            color=np.array([0.34, 0.52, 0.22], dtype=float),
            physics_material=support_material,
        )
        FixedCuboid(
            prim_path=self._stem_prim_path,
            name="tomato_stem",
            position=np.array(self._plan.stem_center_world_m, dtype=float),
            scale=np.array(self._plan.stem_scale_m, dtype=float),
            size=1.0,
            color=np.array([0.24, 0.58, 0.26], dtype=float),
            physics_material=support_material,
        )
        stem_anchor = UsdGeom.Xform.Define(self._stage, self._stem_anchor_prim_path)
        stem_anchor.AddTranslateOp().Set(Gf.Vec3d(*self._stem_joint_anchor_world_position()))
        self._apply_kinematic_rigid_body(self._stem_anchor_prim_path)

        self._tomato_body = DynamicSphere(
            prim_path=self._tomato_prim_path,
            name="target_tomato",
            position=np.array(self._plan.tomato_initial_world_m, dtype=float),
            radius=self._plan.tomato_radius_m,
            mass=self._plan.tomato_mass_kg,
            color=np.array([0.88, 0.16, 0.12], dtype=float),
            physics_material=fruit_material,
        )
        self._tray_contact_paths = self._create_place_tray(physics_material=support_material)

        fixed_camera = UsdGeom.Camera.Define(self._stage, self._fixed_camera_path)
        fixed_camera.AddTranslateOp().Set(Gf.Vec3d(1.20, -1.60, 1.20))
        fixed_camera.AddRotateXYZOp().Set(Gf.Vec3f(52.0, 0.0, 42.0))
        fixed_camera.GetFocalLengthAttr().Set(24.0)

        hand_camera = UsdGeom.Camera.Define(self._stage, self._hand_camera_path)
        hand_camera.AddTranslateOp().Set(Gf.Vec3d(*self._plan.hand_camera_local_offset_m))
        hand_camera.AddRotateXYZOp().Set(Gf.Vec3f(*self._plan.hand_camera_local_rotation_deg))
        hand_camera.GetFocalLengthAttr().Set(18.0)
        hand_camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1000.0))

    def _create_place_tray(self, *, physics_material: object) -> tuple[str, ...]:
        from isaacsim.core.api.objects import FixedCuboid

        inner_x, inner_y, inner_z = self._plan.tray_inner_size_m
        wall = self._plan.tray_wall_thickness_m
        base_top_z = self._plan.place_position_m[2] - self._plan.tomato_radius_m
        base_center = np.array(
            [
                self._plan.place_position_m[0],
                self._plan.place_position_m[1],
                base_top_z - wall * 0.5,
            ],
            dtype=float,
        )
        wall_center_z = base_top_z + inner_z * 0.5
        tray_paths = [self._tray_base_path]
        tray_specs = (
            (
                self._tray_base_path,
                base_center,
                np.array([inner_x + 2.0 * wall, inner_y + 2.0 * wall, wall], dtype=float),
            ),
            (
                f"{self._tray_root_path}/WallLeft",
                np.array(
                    [self._plan.place_position_m[0], self._plan.place_position_m[1] + (inner_y + wall) * 0.5, wall_center_z],
                    dtype=float,
                ),
                np.array([inner_x + 2.0 * wall, wall, inner_z], dtype=float),
            ),
            (
                f"{self._tray_root_path}/WallRight",
                np.array(
                    [self._plan.place_position_m[0], self._plan.place_position_m[1] - (inner_y + wall) * 0.5, wall_center_z],
                    dtype=float,
                ),
                np.array([inner_x + 2.0 * wall, wall, inner_z], dtype=float),
            ),
            (
                f"{self._tray_root_path}/WallFront",
                np.array(
                    [self._plan.place_position_m[0] + (inner_x + wall) * 0.5, self._plan.place_position_m[1], wall_center_z],
                    dtype=float,
                ),
                np.array([wall, inner_y, inner_z], dtype=float),
            ),
            (
                f"{self._tray_root_path}/WallBack",
                np.array(
                    [self._plan.place_position_m[0] - (inner_x + wall) * 0.5, self._plan.place_position_m[1], wall_center_z],
                    dtype=float,
                ),
                np.array([wall, inner_y, inner_z], dtype=float),
            ),
        )
        for prim_path, position, scale in tray_specs:
            FixedCuboid(
                prim_path=prim_path,
                name=Path(prim_path).name.lower(),
                position=position,
                scale=scale,
                size=1.0,
                color=np.array([0.16, 0.64, 0.86], dtype=float),
                physics_material=physics_material,
            )
            tray_paths.append(prim_path)
        return tuple(dict.fromkeys(tray_paths))

    def _apply_kinematic_rigid_body(self, prim_path: str) -> None:
        from pxr import PhysxSchema, UsdPhysics

        prim = self._stage.GetPrimAtPath(prim_path)
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI.Apply(prim)
        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
        rigid_body_api.CreateKinematicEnabledAttr(True)
        physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physx_rigid_body_api.CreateDisableGravityAttr(True)

    def _initialize_physics_handles(self) -> None:
        from isaacsim.core.prims import RigidPrim

        if self._tomato_body is not None:
            self._tomato_body.initialize()
        self._tomato_contact_view = RigidPrim(
            prim_paths_expr=self._tomato_prim_path,
            name="tomato_contact_view",
            track_contact_forces=True,
            contact_filter_prim_paths_expr=[
                self._left_finger_prim_path,
                self._right_finger_prim_path,
                *self._tray_contact_paths,
            ],
            max_contact_count=32,
        )
        self._tomato_contact_view.initialize()
        self._ensure_fruit_stem_joint()

    def _build_ui_window(self) -> None:
        import omni.ui as ui

        self._control_window = ui.Window(
            "Tomato Harvest Controls",
            width=320,
            height=230,
            visible=True,
        )
        self._control_window.position_x = 40
        self._control_window.position_y = 80
        with self._control_window.frame:
            with ui.VStack(spacing=8, height=0):
                ui.Label("Tomato Harvest Scenario", height=24)
                self._phase_label = ui.Label("Phase: Ready", height=24)
                self._status_label = ui.Label("Ready", height=36, word_wrap=True)
                self._camera_label = ui.Label("Camera: fixed", height=24)
                with ui.HStack(height=32, spacing=6):
                    ui.Button("Start", clicked_fn=self._on_start_clicked)
                    ui.Button("Stop", clicked_fn=self._on_stop_clicked)
                    ui.Button("Reset", clicked_fn=self._on_reset_clicked)
                with ui.HStack(height=32, spacing=6):
                    ui.Button("Fixed Camera", clicked_fn=self._on_fixed_camera_clicked)
                    ui.Button("Hand Camera", clicked_fn=self._on_hand_camera_clicked)

    def _update_runtime(self) -> None:
        if self._pending_ui_action is not None:
            action = self._pending_ui_action
            self._pending_ui_action = None
            action()
        if self._fruit_stem_joint_active and not self._fruit_hand_joint_active:
            self._maintain_stem_attachment()
        if self._animation is not None:
            self._advance_animation()
        if self._animation is None:
            self._advance_runtime_monitors()
        if self._test_mode and self._phase in {HarvestPhase.COMPLETE, HarvestPhase.FAILED} and self._animation is None:
            self._test_completed = True
            self._test_exit_code = 0 if self._phase == HarvestPhase.COMPLETE else 1

    def _advance_runtime_monitors(self) -> None:
        if self._grasp_check_frames_remaining > 0:
            self._grasp_check_frames_remaining -= 1
            if self._has_physical_grasp():
                self._grasp_contact_stable_frames += 1
            else:
                self._grasp_contact_stable_frames = 0
            if self._grasp_contact_stable_frames >= self._plan.grasp_hold_frame_count:
                self._finalize_grasp()
                return
            if self._grasp_check_frames_remaining == 0:
                self._scenario_active = False
                self._set_phase(HarvestPhase.FAILED, "The tomato was not stably grasped by both fingers.")
                return
        if self._settle_monitor_active:
            self._advance_settle_monitor()

    def _on_start_clicked(self) -> None:
        self._queue_ui_action(self._start_scenario)

    def _on_stop_clicked(self) -> None:
        self._queue_ui_action(self._stop_scenario)

    def _on_reset_clicked(self) -> None:
        self._queue_ui_action(self._reset_scene)

    def _on_fixed_camera_clicked(self) -> None:
        self._queue_ui_action(lambda: self._set_camera_view(CameraViewMode.FIXED))

    def _on_hand_camera_clicked(self) -> None:
        self._queue_ui_action(lambda: self._set_camera_view(CameraViewMode.HAND))

    def _queue_ui_action(self, action: Callable[[], None]) -> None:
        self._pending_ui_action = action

    def _stop_scenario(self) -> None:
        self._stop_requested = True
        self._animation = None
        self._scenario_active = False
        self._grasp_check_frames_remaining = 0
        self._settle_monitor_active = False
        self._pause_timeline()
        self._set_phase(HarvestPhase.STOPPED, "Scenario stopped. Press Reset to return home.")

    def _start_scenario(self) -> None:
        self._prepare_scene_for_start()
        self._play_timeline()
        self._pump_updates(2)
        self._stop_requested = False
        self._scenario_active = True
        self._set_phase(HarvestPhase.SEARCHING, "Searching for the tomato with the hand camera.")
        self._queue_scan_pose(0)

    def _prepare_scene_for_start(self) -> None:
        self._pause_timeline()
        self._animation = None
        self._scenario_active = False
        self._stop_requested = False
        self._grasp_check_frames_remaining = 0
        self._grasp_contact_stable_frames = 0
        self._settle_monitor_active = False
        self._settle_stable_frames = 0
        self._settle_elapsed_frames = 0
        self._found_camera_point = None
        self._found_world_point = None
        if self._fruit_hand_joint_active:
            self._remove_physics_joint(self._fruit_hand_joint_path)
            self._fruit_hand_joint_active = False
        self._set_articulation_positions(self._plan.home_dof_positions)
        self._pump_updates(4)

    def _reset_scene(self, *, reset_phase: bool = True) -> None:
        self._prepare_scene_for_start()
        self._reset_physics_scene_state()
        self._pump_updates(4)
        self._set_camera_view(CameraViewMode.FIXED)
        if reset_phase:
            self._set_phase(HarvestPhase.READY, "Scene reset complete. Press Start to run again.")

    def _queue_scan_pose(self, index: int) -> None:
        if self._stop_requested:
            return
        if index >= len(self._plan.scan_poses):
            self._scenario_active = False
            self._set_phase(HarvestPhase.FAILED, "Target was not found during the 360-degree search.")
            return

        pose = self._plan.scan_poses[index]
        self._set_phase(HarvestPhase.SEARCHING, f"Searching pose {index + 1}/{len(self._plan.scan_poses)}: {pose.label}")
        self._queue_joint_animation(
            target_positions=np.array(pose.dof_positions, dtype=float),
            frames=45,
            on_complete=lambda idx=index: self._complete_scan_pose(idx),
        )

    def _complete_scan_pose(self, index: int) -> None:
        if self._stop_requested:
            return
        camera_point, world_point = self._read_tomato_positions()
        target_visible = is_target_visible(
            camera_point,
            world_point,
            expected_height_m=self._plan.tomato_initial_world_m[2],
            xy_limit_m=self._plan.hand_camera_xy_limit_m,
            min_depth_m=self._plan.hand_camera_min_depth_m,
            max_depth_m=self._plan.hand_camera_max_depth_m,
            height_tolerance_m=self._plan.search_height_tolerance_m,
        )
        if target_visible:
            self._found_camera_point = camera_point
            self._found_world_point = world_point
            for line in build_target_found_messages(camera_point, world_point):
                print(line, flush=True)
            self._set_phase(HarvestPhase.TARGET_FOUND, "Target found. Solving IK for harvest.")
            self._queue_ik_approach(pre_grasp=True)
            return
        self._queue_scan_pose(index + 1)

    def _queue_ik_approach(self, *, pre_grasp: bool) -> None:
        if self._found_world_point is None:
            self._scenario_active = False
            self._set_phase(HarvestPhase.FAILED, "Target coordinates are unavailable.")
            return

        offset = self._plan.grasp_pre_offset_m if pre_grasp else self._plan.grasp_offset_m
        grasp_center_target = tuple(self._found_world_point[i] + offset[i] for i in range(3))
        target_position = self._compute_hand_target_position(grasp_center_target)
        self._set_phase(
            HarvestPhase.APPROACHING if pre_grasp else HarvestPhase.GRASPING,
            "Moving the hand toward the tomato using IK." if pre_grasp else "Closing in on the tomato.",
        )
        target_positions = self._solve_ik_joint_positions(
            target_position,
            target_orientation=self._top_down_hand_orientation,
        )
        if target_positions is None:
            self._scenario_active = False
            self._set_phase(HarvestPhase.FAILED, "IK failed while moving toward the tomato.")
            return
        self._queue_joint_animation(
            target_positions=target_positions,
            frames=70 if pre_grasp else 55,
            on_complete=lambda: self._queue_ik_approach(pre_grasp=False) if pre_grasp else self._close_gripper(),
        )

    def _close_gripper(self) -> None:
        current_positions = self._get_articulation_positions()
        target_positions = current_positions.copy()
        target_positions[7] = 0.0
        target_positions[8] = 0.0
        self._set_phase(HarvestPhase.GRASPING, "Closing the gripper around the tomato.")
        self._queue_joint_animation(target_positions=target_positions, frames=35, on_complete=self._begin_grasp_assessment)

    def _begin_grasp_assessment(self) -> None:
        self._set_phase(HarvestPhase.GRASPING, "Waiting for stable dual-finger contact on the tomato.")
        self._grasp_contact_stable_frames = 0
        self._grasp_check_frames_remaining = self._plan.grasp_hold_frame_count * 4

    def _finalize_grasp(self) -> None:
        self._grasp_check_frames_remaining = 0
        self._grasp_contact_stable_frames = 0
        if not self._create_hand_grasp_joint():
            self._scenario_active = False
            self._set_phase(HarvestPhase.FAILED, "Failed to create a physical grasp joint for the tomato.")
            return
        self._set_tomato_kinematic_enabled(False)
        self._fruit_stem_joint_active = False
        self._queue_pull_motion()

    def _queue_pull_motion(self) -> None:
        if self._found_world_point is None:
            self._scenario_active = False
            self._set_phase(HarvestPhase.FAILED, "Target coordinates are unavailable.")
            return
        pull_target = tuple(self._found_world_point[index] + self._plan.pull_offset_m[index] for index in range(3))
        target_positions = self._solve_ik_joint_positions(
            self._compute_hand_target_position(pull_target),
            target_orientation=self._top_down_hand_orientation,
        )
        if target_positions is None:
            self._scenario_active = False
            self._set_phase(HarvestPhase.FAILED, "IK failed while pulling the tomato away from the stem.")
            return
        self._set_phase(HarvestPhase.GRASPING, "Pulling the grasped tomato to break the stem joint.")
        self._queue_joint_animation(target_positions=target_positions, frames=45, on_complete=self._verify_detach_then_place)

    def _verify_detach_then_place(self) -> None:
        if not self._is_tomato_detached():
            self._scenario_active = False
            self._set_phase(HarvestPhase.FAILED, "The tomato did not detach from the stem during the pull motion.")
            return
        self._queue_place_motion()

    def _queue_place_motion(self) -> None:
        self._set_phase(HarvestPhase.PLACING, "Moving the harvested tomato to the place target.")
        place_pre_position = tuple(
            self._plan.place_position_m[index] + self._plan.place_pre_offset_m[index] for index in range(3)
        )
        target_positions = self._solve_ik_joint_positions(
            self._compute_hand_target_position(place_pre_position),
            target_orientation=self._top_down_hand_orientation,
        )
        if target_positions is None:
            self._scenario_active = False
            self._set_phase(HarvestPhase.FAILED, "IK failed while moving to the place target.")
            return
        self._queue_joint_animation(target_positions=target_positions, frames=90, on_complete=self._queue_place_descent)

    def _queue_place_descent(self) -> None:
        target_positions = self._solve_ik_joint_positions(
            self._compute_hand_target_position(self._plan.place_position_m),
            target_orientation=self._top_down_hand_orientation,
        )
        if target_positions is None:
            self._scenario_active = False
            self._set_phase(HarvestPhase.FAILED, "IK failed while lowering the tomato into the tray.")
            return
        self._queue_joint_animation(target_positions=target_positions, frames=55, on_complete=self._release_tomato)

    def _release_tomato(self) -> None:
        current_positions = self._get_articulation_positions()
        target_positions = current_positions.copy()
        target_positions[7] = 0.04
        target_positions[8] = 0.04
        self._queue_joint_animation(target_positions=target_positions, frames=30, on_complete=self._release_tomato_and_retreat)

    def _release_tomato_and_retreat(self) -> None:
        self._remove_physics_joint(self._fruit_hand_joint_path)
        self._fruit_hand_joint_active = False
        retreat_position = tuple(
            self._plan.place_position_m[index] + self._plan.place_retreat_offset_m[index] for index in range(3)
        )
        target_positions = self._solve_ik_joint_positions(
            self._compute_hand_target_position(retreat_position),
            target_orientation=self._top_down_hand_orientation,
        )
        if target_positions is None:
            self._scenario_active = False
            self._set_phase(HarvestPhase.FAILED, "IK failed while retreating from the tray.")
            return
        self._set_phase(HarvestPhase.PLACING, "Released the tomato. Retreating and waiting for tray settling.")
        self._queue_joint_animation(target_positions=target_positions, frames=50, on_complete=self._start_settle_monitor)

    def _start_settle_monitor(self) -> None:
        self._settle_monitor_active = True
        self._settle_stable_frames = 0
        self._settle_elapsed_frames = 0
        self._set_phase(HarvestPhase.PLACING, "Waiting for the tomato to settle in the tray.")

    def _complete_scenario(self) -> None:
        self._settle_monitor_active = False
        self._scenario_active = False
        self._pause_timeline()
        self._set_phase(HarvestPhase.COMPLETE, "Scenario complete. The tomato has been placed.")

    def _queue_joint_animation(
        self,
        *,
        target_positions: np.ndarray,
        frames: int,
        on_complete: Callable[[], None] | None,
    ) -> None:
        self._animation = JointAnimation(
            start_positions=self._get_articulation_positions(),
            target_positions=target_positions,
            frames=max(frames, 1),
            on_complete=on_complete,
        )

    def _advance_animation(self) -> None:
        animation = self._animation
        progress = float(animation.frame_index + 1) / float(animation.frames)
        positions = animation.start_positions + (animation.target_positions - animation.start_positions) * progress
        self._set_articulation_positions(positions)
        animation.frame_index += 1
        if animation.frame_index >= animation.frames:
            self._animation = None
            if animation.on_complete is not None:
                animation.on_complete()

    def _solve_ik_joint_positions(
        self,
        target_position_m: tuple[float, float, float],
        *,
        target_orientation: np.ndarray | None = None,
    ) -> np.ndarray | None:
        robot_base_translation, robot_base_orientation = self._articulation.get_world_pose()
        self._kinematics_solver.set_robot_base_pose(robot_base_translation, robot_base_orientation)
        if target_orientation is None:
            _, target_orientation = self._read_hand_pose()
        action, success = self._articulation_kinematics_solver.compute_inverse_kinematics(
            np.array(target_position_m, dtype=float),
            np.array(target_orientation, dtype=float),
            position_tolerance=0.015,
            orientation_tolerance=0.15,
        )
        if not success:
            return None
        return self._merge_action_joint_positions(action)

    def _capture_reference_hand_pose_data(self, dof_positions: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        current_positions = self._get_articulation_positions()
        self._set_articulation_positions(dof_positions)
        self._pump_updates(6)
        hand_position, reference_orientation = self._read_hand_pose()
        grasp_center_position = self._read_grasp_center_position()
        self._set_articulation_positions(current_positions)
        self._pump_updates(6)
        return (
            np.array(reference_orientation, dtype=float),
            np.array(grasp_center_position - hand_position, dtype=float),
        )

    def _merge_action_joint_positions(self, action: object) -> np.ndarray:
        current_positions = self._get_articulation_positions()
        action_positions = np.array(action.joint_positions, dtype=float)
        if action.joint_indices is None:
            current_positions[: len(action_positions)] = action_positions
            return current_positions
        current_positions[np.array(action.joint_indices, dtype=int)] = action_positions
        return current_positions

    def _read_tomato_positions(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        from pxr import Gf

        tomato_world_matrix = self._compute_local_to_world(self._tomato_prim_path)
        hand_camera_world_matrix = self._compute_local_to_world(self._hand_camera_path)
        tomato_world = tomato_world_matrix.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
        tomato_camera = hand_camera_world_matrix.GetInverse().Transform(tomato_world)
        return (
            (tomato_camera[0], tomato_camera[1], tomato_camera[2]),
            (tomato_world[0], tomato_world[1], tomato_world[2]),
        )

    def _read_tomato_world_position(self) -> tuple[float, float, float]:
        if self._tomato_body is not None:
            tomato_position, _ = self._tomato_body.get_world_pose()
            return (float(tomato_position[0]), float(tomato_position[1]), float(tomato_position[2]))
        _, world_point = self._read_tomato_positions()
        return world_point

    def _read_grasp_center_position(self) -> np.ndarray:
        from pxr import Gf

        hand_world_matrix = self._compute_local_to_world(self._hand_prim_path)
        grasp_center = hand_world_matrix.Transform(Gf.Vec3d(*self._plan.grasp_center_local_offset_m))
        return np.array([grasp_center[0], grasp_center[1], grasp_center[2]], dtype=float)

    def _compute_hand_target_position(
        self,
        grasp_center_target_position_m: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        if self._top_down_grasp_center_offset_world is None:
            return grasp_center_target_position_m
        return tuple(
            float(grasp_center_target_position_m[index] - self._top_down_grasp_center_offset_world[index])
            for index in range(3)
        )

    def _read_hand_pose(self) -> tuple[np.ndarray, np.ndarray]:
        from isaacsim.core.utils.numpy.rotations import rot_matrices_to_quats

        hand_world_matrix = self._compute_local_to_world(self._hand_prim_path)
        hand_position = hand_world_matrix.Transform((0.0, 0.0, 0.0))
        hand_rotation = np.array(
            [[hand_world_matrix[row_index][column_index] for column_index in range(3)] for row_index in range(3)],
            dtype=float,
        )
        hand_orientation = rot_matrices_to_quats(hand_rotation[np.newaxis, :, :])[0]
        return (
            np.array([hand_position[0], hand_position[1], hand_position[2]], dtype=float),
            np.array(hand_orientation, dtype=float),
        )

    def _compute_local_to_world(self, prim_path: str) -> object:
        from pxr import UsdGeom

        return UsdGeom.Xformable(self._stage.GetPrimAtPath(prim_path)).ComputeLocalToWorldTransform(0.0)

    def _set_tomato_world_position(self, position_m: tuple[float, float, float]) -> None:
        if self._tomato_body is not None:
            self._tomato_body.set_world_pose(position=np.array(position_m, dtype=float))
            return
        if self._stage is None:
            return
        from pxr import Gf, UsdGeom

        prim = self._stage.GetPrimAtPath(self._tomato_prim_path)
        xform = UsdGeom.Xformable(prim)
        translate_ops = [op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
        translate_op = translate_ops[0] if translate_ops else xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(*position_m))

    def _reset_physics_scene_state(self) -> None:
        if self._stage is None:
            return
        self._remove_physics_joint(self._fruit_hand_joint_path)
        self._remove_physics_joint(self._fruit_stem_joint_path)
        self._fruit_hand_joint_active = False
        self._fruit_stem_joint_active = False
        self._set_tomato_kinematic_enabled(True)
        self._set_tomato_world_position(self._plan.tomato_initial_world_m)
        if self._tomato_body is not None:
            self._tomato_body.set_linear_velocity(np.zeros(3, dtype=float))
            self._tomato_body.set_angular_velocity(np.zeros(3, dtype=float))
        self._pump_updates(2)
        self._ensure_fruit_stem_joint()
        self._pump_updates(1)

    def _ensure_fruit_stem_joint(self) -> None:
        self._fruit_stem_joint_active = True
        self._set_tomato_kinematic_enabled(True)
        self._maintain_stem_attachment()

    def _create_hand_grasp_joint(self) -> bool:
        tomato_position = self._read_tomato_world_position()
        created = self._create_fixed_joint(
            joint_path=self._fruit_hand_joint_path,
            body0_path=self._hand_prim_path,
            body1_path=self._tomato_prim_path,
            anchor_world_position_m=tomato_position,
        )
        self._fruit_hand_joint_active = created
        return created

    def _create_fixed_joint(
        self,
        *,
        joint_path: str,
        body0_path: str,
        body1_path: str,
        anchor_world_position_m: tuple[float, float, float],
        break_force_n: float | None = None,
        break_torque_nm: float | None = None,
    ) -> bool:
        from pxr import Gf, Sdf, UsdPhysics

        if self._stage is None:
            return False
        self._remove_physics_joint(joint_path)
        joint = UsdPhysics.FixedJoint.Define(self._stage, joint_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(body0_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(body1_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*self._world_to_local_point(body0_path, anchor_world_position_m)))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*self._world_to_local_point(body1_path, anchor_world_position_m)))
        if break_force_n is not None:
            joint.CreateBreakForceAttr(float(break_force_n))
        if break_torque_nm is not None:
            joint.CreateBreakTorqueAttr(float(break_torque_nm))
        return True

    def _remove_physics_joint(self, joint_path: str) -> None:
        if self._stage is None:
            return
        prim = self._stage.GetPrimAtPath(joint_path)
        if prim.IsValid():
            self._stage.RemovePrim(joint_path)

    def _world_to_local_point(self, prim_path: str, world_point_m: tuple[float, float, float]) -> tuple[float, float, float]:
        from pxr import Gf

        local_point = self._compute_local_to_world(prim_path).GetInverse().Transform(Gf.Vec3d(*world_point_m))
        return (float(local_point[0]), float(local_point[1]), float(local_point[2]))

    def _set_tomato_kinematic_enabled(self, enabled: bool) -> None:
        from pxr import PhysxSchema, UsdPhysics

        if self._stage is None:
            return
        prim = self._stage.GetPrimAtPath(self._tomato_prim_path)
        if not prim.IsValid():
            return
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI.Apply(prim)
        rigid_body_api = UsdPhysics.RigidBodyAPI(prim)
        rigid_body_api.CreateKinematicEnabledAttr(bool(enabled))
        physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physx_rigid_body_api.CreateDisableGravityAttr(bool(enabled))

    def _has_physical_grasp(self) -> bool:
        current_positions = self._get_articulation_positions()
        if current_positions[7] > 0.012 or current_positions[8] > 0.012:
            return False
        return has_dual_finger_contact(
            self._read_finger_contact_forces(),
            force_threshold_n=self._plan.finger_contact_force_threshold_n,
        )

    def _read_finger_contact_forces(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        if self._tomato_contact_view is None:
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        matrix = self._tomato_contact_view.get_contact_force_matrix(dt=self._physics_dt())
        if matrix is None:
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        contact_matrix = np.asarray(matrix, dtype=float)
        if contact_matrix.ndim < 3 or contact_matrix.shape[1] < 2:
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        left_force = tuple(float(value) for value in contact_matrix[0, 0, :3])
        right_force = tuple(float(value) for value in contact_matrix[0, 1, :3])
        return (left_force, right_force)

    def _physics_dt(self) -> float:
        return 1.0 / 60.0

    def _is_tomato_detached(self) -> bool:
        tomato_position = np.array(self._read_tomato_world_position(), dtype=float)
        initial_position = np.array(self._plan.tomato_initial_world_m, dtype=float)
        return float(np.linalg.norm(tomato_position - initial_position)) >= 0.035

    def _read_tomato_velocities(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        if self._tomato_body is None:
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        linear_velocity = self._tomato_body.get_linear_velocity()
        angular_velocity = self._tomato_body.get_angular_velocity()
        return (
            (float(linear_velocity[0]), float(linear_velocity[1]), float(linear_velocity[2])),
            (float(angular_velocity[0]), float(angular_velocity[1]), float(angular_velocity[2])),
        )

    def _stem_joint_anchor_world_position(self) -> tuple[float, float, float]:
        return (
            self._plan.tomato_initial_world_m[0],
            self._plan.tomato_initial_world_m[1],
            self._plan.tomato_initial_world_m[2] + self._plan.tomato_radius_m * 0.85,
        )

    def _maintain_stem_attachment(self) -> None:
        self._set_tomato_world_position(self._plan.tomato_initial_world_m)
        if self._tomato_body is not None:
            self._tomato_body.set_linear_velocity(np.zeros(3, dtype=float))
            self._tomato_body.set_angular_velocity(np.zeros(3, dtype=float))

    def _is_tomato_in_tray_bounds(self) -> bool:
        return is_point_in_box_xy(
            self._read_tomato_world_position(),
            self._plan.place_position_m,
            self._plan.tray_inner_size_m,
            margin_m=self._plan.tomato_radius_m * 0.2,
        )

    def _advance_settle_monitor(self) -> None:
        self._settle_elapsed_frames += 1
        linear_velocity, angular_velocity = self._read_tomato_velocities()
        if self._is_tomato_in_tray_bounds() and is_object_settled(
            linear_velocity,
            angular_velocity,
            linear_speed_threshold_mps=self._plan.settle_linear_speed_threshold_mps,
            angular_speed_threshold_radps=self._plan.settle_angular_speed_threshold_radps,
        ):
            self._settle_stable_frames += 1
        else:
            self._settle_stable_frames = 0
        if self._settle_stable_frames >= self._plan.settle_frame_count:
            self._complete_scenario()
            return
        if self._settle_elapsed_frames >= self._plan.settle_timeout_frames:
            self._settle_monitor_active = False
            self._scenario_active = False
            self._set_phase(HarvestPhase.FAILED, "The tomato did not settle inside the tray before timeout.")

    def _set_articulation_positions(self, positions: Sequence[float]) -> None:
        self._articulation.set_joint_positions(np.array(list(positions), dtype=float))

    def _get_articulation_positions(self) -> np.ndarray:
        return np.array(self._articulation.get_joint_positions(), dtype=float)

    def _set_camera_view(self, camera_view: CameraViewMode) -> None:
        self._active_camera_view = camera_view
        camera_path = self._fixed_camera_path if camera_view == CameraViewMode.FIXED else self._hand_camera_path
        if not self._headless:
            self._try_set_active_camera(camera_path)
        if self._camera_label is not None:
            self._camera_label.text = f"Camera: {camera_view.value}"

    def _try_set_active_camera(self, camera_prim_path: str) -> None:
        try:
            import omni.kit.app
            import omni.kit.viewport.utility

            app = omni.kit.app.get_app()
            for _ in range(120):
                viewport = omni.kit.viewport.utility.get_active_viewport()
                if viewport is not None:
                    viewport.camera_path = camera_prim_path
                    return
                app.update()
                time.sleep(0.01)
        except Exception:
            return

    def _set_phase(self, phase: HarvestPhase, message: str) -> None:
        self._phase = phase
        self._status_message = message
        print(f"[{phase.value}] {message}", flush=True)
        if self._phase_label is not None:
            self._phase_label.text = f"Phase: {phase.value}"
        if self._status_label is not None:
            self._status_label.text = message

    def _pump_updates(self, frame_count: int) -> None:
        for _ in range(frame_count):
            self._simulation_app.update()

    def _play_timeline(self) -> None:
        if self._timeline is not None:
            self._timeline.play()

    def _pause_timeline(self) -> None:
        if self._timeline is not None:
            self._timeline.pause()
