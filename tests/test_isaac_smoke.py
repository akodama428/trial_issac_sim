from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest

from tomato_harvest_poc.isaac_smoke import build_smoke_scene_plan


class IsaacSmokeScenePlanTest(unittest.TestCase):
    def test_plan_contains_minimum_3d_scene_prims(self) -> None:
        plan = build_smoke_scene_plan()

        self.assertEqual(plan.camera_prim_path, "/World/Camera_EyeToHand")
        self.assertIn("/World/GroundPlane", plan.required_prim_paths)
        self.assertIn("/World/SmokeCube", plan.required_prim_paths)
        self.assertIn("/World/TargetTomato", plan.required_prim_paths)

    def test_plan_uses_project_eye_to_hand_camera_pose(self) -> None:
        plan = build_smoke_scene_plan()

        self.assertEqual(plan.camera_position_m, (0.8, 0.0, 1.35))
        self.assertEqual(plan.camera_rotation_deg, (-30.0, 0.0, 180.0))

    def test_launch_command_uses_isaac_python(self) -> None:
        plan = build_smoke_scene_plan()

        self.assertEqual(
            plan.container_command,
            "/isaac-sim/python.sh scripts/isaac_viewport_smoke.py",
        )


class IsaacViewportSmokeArgsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        script_path = Path(__file__).resolve().parents[1] / "scripts" / "isaac_viewport_smoke.py"
        spec = importlib.util.spec_from_file_location("isaac_viewport_smoke", script_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Failed to load smoke script from {script_path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        cls.script = module

    def test_defaults_to_minimal_local_viewport(self) -> None:
        args = self.script.parse_args([])

        self.assertFalse(args.headless)
        self.assertFalse(args.with_franka)
        self.assertFalse(args.enable_livestream)
        self.assertFalse(args.use_eye_to_hand_camera)
        self.assertEqual(args.timeout_seconds, 0.0)

    def test_accepts_explicit_franka_and_livestream_flags(self) -> None:
        args = self.script.parse_args(
            ["--with-franka", "--enable-livestream", "--timeout-seconds", "120"]
        )

        self.assertTrue(args.with_franka)
        self.assertTrue(args.enable_livestream)
        self.assertEqual(args.timeout_seconds, 120.0)

    def test_appframework_argv_uses_local_ext_paths(self) -> None:
        argv = self.script.build_appframework_argv(headless=False)

        self.assertIn("/isaac-sim/exts", argv)
        self.assertIn("/isaac-sim/extscache", argv)
        self.assertIn("/isaac-sim/apps", argv)
        self.assertIn("--/app/viewport/defaultCamPos/x=1.6", argv)
        self.assertIn("--/app/viewport/defaultCamPos/y=-1.6", argv)
        self.assertIn("--/app/viewport/defaultCamPos/z=1.1", argv)
        self.assertIn("--/persistent/renderer/startupMessageDisplayed=true", argv)
        self.assertIn("--/renderer/asyncInit=true", argv)
        self.assertIn("--/app/hangDetector/timeout=300", argv)
        self.assertIn("omni.kit.manipulator.camera", argv)
        self.assertIn("omni.kit.manipulator.prim", argv)
        self.assertIn("omni.kit.manipulator.selection", argv)
        self.assertIn("omni.kit.viewport.actions", argv)
        self.assertIn("omni.kit.viewport.legacy_gizmos", argv)
        self.assertIn("omni.kit.viewport.window", argv)
        self.assertIn("omni.kit.viewport.utility", argv)
        self.assertIn("omni.kit.window.status_bar", argv)
        self.assertIn("omni.kit.window.toolbar", argv)
        self.assertIn("omni.hydra.rtx", argv)
        self.assertNotIn("--no-window", argv)

    def test_headless_appframework_argv_disables_window(self) -> None:
        argv = self.script.build_appframework_argv(headless=True)

        self.assertIn("--no-window", argv)

    def test_robot_visual_selection_defaults_to_proxy_without_flag(self) -> None:
        load_franka, message = self.script.choose_robot_visual(
            with_franka_requested=False,
            franka_asset_available=False,
        )

        self.assertFalse(load_franka)
        self.assertIn("robot proxy", message)

    def test_robot_visual_selection_uses_franka_when_requested_and_available(self) -> None:
        load_franka, message = self.script.choose_robot_visual(
            with_franka_requested=True,
            franka_asset_available=True,
        )

        self.assertTrue(load_franka)
        self.assertIn("Franka USD asset", message)

    def test_proxy_robot_pose_changes_over_time(self) -> None:
        pose_a = self.script.compute_proxy_robot_pose(0.0)
        pose_b = self.script.compute_proxy_robot_pose(0.8)

        self.assertNotEqual(pose_a.wrist_position, pose_b.wrist_position)
        self.assertNotEqual(pose_a.left_finger_position, pose_b.left_finger_position)
        self.assertNotEqual(pose_a.forearm_rotation_deg, pose_b.forearm_rotation_deg)
