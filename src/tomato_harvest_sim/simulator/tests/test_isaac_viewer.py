from __future__ import annotations

import unittest

from tomato_harvest_sim.simulator.isaac_viewer import (
    OFFICIAL_FRANKA_ASSET_RELATIVE_PATH,
    build_appframework_argv,
    build_official_franka_asset_path,
    build_simulation_app_config,
    is_detach_intent_phase,
    is_headless_terminal_phase,
    parse_args,
    select_hand_mount_prim_path,
    stem_physics_prim_path,
)
from tomato_harvest_sim.simulator.scene_config import load_scene_layout_config
from tomato_harvest_sim.simulator.scene_plan import build_review_scene_plan


class IsaacViewerConfigTest(unittest.TestCase):
    def test_contact_rich_scene_uses_120_hz_physics(self) -> None:
        from tomato_harvest_sim.simulator.isaac_viewer import PHYSICS_STEPS_PER_SECOND

        self.assertEqual(PHYSICS_STEPS_PER_SECOND, 120)

    def test_parse_args_defaults_to_fixed_camera_gui_mode(self) -> None:
        args = parse_args([])

        self.assertFalse(args.headless)
        self.assertEqual(args.headless_steps, 64)
        self.assertEqual(args.camera_view, "fixed")
        self.assertEqual(args.grasp_mode, "success")

    def test_parse_args_accepts_physics_grasp_mode(self) -> None:
        args = parse_args(["--grasp-mode", "physics"])

        self.assertEqual(args.grasp_mode, "physics")

    def test_build_appframework_argv_includes_viewport_extensions(self) -> None:
        argv = build_appframework_argv(headless=False)

        self.assertIn("omni.kit.viewport.window", argv)
        self.assertIn("omni.kit.viewport.utility", argv)
        self.assertIn("omni.kit.window.toolbar", argv)
        self.assertNotIn("--no-window", argv)

    def test_build_appframework_argv_adds_no_window_in_headless_mode(self) -> None:
        argv = build_appframework_argv(headless=True)

        self.assertIn("--no-window", argv)

    def test_build_simulation_app_config_uses_gui_defaults(self) -> None:
        config = build_simulation_app_config(headless=False)

        self.assertFalse(config["headless"])
        self.assertEqual(config["renderer"], "RaytracedLighting")
        self.assertFalse(config["disable_viewport_updates"])
        self.assertIn("--/app/hangDetector/timeout=300", config["extra_args"])
        self.assertIn("--empty", config["extra_args"])
        self.assertIn("omni.kit.viewport.window", config["extra_args"])

    def test_build_official_franka_asset_path_appends_known_relative_path(self) -> None:
        path = build_official_franka_asset_path("https://assets.example.com/Assets/Isaac/6.0/")

        self.assertEqual(
            path,
            "https://assets.example.com/Assets/Isaac/6.0/" + OFFICIAL_FRANKA_ASSET_RELATIVE_PATH,
        )

    def test_review_scene_plan_contains_reviewable_scene_items(self) -> None:
        layout = load_scene_layout_config()
        plan = build_review_scene_plan()

        self.assertEqual(plan.robot_prim_path, "/World/FrankaPanda")
        self.assertEqual(plan.fixed_camera_prim_path, "/World/Camera_Fixed")
        self.assertEqual(plan.hand_camera_mount_prim_suffix, "panda_hand")
        self.assertEqual(plan.hand_camera_prim_name, "HandCamera")
        self.assertEqual(plan.fixed_camera_pose.x, layout.fixed_camera_pose.x)
        self.assertEqual(plan.fixed_camera_pose.y, layout.fixed_camera_pose.y)
        self.assertEqual(plan.fixed_camera_focal_length_mm, layout.fixed_camera_focal_length_mm)
        self.assertEqual(plan.tray_inner_size_m, layout.tray_inner_size_m)
        self.assertEqual(plan.tray_wall_thickness_m, layout.tray_wall_thickness_m)
        self.assertEqual(plan.hand_camera_pose.z, layout.hand_camera_pose.z)
        self.assertEqual(plan.hand_camera_pose.pitch, layout.hand_camera_pose.pitch)
        self.assertEqual(plan.hand_camera_clipping_range_m, layout.hand_camera_clipping_range_m)
        self.assertIn("/World/TomatoBranch", plan.required_prim_paths)
        self.assertIn("/World/TomatoStem", plan.required_prim_paths)
        self.assertIn("/World/TargetTomato", plan.required_prim_paths)
        self.assertIn("/World/PlaceTray", plan.required_prim_paths)
        self.assertIn("/World/RobotToolProxy", plan.required_prim_paths)

    def test_visible_stem_is_the_physics_rigid_body(self) -> None:
        plan = build_review_scene_plan()

        self.assertEqual(stem_physics_prim_path(plan), "/World/TomatoStem")

    def test_select_hand_mount_prim_path_prefers_geometry_hand(self) -> None:
        prim_path = select_hand_mount_prim_path(
            (
                "/World/FrankaPanda/panda_hand",
                "/World/FrankaPanda/Geometry/panda_hand",
            ),
            hand_mount_prim_suffix="panda_hand",
        )

        self.assertEqual(prim_path, "/World/FrankaPanda/Geometry/panda_hand")

    def test_build_simulation_app_config_adds_no_window_in_headless_mode(self) -> None:
        config = build_simulation_app_config(headless=True)

        self.assertIn("--no-window", config["extra_args"])


class HeadlessTerminalPhaseTest(unittest.TestCase):
    """収穫サイクルの終端フェーズを検知したらヘッドレス実行を早期終了する仕様。"""

    def test_harvest_cycle_completion_stops_headless_run(self) -> None:
        self.assertTrue(is_headless_terminal_phase("complete"))

    def test_only_detaching_phase_activates_detach_intent(self) -> None:
        self.assertTrue(is_detach_intent_phase("detaching"))
        for phase in (None, "grasp_evaluation", "moving_to_place", "failed"):
            with self.subTest(phase=phase):
                self.assertFalse(is_detach_intent_phase(phase))

    def test_harvest_cycle_failure_stops_headless_run(self) -> None:
        self.assertTrue(is_headless_terminal_phase("failed"))

    def test_transient_phases_keep_headless_run_going(self) -> None:
        for phase in ("idle", "detecting", "target_found", "moving_to_place", "returning_home"):
            with self.subTest(phase=phase):
                self.assertFalse(is_headless_terminal_phase(phase))

    def test_unobserved_phase_keeps_headless_run_going(self) -> None:
        self.assertFalse(is_headless_terminal_phase(None))

    def test_unknown_phase_value_keeps_headless_run_going(self) -> None:
        self.assertFalse(is_headless_terminal_phase("unexpected_value"))


if __name__ == "__main__":
    unittest.main()
