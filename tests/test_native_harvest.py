from __future__ import annotations

import unittest

from tomato_harvest_poc.native_harvest import (
    CameraViewMode,
    HarvestPhase,
    build_harvest_scenario_plan,
    build_target_found_messages,
    format_xyz,
    is_object_settled,
    is_point_in_box_xy,
    is_target_visible,
    has_dual_finger_contact,
)


class NativeHarvestPlanTest(unittest.TestCase):
    def test_plan_matches_expected_poc_defaults(self) -> None:
        plan = build_harvest_scenario_plan()

        self.assertEqual(plan.tomato_radius_m, 0.01)
        self.assertEqual(plan.branch_center_world_m, (0.50, 0.0, 0.57))
        self.assertEqual(plan.branch_scale_m, (0.22, 0.03, 0.03))
        self.assertEqual(plan.tomato_initial_world_m, (0.50, 0.0, 0.42))
        self.assertEqual(plan.place_pre_offset_m, (0.0, 0.0, 0.12))
        self.assertEqual(plan.place_retreat_offset_m, (0.0, 0.0, 0.16))
        self.assertEqual(plan.place_position_m, (0.35, -0.45, 0.385))
        self.assertEqual(plan.hand_camera_local_offset_m, (0.0, 0.0, 0.10))
        self.assertEqual(plan.hand_camera_local_rotation_deg, (0.0, 180.0, 0.0))
        self.assertEqual(plan.grasp_center_local_offset_m, (0.0, 0.0, 0.1034))
        self.assertEqual(plan.stem_center_world_m, (0.50, 0.0, 0.435))
        self.assertEqual(plan.stem_scale_m, (0.012, 0.012, 0.05))
        self.assertEqual(plan.pull_offset_m, (0.0, 0.0, 0.10))
        self.assertEqual(plan.tray_inner_size_m, (0.12, 0.18, 0.06))
        self.assertGreater(plan.stem_break_force_n, 0.0)
        self.assertGreater(plan.finger_contact_force_threshold_n, 0.0)
        self.assertGreater(plan.settle_frame_count, 0)
        self.assertGreaterEqual(len(plan.scan_poses), 6)
        self.assertEqual(len(plan.home_dof_positions), 9)
        self.assertEqual(len(plan.top_down_reference_dof_positions), 9)
        self.assertGreater(plan.grasp_pre_offset_m[2], plan.grasp_offset_m[2])
        self.assertEqual(plan.grasp_offset_m[2], 0.0)
        self.assertTrue(any("front" in pose.label for pose in plan.scan_poses))
        self.assertTrue(any("back" in pose.label for pose in plan.scan_poses))

    def test_target_visibility_requires_depth_xy_and_height_constraints(self) -> None:
        plan = build_harvest_scenario_plan()
        expected_height = plan.tomato_initial_world_m[2]

        self.assertTrue(
            is_target_visible(
                (0.01, -0.02, -0.12),
                (0.50, 0.00, expected_height),
                expected_height_m=expected_height,
                xy_limit_m=plan.hand_camera_xy_limit_m,
                min_depth_m=plan.hand_camera_min_depth_m,
                max_depth_m=plan.hand_camera_max_depth_m,
                height_tolerance_m=plan.search_height_tolerance_m,
            )
        )
        self.assertFalse(
            is_target_visible(
                (0.70, 0.00, -0.12),
                (0.50, 0.00, expected_height),
                expected_height_m=expected_height,
                xy_limit_m=plan.hand_camera_xy_limit_m,
                min_depth_m=plan.hand_camera_min_depth_m,
                max_depth_m=plan.hand_camera_max_depth_m,
                height_tolerance_m=plan.search_height_tolerance_m,
            )
        )
        self.assertFalse(
            is_target_visible(
                (0.01, -0.02, 1.70),
                (0.50, 0.00, expected_height),
                expected_height_m=expected_height,
                xy_limit_m=plan.hand_camera_xy_limit_m,
                min_depth_m=plan.hand_camera_min_depth_m,
                max_depth_m=plan.hand_camera_max_depth_m,
                height_tolerance_m=plan.search_height_tolerance_m,
            )
        )
        self.assertFalse(
            is_target_visible(
                (0.01, -0.02, -0.12),
                (0.50, 0.00, expected_height + 0.30),
                expected_height_m=expected_height,
                xy_limit_m=plan.hand_camera_xy_limit_m,
                min_depth_m=plan.hand_camera_min_depth_m,
                max_depth_m=plan.hand_camera_max_depth_m,
                height_tolerance_m=plan.search_height_tolerance_m,
            )
        )

    def test_target_found_messages_include_camera_and_world_coordinates(self) -> None:
        messages = build_target_found_messages((0.01, -0.02, -0.12), (0.64, 0.0, 0.55))

        self.assertEqual(messages[0], "Target is Found!")
        self.assertEqual(messages[1], "Tomato camera xyz: (0.0100, -0.0200, -0.1200)")
        self.assertEqual(messages[2], "Tomato world xyz: (0.6400, 0.0000, 0.5500)")

    def test_enums_expose_expected_ui_values(self) -> None:
        self.assertEqual(CameraViewMode.FIXED.value, "fixed")
        self.assertEqual(CameraViewMode.HAND.value, "hand")
        self.assertEqual(HarvestPhase.COMPLETE.value, "Complete")
        self.assertEqual(format_xyz((1.0, 2.0, 3.0)), "(1.0000, 2.0000, 3.0000)")

    def test_dual_finger_contact_requires_both_fingers_over_threshold(self) -> None:
        self.assertTrue(
            has_dual_finger_contact(
                ((0.0, 0.0, 2.4), (0.0, 0.0, 2.1)),
                force_threshold_n=1.8,
            )
        )
        self.assertFalse(
            has_dual_finger_contact(
                ((0.0, 0.0, 2.4), (0.0, 0.0, 1.2)),
                force_threshold_n=1.8,
            )
        )

    def test_object_settled_requires_low_linear_and_angular_velocity(self) -> None:
        self.assertTrue(
            is_object_settled(
                (0.001, -0.002, 0.001),
                (0.02, 0.01, -0.01),
                linear_speed_threshold_mps=0.01,
                angular_speed_threshold_radps=0.10,
            )
        )
        self.assertFalse(
            is_object_settled(
                (0.040, 0.0, 0.0),
                (0.02, 0.01, -0.01),
                linear_speed_threshold_mps=0.01,
                angular_speed_threshold_radps=0.10,
            )
        )
        self.assertFalse(
            is_object_settled(
                (0.001, -0.002, 0.001),
                (0.20, 0.01, -0.01),
                linear_speed_threshold_mps=0.01,
                angular_speed_threshold_radps=0.10,
            )
        )

    def test_point_in_box_xy_checks_tray_bounds_with_margin(self) -> None:
        self.assertTrue(
            is_point_in_box_xy(
                point_m=(0.35, -0.45, 0.39),
                center_m=(0.35, -0.45, 0.385),
                size_m=(0.12, 0.18, 0.06),
                margin_m=0.005,
            )
        )
        self.assertFalse(
            is_point_in_box_xy(
                point_m=(0.42, -0.45, 0.39),
                center_m=(0.35, -0.45, 0.385),
                size_m=(0.12, 0.18, 0.06),
                margin_m=0.005,
            )
        )


if __name__ == "__main__":
    unittest.main()
