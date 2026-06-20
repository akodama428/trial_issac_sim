from __future__ import annotations

import unittest


class FrankaSmokePlanTest(unittest.TestCase):
    def test_plan_uses_expected_isaac_command_and_prims(self) -> None:
        from tomato_harvest_poc.franka_smoke import build_franka_smoke_plan

        plan = build_franka_smoke_plan()

        self.assertEqual(
            plan.container_command,
            "/isaac-sim/python.sh scripts/isaac_franka_smoke.py",
        )
        self.assertEqual(plan.robot_prim_path, "/World/FrankaPanda")
        self.assertEqual(plan.debug_camera_prim_path, "/World/Camera_Debug")
        self.assertEqual(plan.hand_camera_prim_name, "HandCamera")
        self.assertEqual(plan.tomato_radius_m, 0.01)
        self.assertEqual(plan.tomato_highlight_radius_m, 0.013)
        self.assertEqual(plan.hand_camera_local_offset_m, (0.0, 0.0, 0.06))
        self.assertEqual(plan.hand_camera_local_rotation_deg, (0.0, 180.0, 0.0))
        self.assertEqual(plan.centering_preferred_depth_m, 0.12)
        self.assertGreater(plan.centering_interpolation_frames, 0)

    def test_motion_cycle_has_grasp_and_pull_states(self) -> None:
        from tomato_harvest_poc.franka_smoke import build_franka_smoke_plan

        plan = build_franka_smoke_plan()
        labels = [step.label for step in plan.motion_steps]

        self.assertEqual(labels, ["home_open", "pre_grasp_open", "grasp_closed", "pull_closed"])
        self.assertTrue(all(len(step.dof_positions) == 9 for step in plan.motion_steps))
        self.assertGreater(plan.frames_per_step, 0)

        pre_grasp = plan.motion_steps[1].dof_positions
        grasp = plan.motion_steps[2].dof_positions
        pull = plan.motion_steps[3].dof_positions
        self.assertGreater(pre_grasp[7], grasp[7])
        self.assertEqual(grasp[7], grasp[8])
        self.assertEqual(pull[7], grasp[7])

    def test_motion_step_index_advances_in_fixed_frame_windows(self) -> None:
        from tomato_harvest_poc.franka_smoke import select_motion_step

        self.assertEqual(select_motion_step(frame=0, frames_per_step=90, motion_step_count=4), 0)
        self.assertEqual(select_motion_step(frame=89, frames_per_step=90, motion_step_count=4), 0)
        self.assertEqual(select_motion_step(frame=90, frames_per_step=90, motion_step_count=4), 1)
        self.assertEqual(select_motion_step(frame=270, frames_per_step=90, motion_step_count=4), 3)
        self.assertEqual(select_motion_step(frame=360, frames_per_step=90, motion_step_count=4), 0)

    def test_interpolate_dof_positions_blends_between_keyframes(self) -> None:
        from tomato_harvest_poc.franka_smoke import interpolate_dof_positions

        start = (0.0, 1.0, 2.0)
        end = (2.0, 3.0, 4.0)

        self.assertEqual(interpolate_dof_positions(start, end, progress=0.0), start)
        self.assertEqual(interpolate_dof_positions(start, end, progress=1.0), end)
        self.assertEqual(interpolate_dof_positions(start, end, progress=0.5), (1.0, 2.0, 3.0))

    def test_default_light_specs_include_key_and_fill_lights(self) -> None:
        from tomato_harvest_poc.franka_smoke import build_default_light_specs

        specs = build_default_light_specs()

        self.assertEqual(len(specs), 3)
        self.assertEqual(specs[0].kind, "distant")
        self.assertEqual(specs[0].prim_path, "/World/KeyLight")
        self.assertEqual(specs[1].kind, "sphere")
        self.assertIsNotNone(specs[1].translate_m)
        self.assertGreater(specs[1].intensity, 0.0)

    def test_look_at_rotation_points_camera_toward_target(self) -> None:
        from tomato_harvest_poc.franka_smoke import compute_look_at_rotate_xyz_deg

        rotation = compute_look_at_rotate_xyz_deg(
            (0.8, 0.0, 1.35),
            (0.64, 0.0, 0.72),
        )

        self.assertAlmostEqual(rotation[0], 0.0, places=2)
        self.assertAlmostEqual(rotation[1], 14.25, places=2)
        self.assertAlmostEqual(rotation[2], 0.0, places=3)

    def test_camera_look_at_rows_store_translation_in_last_row(self) -> None:
        from tomato_harvest_poc.franka_smoke import build_camera_look_at_rows

        rows = build_camera_look_at_rows((1.0, -0.5, 1.2), (0.5, 0.0, 0.7))

        self.assertEqual(rows[3], (1.0, -0.5, 1.2, 1.0))
        self.assertAlmostEqual(rows[0][3], 0.0, places=6)
        self.assertAlmostEqual(rows[1][3], 0.0, places=6)
        self.assertAlmostEqual(rows[2][3], 0.0, places=6)

    def test_camera_center_error_uses_only_xy_offset(self) -> None:
        from tomato_harvest_poc.franka_smoke import compute_camera_center_error

        self.assertAlmostEqual(compute_camera_center_error((0.003, 0.004, -0.25)), 0.005, places=6)

    def test_centering_camera_position_preserves_view_ray_and_depth(self) -> None:
        from tomato_harvest_poc.franka_smoke import compute_centering_camera_position

        camera_position = compute_centering_camera_position(
            (0.20, 0.10, 0.30),
            (0.50, 0.10, 0.30),
            preferred_depth_m=0.12,
        )

        self.assertEqual(camera_position, (0.38, 0.10, 0.30))

    def test_tomato_centering_score_penalizes_points_behind_camera(self) -> None:
        from tomato_harvest_poc.franka_smoke import compute_tomato_centering_score

        visible = compute_tomato_centering_score((0.01, 0.01, -0.25))
        behind = compute_tomato_centering_score((0.01, 0.01, 0.10))

        self.assertLess(visible, behind)


if __name__ == "__main__":
    unittest.main()
