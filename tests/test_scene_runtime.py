from __future__ import annotations

import unittest

from tomato_harvest_sim.api.contracts import Pose3D
from tomato_harvest_sim.simulator.scene_config import load_scene_layout_config
from tomato_harvest_sim.simulator.scene_runtime import IsaacSceneRuntime


class SceneRuntimeLayoutTest(unittest.TestCase):
    def test_boot_creates_expected_scene_layout(self) -> None:
        runtime = IsaacSceneRuntime()
        layout = load_scene_layout_config()

        snapshot = runtime.boot()

        self.assertEqual(snapshot.robot_model, "Franka Panda")
        self.assertEqual(snapshot.active_camera, "fixed_camera")
        self.assertTrue(snapshot.tomato_attached)
        self.assertEqual(snapshot.fixed_camera_pose, layout.fixed_camera_pose)
        self.assertEqual(snapshot.hand_camera_pose, layout.hand_camera_pose)
        self.assertEqual(snapshot.branch_pose, layout.branch_pose)
        self.assertEqual(snapshot.stem_pose, layout.stem_pose)
        self.assertEqual(snapshot.tomato_pose, layout.tomato_pose)
        self.assertEqual(snapshot.tray_pose, layout.tray_pose)

    def test_camera_switch_toggles_between_fixed_and_hand_camera(self) -> None:
        runtime = IsaacSceneRuntime()
        runtime.boot()

        hand_snapshot = runtime.set_active_camera("hand_camera")
        fixed_snapshot = runtime.set_active_camera("fixed_camera")

        self.assertEqual(hand_snapshot.active_camera, "hand_camera")
        self.assertEqual(fixed_snapshot.active_camera, "fixed_camera")

    def test_reset_restores_scene_deterministically(self) -> None:
        runtime = IsaacSceneRuntime()
        layout = load_scene_layout_config()
        runtime.boot()

        runtime.set_active_camera("hand_camera")
        runtime.move_robot_home(False)
        runtime.set_tomato_pose(Pose3D(0.70, 0.10, 0.40, 0.0, 0.0, 0.0))
        runtime.detach_tomato()

        snapshot = runtime.reset_scene()

        self.assertEqual(snapshot.active_camera, "fixed_camera")
        self.assertTrue(snapshot.robot_home)
        self.assertTrue(snapshot.tomato_attached)
        self.assertEqual(snapshot.stem_pose, layout.stem_pose)
        self.assertEqual(snapshot.tomato_pose, layout.tomato_pose)

    def test_close_gripper_clears_active_motion_target(self) -> None:
        runtime = IsaacSceneRuntime(physics_grasp_enabled=True)
        runtime.boot()
        target_pose = Pose3D(0.62, 0.0, 0.585, 180.0, 0.0, 0.0)

        runtime.set_grasp_pose(target_pose, waypoint_poses=(target_pose,))
        snapshot = runtime.close_gripper()

        self.assertIsNone(snapshot.target_tool_pose)
        self.assertEqual(snapshot.motion_waypoints, ())
        self.assertIsNone(snapshot.active_waypoint_index)
        self.assertIsNone(snapshot.motion_joint_trajectory)

    def test_simulator_runtime_runs_without_robot_runtime(self) -> None:
        runtime = IsaacSceneRuntime()

        runtime.boot()
        summary = runtime.describe_scene()

        self.assertIn("Franka Panda", summary)
        self.assertIn("fixed_camera", summary)
        self.assertIn("tomato_attached=True", summary)


if __name__ == "__main__":
    unittest.main()
