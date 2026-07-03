from __future__ import annotations

import unittest

from tomato_harvest_sim.api.contracts import Pose3D, TomatoStatus
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
        # set_robot_tool_pose で非ホーム位置を設定
        runtime.set_robot_tool_pose(Pose3D(0.1, 0.0, 0.3, 0.0, 0.0, 0.0))
        runtime.set_tomato_pose(Pose3D(0.70, 0.10, 0.40, 0.0, 0.0, 0.0))
        runtime.detach_tomato()

        snapshot = runtime.reset_scene()

        self.assertEqual(snapshot.active_camera, "fixed_camera")
        self.assertTrue(snapshot.robot_home)
        self.assertTrue(snapshot.tomato_attached)
        self.assertEqual(snapshot.stem_pose, layout.stem_pose)
        self.assertEqual(snapshot.tomato_pose, layout.tomato_pose)

    def test_simulator_runtime_runs_without_robot_runtime(self) -> None:
        runtime = IsaacSceneRuntime()

        runtime.boot()
        summary = runtime.describe_scene()

        self.assertIn("Franka Panda", summary)
        self.assertIn("fixed_camera", summary)
        self.assertIn("tomato_attached=True", summary)

    def test_apply_finger_positions_closed_triggers_stable_grasp(self) -> None:
        """finger[7-8]が閉じたとき、ツールがトマト近傍なら把持成功になる"""
        layout = load_scene_layout_config()
        runtime = IsaacSceneRuntime()
        runtime.boot()
        # ツールをトマトの直上に配置（把持可能な距離）
        grasp_pose = Pose3D(
            layout.tomato_pose.x,
            layout.tomato_pose.y,
            layout.tomato_pose.z + 0.045,  # GRASP_TOMATO_OFFSET_Z_M より少し小さい
            180.0, 0.0, 0.0,
        )
        runtime.set_robot_tool_pose(grasp_pose)
        # finger を閉じる
        finger_closed = 0.0
        snapshot = runtime.apply_finger_positions(finger_closed, finger_closed)
        self.assertTrue(snapshot.gripper_closed)
        self.assertEqual(snapshot.tomato_status, TomatoStatus.HELD)

    def test_apply_finger_positions_open_after_close_releases_gripper(self) -> None:
        """finger[7-8]が開いたとき、gripper_closed が False になる"""
        runtime = IsaacSceneRuntime()
        runtime.boot()
        # まず閉じる
        runtime.apply_finger_positions(0.0, 0.0)
        # 次に開く
        finger_open = 0.04
        snapshot = runtime.apply_finger_positions(finger_open, finger_open)
        self.assertFalse(snapshot.gripper_closed)


if __name__ == "__main__":
    unittest.main()
