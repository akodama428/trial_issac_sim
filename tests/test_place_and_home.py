from __future__ import annotations

import unittest

from tomato_harvest_sim.api.contracts import ControlCommand, HarvestTaskPhase, MotionCommand, Pose3D, TomatoStatus
from tomato_harvest_sim.app.application import create_tomato_harvest_application
from tomato_harvest_sim.simulator.scene_config import load_scene_layout_config
from tomato_harvest_sim.simulator.scene_runtime import IsaacSceneRuntime


class PlaceAndHomeTest(unittest.TestCase):
    def test_detached_tomato_can_be_placed_and_robot_returns_home(self) -> None:
        layout = load_scene_layout_config()
        system = create_tomato_harvest_application(grasp_mode="success")
        system.boot()
        system.apply_control(ControlCommand.START)

        logs: list[str] = []
        for _ in range(64):
            logs.extend(system.step())
            if system.robot.state.task_phase is HarvestTaskPhase.COMPLETE:
                break

        self.assertEqual(system.robot.state.task_phase, HarvestTaskPhase.COMPLETE)
        self.assertEqual(system.simulator.state.tomato_status, TomatoStatus.PLACED)
        self.assertTrue(system.simulator.state.robot_home)
        self.assertEqual(system.simulator.state.target_tool_pose, layout.home_tool_pose)
        self.assertTrue(any("Tomato placed in the tray" in line for line in logs))
        self.assertTrue(any("returned home" in line for line in logs))

    def test_open_gripper_at_place_marks_tomato_as_placed(self) -> None:
        layout = load_scene_layout_config()
        runtime = IsaacSceneRuntime()
        runtime.boot()

        runtime.apply_motion_command(
            MotionCommand(
                command_name="move_to_grasp",
                planner_name="moveit2_grasp_demo",
                target_pose=Pose3D(
                    layout.tomato_pose.x,
                    layout.tomato_pose.y,
                    layout.tomato_pose.z + 0.045,
                    180.0,
                    0.0,
                    0.0,
                ),
            )
        )
        for _ in range(16):
            runtime.advance()
        runtime.apply_motion_command(
            MotionCommand(
                command_name="close_gripper",
                planner_name="moveit2_grasp_demo",
            )
        )
        runtime.apply_motion_command(
            MotionCommand(
                command_name="pull_to_detach",
                planner_name="moveit2_grasp_demo",
                target_pose=Pose3D(0.34, 0.00, 0.62, 180.0, 0.0, 0.0),
            )
        )
        for _ in range(8):
            runtime.advance()
        runtime.apply_motion_command(
            MotionCommand(
                command_name="move_to_place",
                planner_name="moveit2_grasp_demo",
                target_pose=Pose3D(0.35, -0.35, 0.57, 180.0, 0.0, 0.0),
            )
        )
        for _ in range(12):
            runtime.advance()
        runtime.apply_motion_command(
            MotionCommand(
                command_name="open_gripper",
                planner_name="moveit2_grasp_demo",
            )
        )

        snapshot = runtime.snapshot()
        self.assertEqual(snapshot.tomato_status, TomatoStatus.PLACED)
        self.assertEqual(snapshot.tomato_pose, Pose3D(0.35, -0.35, 0.48, 0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
