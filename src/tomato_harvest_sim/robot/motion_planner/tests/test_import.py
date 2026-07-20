"""motion_planner パッケージが ROS2 なしで import できることを確認する。"""
from __future__ import annotations

import unittest


class TestImportable(unittest.TestCase):
    def test_package_importable(self) -> None:
        import tomato_harvest_sim.robot.motion_planner as m
        self.assertTrue(callable(m.main))
        self.assertTrue(callable(m.build_planner))

    def test_moveit_bridge_modules_are_importable_without_ros(self) -> None:
        from tomato_harvest_sim.robot.motion_planner.moveit_bridge import (
            client,
            config,
            geometry,
            goal_planner,
            phase_planner,
            phase_policy,
            planning_scene,
            request_builder,
            trajectory,
        )

        self.assertTrue(callable(phase_policy.phase_planning_specs))
        self.assertTrue(hasattr(phase_planner, "Ros2MoveIt2PlannerBridge"))
        self.assertTrue(hasattr(goal_planner, "MoveItGoalPlanner"))
        self.assertTrue(hasattr(client, "Ros2MoveIt2Clients"))
        self.assertTrue(hasattr(config, "MoveItPlannerConfig"))
        self.assertTrue(callable(request_builder.build_pose_goal_request))
        self.assertTrue(callable(planning_scene.build_planning_scene_request))
        self.assertTrue(callable(trajectory.concatenate_trajectories))
        self.assertTrue(callable(geometry.moveit_link_target_pose_from_runtime_tool_pose))


if __name__ == "__main__":
    unittest.main()
