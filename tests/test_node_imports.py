"""5 ノードファイルが正しく import できることを確認するテスト（ROS2 非依存）。"""
from __future__ import annotations

import unittest


class TestNodeModulesImportable(unittest.TestCase):
    def test_tomato_detector_node_importable(self) -> None:
        import tomato_harvest_sim.robot.tomato_detector_node as m
        self.assertTrue(callable(m.main))

    def test_behavior_planner_node_importable(self) -> None:
        import tomato_harvest_sim.robot.behavior_planner_node as m
        self.assertTrue(callable(m.main))

    def test_trajectory_planner_node_importable(self) -> None:
        import tomato_harvest_sim.robot.trajectory_planner_node as m
        self.assertTrue(callable(m.main))

    def test_trajectory_monitor_node_importable(self) -> None:
        import tomato_harvest_sim.robot.trajectory_monitor_node as m
        self.assertTrue(callable(m.main))

    def test_motion_command_node_importable(self) -> None:
        import tomato_harvest_sim.robot.motion_command_node as m
        self.assertTrue(callable(m.main))
        self.assertTrue(callable(m.build_motion_command))


if __name__ == "__main__":
    unittest.main()
