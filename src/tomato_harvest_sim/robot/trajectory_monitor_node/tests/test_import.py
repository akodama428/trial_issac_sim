"""trajectory_monitor_node パッケージが ROS2 なしで import できることを確認する。"""
from __future__ import annotations

import unittest


class TestImportable(unittest.TestCase):
    def test_package_importable(self) -> None:
        import tomato_harvest_sim.robot.trajectory_monitor_node as m
        self.assertTrue(callable(m.main))
        self.assertTrue(callable(m.trajectory_status_from_execution_status))


if __name__ == "__main__":
    unittest.main()
