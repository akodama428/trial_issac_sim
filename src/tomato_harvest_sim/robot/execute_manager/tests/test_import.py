"""execute_manager パッケージが ROS2 なしで import できることを確認する。"""
from __future__ import annotations

import unittest


class TestImportable(unittest.TestCase):
    def test_package_importable(self) -> None:
        import tomato_harvest_sim.robot.execute_manager as m
        self.assertTrue(callable(m.main_motion_command))
        self.assertTrue(callable(m.main_trajectory_monitor))
        self.assertTrue(callable(m.build_motion_command))
        self.assertTrue(callable(m.trajectory_status_from_execution_status))


if __name__ == "__main__":
    unittest.main()
