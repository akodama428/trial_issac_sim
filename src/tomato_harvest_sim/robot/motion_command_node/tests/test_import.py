"""motion_command_node パッケージが ROS2 なしで import できることを確認する。"""
from __future__ import annotations

import unittest


class TestImportable(unittest.TestCase):
    def test_package_importable(self) -> None:
        import tomato_harvest_sim.robot.motion_command_node as m
        self.assertTrue(callable(m.main))
        self.assertTrue(callable(m.build_motion_command))


if __name__ == "__main__":
    unittest.main()
