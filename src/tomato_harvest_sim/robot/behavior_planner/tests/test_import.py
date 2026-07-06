"""behavior_planner パッケージが ROS2 なしで import できることを確認する。"""
from __future__ import annotations

import unittest


class TestImportable(unittest.TestCase):
    def test_package_importable(self) -> None:
        import tomato_harvest_sim.robot.behavior_planner as m
        self.assertTrue(callable(m.main))
        self.assertTrue(callable(m.detaching_outcome))
        self.assertTrue(callable(m.moving_to_place_outcome))


if __name__ == "__main__":
    unittest.main()
