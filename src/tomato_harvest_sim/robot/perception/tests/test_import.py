"""perception パッケージが ROS2 なしで import できることを確認する。"""
from __future__ import annotations

import unittest


class TestImportable(unittest.TestCase):
    def test_package_importable(self) -> None:
        import tomato_harvest_sim.robot.perception as m
        self.assertTrue(callable(m.main))
        self.assertTrue(callable(m.TomatoTargetEstimator))


if __name__ == "__main__":
    unittest.main()
