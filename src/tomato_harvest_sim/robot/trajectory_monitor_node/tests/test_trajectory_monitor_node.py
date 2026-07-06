"""trajectory_monitor_node の execution_status → trajectory_status 変換ロジックのテスト。"""
from __future__ import annotations

import unittest

from tomato_harvest_sim.robot.trajectory_monitor_node import (
    trajectory_status_from_execution_status,
)


class TestTrajectoryMonitorLogic(unittest.TestCase):
    def test_running_maps_to_ok(self) -> None:
        self.assertEqual(trajectory_status_from_execution_status("running"), "ok")

    def test_succeeded_maps_to_ok(self) -> None:
        self.assertEqual(trajectory_status_from_execution_status("succeeded"), "ok")

    def test_aborted_maps_to_aborted(self) -> None:
        self.assertEqual(trajectory_status_from_execution_status("aborted"), "aborted")


if __name__ == "__main__":
    unittest.main()
