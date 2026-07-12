"""execute_manager の execution_status → trajectory_status 変換ロジックのテスト。"""
from __future__ import annotations

import json
import unittest

from tomato_harvest_sim.robot.execute_manager import (
    trajectory_status_from_execution_status,
)
from tomato_harvest_sim.robot.execute_manager.trajectory_monitor import (
    trajectory_status_payload,
)


class TestTrajectoryMonitorLogic(unittest.TestCase):
    def test_running_maps_to_ok(self) -> None:
        self.assertEqual(trajectory_status_from_execution_status("running"), "ok")

    def test_succeeded_maps_to_ok(self) -> None:
        self.assertEqual(trajectory_status_from_execution_status("succeeded"), "ok")

    def test_aborted_maps_to_aborted(self) -> None:
        self.assertEqual(trajectory_status_from_execution_status("aborted"), "aborted")


class TestTrajectoryStatusPayload(unittest.TestCase):
    """abort診断をexecutorからplannerまで通すJSON payload変換 (Issue #32)。"""

    def test_plain_running_becomes_ok_json(self) -> None:
        payload = json.loads(trajectory_status_payload("running"))
        self.assertEqual(payload, {"status": "ok"})

    def test_plain_aborted_becomes_aborted_json(self) -> None:
        payload = json.loads(trajectory_status_payload("aborted"))
        self.assertEqual(payload, {"status": "aborted"})

    def test_abort_diagnostics_fields_are_forwarded(self) -> None:
        raw = json.dumps({
            "status": "aborted",
            "max_joint_error_rad": 0.184,
            "limiting_joint": "panda_joint4",
            "abort_reason": "goal_tolerance_violated",
        })

        payload = json.loads(trajectory_status_payload(raw))

        self.assertEqual(payload["status"], "aborted")
        self.assertAlmostEqual(payload["max_joint_error_rad"], 0.184)
        self.assertEqual(payload["limiting_joint"], "panda_joint4")
        self.assertEqual(payload["abort_reason"], "goal_tolerance_violated")

    def test_json_succeeded_maps_to_ok_without_diagnostics(self) -> None:
        raw = json.dumps({"status": "succeeded"})
        payload = json.loads(trajectory_status_payload(raw))
        self.assertEqual(payload, {"status": "ok"})

    def test_unknown_fields_are_not_forwarded(self) -> None:
        """診断以外の未知fieldは下流契約へ流さない。"""
        raw = json.dumps({"status": "aborted", "unexpected": "x"})
        payload = json.loads(trajectory_status_payload(raw))
        self.assertNotIn("unexpected", payload)


if __name__ == "__main__":
    unittest.main()
