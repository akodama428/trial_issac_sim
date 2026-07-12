from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.ci.summarize_initial_pose_e2e import summarize


class InitialPoseSummaryTest(unittest.TestCase):
    def test_failed_case_does_not_hide_later_success(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "default" / "e2e").mkdir(parents=True)
            (root / "default" / "e2e" / "robot_node.log").write_text("Phase: moving_to_grasp -> failed\n")
            (root / "elbow_left" / "e2e").mkdir(parents=True)
            (root / "elbow_left" / "e2e" / "robot_node.log").write_text("Phase: returning_home -> complete\n")
            result = summarize(root, ["default", "elbow_left"], "abc")
        self.assertEqual(result["case_count"], 2)
        self.assertEqual(result["success_count"], 1)
        self.assertEqual(result["success_rate"], 0.5)
        self.assertEqual(len(result["cases"][0]["initial_positions_rad"]), 7)
