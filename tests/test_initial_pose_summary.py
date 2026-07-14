from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from scripts.ci.summarize_initial_pose_e2e import summarize


class InitialPoseSummaryTest(unittest.TestCase):
    def test_counts_live_tracking_samples(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            case_root = root / "default" / "e2e"
            case_root.mkdir(parents=True)
            (case_root / "robot_node.log").write_text(
                'Phase: returning_home -> complete\n'
            )
            (case_root / "franka_controller.log").write_text(
                'execution_status {"status":"running",'
                '"tracking_error_rad":0.125,"limiting_joint":"panda_joint4"}\n'
                'execution_status {"status":"running",'
                '"tracking_error_rad":0.025,"limiting_joint":"panda_joint2"}\n'
            )

            result = summarize(root, ["default"], "abc")

        case = result["cases"][0]
        self.assertEqual(case["live_tracking_sample_count"], 2)
        self.assertAlmostEqual(case["maximum_live_tracking_error_rad"], 0.125)

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

    def test_stack_startup_failure_is_classified_separately(self) -> None:
        """起動flake (計画系と無関係) を通常の実行失敗と区別する (Issue #40)。"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "default" / "e2e").mkdir(parents=True)
            (root / "default" / "e2e" / "robot_node.log").write_text("")
            (root / "default" / "e2e" / "docker-e2e-console.log").write_text(
                "STACK_STARTUP_FAILED: controller_manager\n"
            )
            result = summarize(root, ["default"], "abc")
        self.assertFalse(result["cases"][0]["success"])
        self.assertEqual(result["cases"][0]["failure_reason"], "stack_startup_failed")
