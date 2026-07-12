"""planning失敗診断の保存 (Issue #28 改善1) のテスト。"""
from __future__ import annotations

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tomato_harvest_sim.robot.motion_planner.planning_diagnostics import (
    PLANNING_DIAGNOSTIC_DIR_ENV,
    PlanningFailureDiagnostic,
    StateValidityReport,
    diagnostic_to_dict,
    diagnostics_directory,
    save_planning_failure_diagnostic,
)


def _diagnostic(*, phase: str = "moving_to_place") -> PlanningFailureDiagnostic:
    return PlanningFailureDiagnostic(
        captured_at_sec=1720000000.123456,
        phase=phase,
        goal_kind="pose",
        reason="motion_plan_error",
        error_code=99999,
        target_xyz_m=(0.35, -0.35, 0.50),
        start_joint_names=("panda_joint1", "panda_joint2"),
        start_positions_rad=(0.1, -0.4),
        start_state=StateValidityReport(
            checked=True,
            valid=False,
            contacts=("attached_tomato|place_tray",),
        ),
    )


class DiagnosticsDirectoryTest(unittest.TestCase):
    def test_diagnostics_are_disabled_when_env_is_unset(self) -> None:
        self.assertIsNone(diagnostics_directory({}))

    def test_diagnostics_are_disabled_when_env_is_blank(self) -> None:
        self.assertIsNone(diagnostics_directory({PLANNING_DIAGNOSTIC_DIR_ENV: "  "}))

    def test_env_value_selects_diagnostics_directory(self) -> None:
        directory = diagnostics_directory(
            {PLANNING_DIAGNOSTIC_DIR_ENV: "/tmp/planning-diag"}
        )
        self.assertEqual(directory, Path("/tmp/planning-diag"))


class DiagnosticSerializationTest(unittest.TestCase):
    def test_diagnostic_dict_keeps_failure_evidence_fields(self) -> None:
        payload = diagnostic_to_dict(_diagnostic())

        self.assertEqual(payload["phase"], "moving_to_place")
        self.assertEqual(payload["goal_kind"], "pose")
        self.assertEqual(payload["error_code"], 99999)
        self.assertEqual(payload["target_xyz_m"], [0.35, -0.35, 0.50])
        self.assertEqual(payload["start_joint_names"], ["panda_joint1", "panda_joint2"])
        self.assertEqual(payload["start_positions_rad"], [0.1, -0.4])
        self.assertEqual(payload["start_state"], {
            "checked": True,
            "valid": False,
            "contacts": ["attached_tomato|place_tray"],
        })

    def test_unchecked_start_state_is_recorded_as_unknown(self) -> None:
        diagnostic = PlanningFailureDiagnostic(
            captured_at_sec=1.0,
            phase="moving_to_pregrasp",
            goal_kind="joint",
            reason="service_timeout",
            error_code=None,
            target_xyz_m=None,
            start_joint_names=("panda_joint1",),
            start_positions_rad=(0.0,),
            start_state=StateValidityReport(checked=False),
        )

        payload = diagnostic_to_dict(diagnostic)

        self.assertIsNone(payload["error_code"])
        self.assertIsNone(payload["target_xyz_m"])
        self.assertEqual(payload["start_state"], {
            "checked": False,
            "valid": None,
            "contacts": [],
        })


class DiagnosticSaveTest(unittest.TestCase):
    def test_diagnostic_is_saved_as_readable_json(self) -> None:
        with TemporaryDirectory() as tmp:
            path = save_planning_failure_diagnostic(_diagnostic(), Path(tmp))

            self.assertIsNotNone(path)
            assert path is not None
            loaded = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(loaded, diagnostic_to_dict(_diagnostic()))

    def test_file_name_identifies_phase_and_goal_kind(self) -> None:
        with TemporaryDirectory() as tmp:
            path = save_planning_failure_diagnostic(_diagnostic(), Path(tmp))

            assert path is not None
            self.assertIn("moving_to_place", path.name)
            self.assertIn("pose", path.name)
            self.assertTrue(path.name.endswith(".json"))

    def test_missing_directory_is_created(self) -> None:
        with TemporaryDirectory() as tmp:
            nested = Path(tmp) / "nested" / "diag"

            path = save_planning_failure_diagnostic(_diagnostic(), nested)

            self.assertIsNotNone(path)
            assert path is not None
            self.assertTrue(path.exists())

    def test_save_is_disabled_when_directory_is_none(self) -> None:
        self.assertIsNone(save_planning_failure_diagnostic(_diagnostic(), None))

    def test_repeated_saves_do_not_overwrite_prior_diagnostics(self) -> None:
        with TemporaryDirectory() as tmp:
            first = save_planning_failure_diagnostic(_diagnostic(), Path(tmp))
            second = save_planning_failure_diagnostic(_diagnostic(), Path(tmp))

            assert first is not None and second is not None
            self.assertNotEqual(first, second)
            self.assertEqual(len(list(Path(tmp).glob("*.json"))), 2)


if __name__ == "__main__":
    unittest.main()
