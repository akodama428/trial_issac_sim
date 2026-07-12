from __future__ import annotations

import unittest

from tomato_harvest_sim.simulator.initial_pose_cases import (
    INITIAL_POSE_CASES, initial_pose_from_environment, validate_cases,
)


class InitialPoseCasesTest(unittest.TestCase):
    def test_ten_fixed_cases_include_default_and_singularity(self) -> None:
        self.assertEqual(len(INITIAL_POSE_CASES), 10)
        self.assertEqual(len({case.case_id for case in INITIAL_POSE_CASES}), 10)
        self.assertTrue(any(case.case_id == "default" for case in INITIAL_POSE_CASES))
        singular = [case for case in INITIAL_POSE_CASES if case.is_singularity_case]
        self.assertEqual(len(singular), 1)
        self.assertEqual(singular[0].case_id, "near_singularity_extended")

    def test_all_cases_have_seven_finite_joints_inside_panda_limits(self) -> None:
        self.assertEqual(validate_cases(INITIAL_POSE_CASES), ())

    def test_environment_selects_fixed_case_and_rejects_unknown_id(self) -> None:
        self.assertEqual(
            initial_pose_from_environment("elbow_left"),
            next(c.positions_rad for c in INITIAL_POSE_CASES if c.case_id == "elbow_left"),
        )
        with self.assertRaises(ValueError):
            initial_pose_from_environment("unknown")


if __name__ == "__main__":
    unittest.main()
