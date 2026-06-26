from __future__ import annotations

import unittest

import numpy as np

from tomato_harvest_sim.robot.trajectory_tracking import joint_positions_reached, step_toward_joint_positions


class FrankaMotionHelperTest(unittest.TestCase):
    def test_step_toward_joint_positions_limits_max_delta_per_joint(self) -> None:
        current = np.array([0.0, -0.4, 0.2, 0.0], dtype=float)
        target = np.array([0.5, -1.0, -0.4, 0.1], dtype=float)

        next_step = step_toward_joint_positions(current, target, max_step_rad=0.1)

        self.assertTrue(np.allclose(next_step, np.array([0.1, -0.5, 0.1, 0.1], dtype=float)))

    def test_step_toward_joint_positions_snaps_when_within_limit(self) -> None:
        current = np.array([0.0, 0.05], dtype=float)
        target = np.array([0.03, -0.04], dtype=float)

        next_step = step_toward_joint_positions(current, target, max_step_rad=0.1)

        self.assertTrue(np.allclose(next_step, target))

    def test_joint_positions_reached_uses_max_abs_error(self) -> None:
        current = np.array([0.0, 0.05, -0.02], dtype=float)
        target = np.array([0.02, 0.07, -0.01], dtype=float)

        self.assertTrue(joint_positions_reached(current, target, tolerance_rad=0.03))
        self.assertFalse(joint_positions_reached(current, target, tolerance_rad=0.015))


if __name__ == "__main__":
    unittest.main()
