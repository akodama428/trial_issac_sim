from __future__ import annotations

import unittest

from tomato_harvest_sim.robot.motion_planner.safe_online_solver import (
    SafetyObservation, SafeSolverPolicy, solve_safe_reconnection,
)


NAMES = tuple(f"panda_joint{i}" for i in range(1, 8))
START = (0.0, -0.4, 0.0, -2.1, 0.0, 1.7, 0.8)
TARGET = (0.2, -0.5, 0.1, -1.9, 0.1, 1.6, 0.7)


class SafeOnlineSolverTest(unittest.TestCase):
    def test_preserves_endpoints_and_stops_at_both_ends(self) -> None:
        result = solve_safe_reconnection(joint_names=NAMES, start_positions_rad=START, target_positions_rad=TARGET)
        self.assertEqual(result.reason, "ok")
        assert result.trajectory is not None
        self.assertEqual(result.trajectory.points[0].positions_rad, START)
        self.assertEqual(result.trajectory.points[-1].positions_rad, TARGET)
        self.assertTrue(all(v == 0.0 for v in result.trajectory.points[0].velocities_rad_s or ()))
        self.assertTrue(all(v == 0.0 for v in result.trajectory.points[-1].velocities_rad_s or ()))

    def test_collision_guard_stops_and_near_collision_slows(self) -> None:
        stopped = solve_safe_reconnection(joint_names=NAMES, start_positions_rad=START, target_positions_rad=TARGET, observation=SafetyObservation(collision_clearance_m=0.01))
        slowed = solve_safe_reconnection(joint_names=NAMES, start_positions_rad=START, target_positions_rad=TARGET, observation=SafetyObservation(collision_clearance_m=0.05))
        clear = solve_safe_reconnection(joint_names=NAMES, start_positions_rad=START, target_positions_rad=TARGET, observation=SafetyObservation(collision_clearance_m=0.20))
        self.assertEqual(stopped.reason, "collision_proximity_stop")
        self.assertIsNone(stopped.trajectory)
        self.assertLess(slowed.speed_scale, clear.speed_scale)
        assert slowed.trajectory and clear.trajectory
        self.assertGreater(slowed.trajectory.points[-1].time_from_start_sec, clear.trajectory.points[-1].time_from_start_sec)

    def test_singularity_guard_stops_and_slows(self) -> None:
        stopped = solve_safe_reconnection(joint_names=NAMES, start_positions_rad=START, target_positions_rad=TARGET, observation=SafetyObservation(singularity_measure=0.04))
        slowed = solve_safe_reconnection(joint_names=NAMES, start_positions_rad=START, target_positions_rad=TARGET, observation=SafetyObservation(singularity_measure=0.10))
        self.assertEqual(stopped.reason, "singularity_stop")
        self.assertIsNone(stopped.trajectory)
        self.assertLess(slowed.speed_scale, 1.0)

    def test_joint_position_guard_rejects_margin_violation(self) -> None:
        unsafe = (*START[:3], -0.08, *START[4:])
        result = solve_safe_reconnection(joint_names=NAMES, start_positions_rad=unsafe, target_positions_rad=TARGET)
        self.assertEqual(result.reason, "joint_position_limit:panda_joint4")
        self.assertIsNone(result.trajectory)

    def test_sampled_velocity_and_acceleration_stay_below_policy(self) -> None:
        policy = SafeSolverPolicy(nominal_velocity_rad_s=0.5, nominal_acceleration_rad_s2=1.0, segments=60)
        result = solve_safe_reconnection(joint_names=NAMES, start_positions_rad=START, target_positions_rad=TARGET, policy=policy)
        assert result.trajectory
        points = result.trajectory.points
        self.assertLessEqual(max(abs(v) for p in points for v in (p.velocities_rad_s or ())), 0.5 + 1e-9)
        accelerations = []
        for left, right in zip(points, points[1:]):
            dt = right.time_from_start_sec - left.time_from_start_sec
            accelerations.extend(abs((rv - lv) / dt) for lv, rv in zip(left.velocities_rad_s or (), right.velocities_rad_s or ()))
        self.assertLessEqual(max(accelerations), 1.0 + 1e-9)


if __name__ == "__main__":
    unittest.main()
