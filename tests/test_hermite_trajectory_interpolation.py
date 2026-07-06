from __future__ import annotations

import unittest

import numpy as np

from tomato_harvest_sim.msg.contracts import JointTrajectory, JointTrajectoryPoint
from tomato_harvest_sim.robot.msg.trajectory_tracking import TrajectorySegment
from tomato_harvest_sim.robot.trajectory_tracking.reference_tracking import (
    build_joint_trajectory_segments,
    sample_trajectory_reference_state,
)

VEL_LIMITS = np.ones(8) * 2.0


def _make_segment(
    *,
    start: list[float],
    target: list[float],
    duration: float = 1.0,
    start_vel: list[float] | None = None,
    target_vel: list[float] | None = None,
    elapsed: float = 0.0,
) -> TrajectorySegment:
    seg = TrajectorySegment(
        start_positions=np.array(start + [0.0]),
        target_positions=np.array(target + [0.0]),
        duration_sec=duration,
        start_time_sec=0.0,
        start_velocities=np.array(start_vel + [0.0]) if start_vel is not None else None,
        target_velocities=np.array(target_vel + [0.0]) if target_vel is not None else None,
    )
    return seg


class HermiteInterpolationTest(unittest.TestCase):
    def _sample(self, seg: TrajectorySegment, *, now_sec: float) -> tuple[np.ndarray, np.ndarray]:
        ref, cmd = sample_trajectory_reference_state(
            active_segment=seg,
            current_positions=np.zeros(8),
            now_sec=now_sec,
            time_epsilon_sec=1e-6,
            arm_joint_velocity_limits_rad_s=VEL_LIMITS,
        )
        return ref.reference_positions[:7], ref.reference_velocities[:7]

    # ---- cubic Hermite property checks ----

    def test_hermite_position_at_start(self):
        """alpha=0 で開始位置が返る"""
        seg = _make_segment(start=[0.0] * 7, target=[1.0] * 7, start_vel=[0.5] * 7, target_vel=[0.0] * 7)
        pos, _ = self._sample(seg, now_sec=0.0)
        np.testing.assert_allclose(pos, np.zeros(7), atol=1e-9)

    def test_hermite_position_at_end(self):
        """alpha=1 で目標位置が返る"""
        seg = _make_segment(start=[0.0] * 7, target=[1.0] * 7, start_vel=[0.5] * 7, target_vel=[0.0] * 7)
        pos, _ = self._sample(seg, now_sec=1.0)
        np.testing.assert_allclose(pos, np.ones(7), atol=1e-9)

    def test_hermite_velocity_at_start(self):
        """alpha=0 で MoveIt が提供した開始速度が返る"""
        v0 = [0.3, -0.2, 0.1, 0.0, 0.0, 0.0, 0.0]
        seg = _make_segment(start=[0.0] * 7, target=[1.0] * 7, start_vel=v0, target_vel=[0.0] * 7)
        _, vel = self._sample(seg, now_sec=0.0)
        np.testing.assert_allclose(vel, np.array(v0), atol=1e-6)

    def test_hermite_velocity_at_end(self):
        """alpha=1 で MoveIt が提供した終端速度（通常は 0）が返る"""
        seg = _make_segment(start=[0.0] * 7, target=[1.0] * 7, start_vel=[0.5] * 7, target_vel=[0.0] * 7)
        _, vel = self._sample(seg, now_sec=1.0)
        np.testing.assert_allclose(vel, np.zeros(7), atol=1e-6)

    def test_hermite_no_velocity_jump_near_end(self):
        """終端直前（alpha≈0.99）と終端（alpha=1.0）で速度が連続する（急変しない）"""
        v1 = [0.0] * 7
        seg = _make_segment(start=[0.0] * 7, target=[1.0] * 7, start_vel=[0.5] * 7, target_vel=v1, duration=2.0)
        _, vel_before = self._sample(seg, now_sec=1.98)
        _, vel_at_end = self._sample(seg, now_sec=2.0)
        diff = np.max(np.abs(vel_at_end - vel_before))
        self.assertLess(diff, 0.15, "終端付近で速度が急変している")

    # ---- linear fallback (velocities=None) ----

    def test_linear_fallback_when_no_velocities(self):
        """velocities が None のときは従来の線形補間にフォールバックする"""
        seg = _make_segment(start=[0.0] * 7, target=[2.0] * 7)  # no velocities
        pos_mid, vel_mid = self._sample(seg, now_sec=0.5)
        np.testing.assert_allclose(pos_mid, np.ones(7), atol=1e-9)
        np.testing.assert_allclose(vel_mid, np.full(7, 2.0), atol=1e-6)  # (2-0)/1.0

    def test_linear_fallback_velocity_zero_at_end(self):
        """velocities=None で alpha=1 のとき速度は 0 になる"""
        seg = _make_segment(start=[0.0] * 7, target=[1.0] * 7)
        _, vel = self._sample(seg, now_sec=1.0)
        np.testing.assert_allclose(vel, np.zeros(7), atol=1e-9)

    # ---- build_joint_trajectory_segments velocity propagation ----

    def test_segment_velocities_populated_from_trajectory_points(self):
        """build_joint_trajectory_segments が trajectory の velocities を各セグメントへ伝播する"""
        v0 = tuple([0.1] * 7)
        v1 = tuple([0.0] * 7)
        traj = JointTrajectory(
            joint_names=tuple(f"j{i}" for i in range(7)),
            points=(
                JointTrajectoryPoint(positions_rad=tuple([0.0] * 7), time_from_start_sec=0.0, velocities_rad_s=v0),
                JointTrajectoryPoint(positions_rad=tuple([1.0] * 7), time_from_start_sec=1.0, velocities_rad_s=v1),
            ),
        )
        current = np.zeros(8)
        expanded = (np.array([0.0] * 8), np.array([1.0] * 8))
        segments, _ = build_joint_trajectory_segments(
            trajectory=traj,
            expanded_targets=expanded,
            current_positions=current,
            joint_tolerance_rad=1e-3,
            time_epsilon_sec=1e-6,
            arm_joint_velocity_limits_rad_s=VEL_LIMITS,
        )
        # segment 0: start=current(zeros)→point[0]; target_vel should be v0
        self.assertIsNotNone(segments[0].target_velocities)
        np.testing.assert_allclose(segments[0].target_velocities[:7], np.array(v0), atol=1e-9)
        # segment 1: point[0]→point[1]; start_vel=v0, target_vel=v1
        self.assertIsNotNone(segments[1].start_velocities)
        self.assertIsNotNone(segments[1].target_velocities)
        np.testing.assert_allclose(segments[1].start_velocities[:7], np.array(v0), atol=1e-9)
        np.testing.assert_allclose(segments[1].target_velocities[:7], np.array(v1), atol=1e-9)

    def test_segment_velocities_none_when_trajectory_points_lack_velocities(self):
        """trajectory に velocities がない場合、セグメントの velocities は None になる"""
        traj = JointTrajectory(
            joint_names=tuple(f"j{i}" for i in range(7)),
            points=(
                JointTrajectoryPoint(positions_rad=tuple([0.0] * 7), time_from_start_sec=0.0),
                JointTrajectoryPoint(positions_rad=tuple([1.0] * 7), time_from_start_sec=1.0),
            ),
        )
        current = np.zeros(8)
        expanded = (np.array([0.0] * 8), np.array([1.0] * 8))
        segments, _ = build_joint_trajectory_segments(
            trajectory=traj,
            expanded_targets=expanded,
            current_positions=current,
            joint_tolerance_rad=1e-3,
            time_epsilon_sec=1e-6,
            arm_joint_velocity_limits_rad_s=VEL_LIMITS,
        )
        for seg in segments:
            self.assertIsNone(seg.target_velocities)


if __name__ == "__main__":
    unittest.main()
