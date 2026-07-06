from __future__ import annotations

import numpy as np

from tomato_harvest_sim.msg.contracts import JointTrajectory, JointTrajectoryPoint
from tomato_harvest_sim.robot.msg.trajectory_tracking import TrajectoryReferenceState, TrajectorySegment


def step_toward_joint_positions(
    current_positions: np.ndarray,
    target_positions: np.ndarray,
    *,
    max_step_rad: float,
) -> np.ndarray:
    limited_delta = np.clip(target_positions - current_positions, -max_step_rad, max_step_rad)
    next_positions = current_positions + limited_delta
    close_mask = np.abs(target_positions - current_positions) <= max_step_rad
    next_positions[close_mask] = target_positions[close_mask]
    return next_positions


def joint_positions_reached(
    current_positions: np.ndarray,
    target_positions: np.ndarray,
    *,
    tolerance_rad: float,
) -> bool:
    return float(np.max(np.abs(target_positions - current_positions))) <= tolerance_rad


def build_joint_trajectory_segments(
    *,
    trajectory: JointTrajectory,
    expanded_targets: tuple[np.ndarray, ...],
    current_positions: np.ndarray | None,
    joint_tolerance_rad: float,
    time_epsilon_sec: float,
    arm_joint_velocity_limits_rad_s: np.ndarray,
) -> tuple[tuple[TrajectorySegment, ...], bool]:
    segments: list[TrajectorySegment] = []
    previous_effective_time_sec = 0.0
    synthetic_start_logged = False

    finite_velocity_limits = np.asarray(arm_joint_velocity_limits_rad_s[:7], dtype=float)
    finite_velocity_limits[~np.isfinite(finite_velocity_limits)] = 0.0

    def _vel(p: JointTrajectoryPoint) -> np.ndarray | None:
        if p.velocities_rad_s:
            return np.asarray(p.velocities_rad_s, dtype=float)
        return None

    for index, point in enumerate(trajectory.points):
        raw_time_sec = max(float(point.time_from_start_sec), 0.0)
        target_positions = np.asarray(expanded_targets[index], dtype=float).copy()
        target_velocities = _vel(point)

        if index == 0:
            if current_positions is not None and not joint_positions_reached(
                current_positions[:7],
                target_positions[:7],
                tolerance_rad=joint_tolerance_rad,
            ):
                start_positions = np.asarray(current_positions, dtype=float).copy()
                start_velocities: np.ndarray | None = np.zeros_like(start_positions)
                synthetic_start_logged = True
                arm_delta = np.abs(target_positions[:7] - start_positions[:7])
                with np.errstate(divide="ignore", invalid="ignore"):
                    required_duration_sec = np.divide(
                        arm_delta,
                        finite_velocity_limits,
                        out=np.zeros_like(arm_delta, dtype=float),
                        where=finite_velocity_limits > 0.0,
                    )
                min_duration_sec = max(float(np.max(required_duration_sec)) * 1.2, time_epsilon_sec)
            else:
                start_positions = target_positions.copy()
                start_velocities = None
                min_duration_sec = time_epsilon_sec
            if synthetic_start_logged:
                effective_time_sec = max(raw_time_sec, min_duration_sec)
            else:
                effective_time_sec = max(raw_time_sec, time_epsilon_sec)
        else:
            start_positions = np.asarray(expanded_targets[index - 1], dtype=float).copy()
            start_velocities = _vel(trajectory.points[index - 1])
            if raw_time_sec <= previous_effective_time_sec:
                effective_time_sec = previous_effective_time_sec + time_epsilon_sec
            else:
                effective_time_sec = raw_time_sec

        segments.append(
            TrajectorySegment(
                start_positions=start_positions,
                target_positions=target_positions,
                duration_sec=max(effective_time_sec - previous_effective_time_sec, time_epsilon_sec),
                start_velocities=start_velocities,
                target_velocities=target_velocities,
            )
        )
        previous_effective_time_sec = effective_time_sec

    return tuple(segments), synthetic_start_logged


def compute_trajectory_reference_state(
    *,
    active_segment: TrajectorySegment,
    current_positions: np.ndarray,
    current_velocities: np.ndarray | None,
    now_sec: float,
    time_epsilon_sec: float,
    arm_joint_velocity_limits_rad_s: np.ndarray,
    proportional_gain: float,
    derivative_gain: float,
) -> tuple[TrajectoryReferenceState, np.ndarray]:
    start_time_sec = active_segment.start_time_sec if active_segment.start_time_sec is not None else now_sec
    elapsed_time_sec = max(now_sec - start_time_sec, 0.0)
    duration_sec = max(active_segment.duration_sec, time_epsilon_sec)
    alpha = min(max(elapsed_time_sec / duration_sec, 0.0), 1.0)

    start_arm = np.asarray(active_segment.start_positions[:7], dtype=float)
    target_arm = np.asarray(active_segment.target_positions[:7], dtype=float)
    reference_arm = start_arm + (target_arm - start_arm) * alpha
    if elapsed_time_sec >= duration_sec:
        reference_arm_velocity = np.zeros_like(target_arm, dtype=float)
    else:
        reference_arm_velocity = (target_arm - start_arm) / duration_sec

    full_reference_positions = np.asarray(current_positions, dtype=float).copy()
    full_reference_positions[:7] = reference_arm
    full_reference_velocities = np.zeros_like(current_positions, dtype=float)
    full_reference_velocities[:7] = reference_arm_velocity

    current_arm_velocities = np.zeros(7, dtype=float)
    effective_derivative_gain = 0.0
    if current_velocities is not None:
        current_arm_velocities[: min(7, current_velocities.shape[0])] = np.asarray(current_velocities[:7], dtype=float)
        effective_derivative_gain = derivative_gain
    position_error = reference_arm - np.asarray(current_positions[:7], dtype=float)
    velocity_error = reference_arm_velocity - current_arm_velocities

    desired_qdot = reference_arm_velocity + proportional_gain * position_error + effective_derivative_gain * velocity_error
    desired_qdot = np.clip(
        desired_qdot,
        -np.asarray(arm_joint_velocity_limits_rad_s[:7], dtype=float),
        np.asarray(arm_joint_velocity_limits_rad_s[:7], dtype=float),
    )

    full_command_velocities = np.zeros_like(current_positions, dtype=float)
    full_command_velocities[:7] = desired_qdot
    return (
        TrajectoryReferenceState(
            reference_positions=full_reference_positions,
            reference_velocities=full_reference_velocities,
            alpha=alpha,
            elapsed_time_sec=elapsed_time_sec,
        ),
        full_command_velocities,
    )


def sample_trajectory_reference_state(
    *,
    active_segment: TrajectorySegment,
    current_positions: np.ndarray,
    now_sec: float,
    time_epsilon_sec: float,
    arm_joint_velocity_limits_rad_s: np.ndarray,
) -> tuple[TrajectoryReferenceState, np.ndarray]:
    start_time_sec = active_segment.start_time_sec if active_segment.start_time_sec is not None else now_sec
    elapsed_time_sec = max(now_sec - start_time_sec, 0.0)
    duration_sec = max(active_segment.duration_sec, time_epsilon_sec)
    alpha = min(max(elapsed_time_sec / duration_sec, 0.0), 1.0)

    start_arm = np.asarray(active_segment.start_positions[:7], dtype=float)
    target_arm = np.asarray(active_segment.target_positions[:7], dtype=float)

    sv = active_segment.start_velocities
    tv = active_segment.target_velocities
    use_hermite = (
        sv is not None
        and tv is not None
        and len(sv) >= 7
        and len(tv) >= 7
    )

    if use_hermite:
        v0 = np.asarray(sv[:7], dtype=float)
        v1 = np.asarray(tv[:7], dtype=float)
        t = alpha
        h00 = 2 * t**3 - 3 * t**2 + 1
        h10 = t**3 - 2 * t**2 + t
        h01 = -2 * t**3 + 3 * t**2
        h11 = t**3 - t**2
        reference_arm = h00 * start_arm + h10 * v0 * duration_sec + h01 * target_arm + h11 * v1 * duration_sec
        dh00 = 6 * t**2 - 6 * t
        dh10 = 3 * t**2 - 4 * t + 1
        dh01 = -6 * t**2 + 6 * t
        dh11 = 3 * t**2 - 2 * t
        reference_arm_velocity = (
            dh00 * start_arm + dh10 * v0 * duration_sec + dh01 * target_arm + dh11 * v1 * duration_sec
        ) / duration_sec
    else:
        reference_arm = start_arm + (target_arm - start_arm) * alpha
        if elapsed_time_sec >= duration_sec:
            reference_arm_velocity = np.zeros_like(target_arm, dtype=float)
        else:
            reference_arm_velocity = (target_arm - start_arm) / duration_sec

    full_reference_positions = np.asarray(current_positions, dtype=float).copy()
    full_reference_positions[:7] = reference_arm
    full_reference_velocities = np.zeros_like(current_positions, dtype=float)
    full_reference_velocities[:7] = reference_arm_velocity

    full_command_velocities = np.zeros_like(current_positions, dtype=float)
    full_command_velocities[:7] = np.clip(
        reference_arm_velocity,
        -np.asarray(arm_joint_velocity_limits_rad_s[:7], dtype=float),
        np.asarray(arm_joint_velocity_limits_rad_s[:7], dtype=float),
    )
    return (
        TrajectoryReferenceState(
            reference_positions=full_reference_positions,
            reference_velocities=full_reference_velocities,
            alpha=alpha,
            elapsed_time_sec=elapsed_time_sec,
        ),
        full_command_velocities,
    )
