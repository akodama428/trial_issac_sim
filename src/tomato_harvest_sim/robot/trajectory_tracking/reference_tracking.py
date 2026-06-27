from __future__ import annotations

import numpy as np

from tomato_harvest_sim.api.contracts import JointTrajectory
from tomato_harvest_sim.robot.api.trajectory_tracking import TrajectoryReferenceState, TrajectorySegment


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

    for index, point in enumerate(trajectory.points):
        raw_time_sec = max(float(point.time_from_start_sec), 0.0)
        target_positions = np.asarray(expanded_targets[index], dtype=float).copy()

        if index == 0:
            if current_positions is not None and not joint_positions_reached(
                current_positions[:7],
                target_positions[:7],
                tolerance_rad=joint_tolerance_rad,
            ):
                start_positions = np.asarray(current_positions, dtype=float).copy()
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
                min_duration_sec = time_epsilon_sec
            if synthetic_start_logged and raw_time_sec <= time_epsilon_sec:
                effective_time_sec = max(raw_time_sec, min_duration_sec)
            else:
                effective_time_sec = max(raw_time_sec, time_epsilon_sec)
        else:
            start_positions = np.asarray(expanded_targets[index - 1], dtype=float).copy()
            if raw_time_sec <= previous_effective_time_sec:
                effective_time_sec = previous_effective_time_sec + time_epsilon_sec
            else:
                effective_time_sec = raw_time_sec

        segments.append(
            TrajectorySegment(
                start_positions=start_positions,
                target_positions=target_positions,
                duration_sec=max(effective_time_sec - previous_effective_time_sec, time_epsilon_sec),
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
