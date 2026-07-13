"""Safety-constrained online reconnection solver (Issue #46).

The solver is deliberately ROS independent.  Runtime adapters provide the latest
collision clearance; joint limits and a Panda elbow singularity indicator are
evaluated here so unsafe candidates never reach the trajectory publisher.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from tomato_harvest_sim.msg.contracts import JointTrajectory, JointTrajectoryPoint


PANDA_POSITION_LIMITS_RAD = {
    "panda_joint1": (-2.8973, 2.8973), "panda_joint2": (-1.7628, 1.7628),
    "panda_joint3": (-2.8973, 2.8973), "panda_joint4": (-3.0718, -0.0698),
    "panda_joint5": (-2.8973, 2.8973), "panda_joint6": (-0.0175, 3.7525),
    "panda_joint7": (-2.8973, 2.8973),
}
PANDA_VELOCITY_LIMITS_RAD_S = {
    **{f"panda_joint{i}": 2.175 for i in range(1, 5)},
    **{f"panda_joint{i}": 2.61 for i in range(5, 8)},
}
PANDA_ACCELERATION_LIMITS_RAD_S2 = {
    "panda_joint1": 15.0, "panda_joint2": 7.5, "panda_joint3": 10.0,
    "panda_joint4": 12.5, "panda_joint5": 15.0, "panda_joint6": 20.0,
    "panda_joint7": 20.0,
}


@dataclass(frozen=True)
class SafetyObservation:
    """Environment observations not derivable from a joint vector."""

    collision_clearance_m: float | None = None
    singularity_measure: float | None = None


@dataclass(frozen=True)
class SafeSolverPolicy:
    collision_stop_m: float = 0.02
    collision_slow_m: float = 0.08
    singularity_stop: float = 0.05
    singularity_slow: float = 0.20
    joint_margin_rad: float = 0.05
    nominal_velocity_rad_s: float = 0.5
    nominal_acceleration_rad_s2: float = 1.0
    min_duration_sec: float = 0.5
    segments: int = 12


@dataclass(frozen=True)
class SolverResult:
    trajectory: JointTrajectory | None
    reason: str
    speed_scale: float


def panda_singularity_measure(joint_names: tuple[str, ...], positions: tuple[float, ...]) -> float | None:
    """Return an elbow-extension indicator in [0, 1].

    Panda joint 4 cannot cross zero; approaching its upper bound extends the
    elbow and approaches the known arm singularity used by the initial-pose suite.
    The adapter accepts a full Jacobian-derived measure when one is available.
    """
    by_name = dict(zip(joint_names, positions))
    q4 = by_name.get("panda_joint4")
    return None if q4 is None else min(1.0, abs(math.sin(q4)))


def solve_safe_reconnection(
    *, joint_names: tuple[str, ...], start_positions_rad: tuple[float, ...],
    target_positions_rad: tuple[float, ...], observation: SafetyObservation = SafetyObservation(),
    policy: SafeSolverPolicy = SafeSolverPolicy(),
) -> SolverResult:
    """Generate a smooth reconnect trajectory, or an explicit safe-stop result."""
    if not (len(joint_names) == len(start_positions_rad) == len(target_positions_rad)):
        return SolverResult(None, "invalid_joint_vector", 0.0)
    if not all(math.isfinite(v) for v in (*start_positions_rad, *target_positions_rad)):
        return SolverResult(None, "non_finite_joint_state", 0.0)

    for name, start, target in zip(joint_names, start_positions_rad, target_positions_rad):
        limits = PANDA_POSITION_LIMITS_RAD.get(name)
        if limits and not all(limits[0] + policy.joint_margin_rad <= value <= limits[1] - policy.joint_margin_rad for value in (start, target)):
            return SolverResult(None, f"joint_position_limit:{name}", 0.0)

    clearance = observation.collision_clearance_m
    if clearance is not None and clearance <= policy.collision_stop_m:
        return SolverResult(None, "collision_proximity_stop", 0.0)
    singularity = observation.singularity_measure
    if singularity is None:
        singularity = panda_singularity_measure(joint_names, start_positions_rad)
    if singularity is not None and singularity <= policy.singularity_stop:
        return SolverResult(None, "singularity_stop", 0.0)

    scales = [1.0]
    if clearance is not None and clearance < policy.collision_slow_m:
        scales.append((clearance - policy.collision_stop_m) /
                      (policy.collision_slow_m - policy.collision_stop_m))
    if singularity is not None and singularity < policy.singularity_slow:
        scales.append((singularity - policy.singularity_stop) /
                      (policy.singularity_slow - policy.singularity_stop))
    speed_scale = max(0.05, min(scales))

    duration = policy.min_duration_sec
    for name, start, target in zip(joint_names, start_positions_rad, target_positions_rad):
        delta = abs(target - start)
        velocity_limit = min(policy.nominal_velocity_rad_s, PANDA_VELOCITY_LIMITS_RAD_S.get(name, policy.nominal_velocity_rad_s)) * speed_scale
        acceleration_limit = min(policy.nominal_acceleration_rad_s2, PANDA_ACCELERATION_LIMITS_RAD_S2.get(name, policy.nominal_acceleration_rad_s2)) * speed_scale
        # cubic smoothstep: peak |v|=1.5*d/T, peak |a|=6*d/T^2
        duration = max(duration, 1.5 * delta / velocity_limit,
                       math.sqrt(6.0 * delta / acceleration_limit))

    points = []
    for index in range(policy.segments + 1):
        u = index / policy.segments
        blend = 3.0 * u * u - 2.0 * u * u * u
        blend_rate = 6.0 * u * (1.0 - u) / duration
        points.append(JointTrajectoryPoint(
            positions_rad=tuple(start + (target - start) * blend for start, target in zip(start_positions_rad, target_positions_rad)),
            time_from_start_sec=duration * u,
            velocities_rad_s=tuple((target - start) * blend_rate for start, target in zip(start_positions_rad, target_positions_rad)),
        ))
    return SolverResult(JointTrajectory(joint_names=joint_names, points=tuple(points)), "ok", speed_scale)
