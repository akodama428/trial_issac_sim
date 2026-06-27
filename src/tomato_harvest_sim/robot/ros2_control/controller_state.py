from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class JointTrajectoryControllerState:
    controller_name: str
    desired_positions_rad: tuple[float, ...]
    actual_positions_rad: tuple[float, ...]
    desired_velocities_rad_s: tuple[float, ...]
    actual_velocities_rad_s: tuple[float, ...]
    error_norm_rad: float
    timestamp_sec: float
