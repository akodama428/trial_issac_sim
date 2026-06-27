from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from tomato_harvest_sim.api.contracts import ExecutionPhaseSpec, JointStateSnapshot, JointTrajectory, Pose3D


@dataclass(frozen=True)
class FrankaMotionProgress:
    active_target: bool
    reached: bool
    distance_m: float | None


@dataclass(frozen=True)
class ObservationData:
    joint_positions: np.ndarray | None
    joint_velocities: np.ndarray | None
    end_effector_pose: Pose3D | None
    joint_state_snapshot: JointStateSnapshot | None


class FrankaExecutionDriverProtocol(Protocol):
    def initialize_if_needed(self) -> bool: ...

    def get_observation(self) -> ObservationData: ...

    def current_joint_positions(self) -> np.ndarray | None: ...

    def current_joint_velocities(self) -> np.ndarray | None: ...

    def current_end_effector_pose(self) -> Pose3D | None: ...

    def current_joint_state_snapshot(self) -> JointStateSnapshot | None: ...

    def home_joint_positions(self) -> np.ndarray | None: ...

    def expand_joint_targets(self, joint_positions: np.ndarray) -> np.ndarray: ...

    def solve_joint_targets_for_pose(self, target_pose: Pose3D, *, position_tolerance_m: float) -> np.ndarray | None: ...

    def set_joint_positions_with_debug(self, positions: np.ndarray, *, context: str) -> None: ...

    def set_joint_velocity_targets_with_debug(
        self,
        *,
        positions: np.ndarray | None,
        velocities: np.ndarray,
        context: str,
    ) -> None: ...


@dataclass
class TrajectorySegment:
    start_positions: np.ndarray
    target_positions: np.ndarray
    duration_sec: float
    deadline_sec: float | None = None
    start_time_sec: float | None = None
    initial_error_max: float | None = None


@dataclass(frozen=True)
class TrajectoryReferenceState:
    reference_positions: np.ndarray
    reference_velocities: np.ndarray
    alpha: float
    elapsed_time_sec: float


@dataclass
class TrackingCommand:
    positions: np.ndarray
    context: str
    velocities: np.ndarray | None = None


@dataclass(frozen=True)
class TrackingStepResult:
    command: TrackingCommand | None = None
    log_message: str | None = None
    reached: bool = False
    replan_reason: str | None = None


@dataclass
class TrajectoryTrackingState:
    target_pose: Pose3D | None = None
    motion_waypoints: tuple[Pose3D, ...] = ()
    snapshot_active_waypoint_index: int | None = None
    joint_trajectory: JointTrajectory | None = None
    execution_phase_spec: ExecutionPhaseSpec | None = None
    position_tolerance_m: float | None = None
    gripper_closed: bool = False
    home_command_pending: bool = False
    home_progress_announced: bool = False
    target_announced: bool = False
    reached_announced: bool = False
    last_snapshot_cycle_id: int | None = None
    waypoint_signature: tuple[Pose3D, ...] | None = None
    joint_waypoint_targets: tuple[np.ndarray, ...] = ()
    active_waypoint_index: int = 0
    joint_trajectory_targets: tuple[np.ndarray, ...] = ()
    joint_trajectory_segments: tuple[TrajectorySegment, ...] = ()
    active_trajectory_point_index: int = 0
    rejected_joint_trajectory: JointTrajectory | None = None
    trajectory_start_time_sec: float | None = None
    trajectory_expected_duration_sec: float | None = None
    trajectory_allowed_duration_sec: float | None = None
    last_control_time_sec: float | None = None
    last_observed_joint_positions: np.ndarray | None = None
    last_observed_joint_time_sec: float | None = None
    blocked_motion_signature: tuple[
        Pose3D | None,
        tuple[Pose3D, ...],
        JointTrajectory | None,
        ExecutionPhaseSpec | None,
    ] | None = None
    pending_replan_reason: str | None = None
    replan_status_announced: bool = False
    trajectory_preview_cache: dict[JointTrajectory, tuple[Pose3D, ...]] = field(default_factory=dict)
    arm_hold_joint_positions: np.ndarray | None = None
