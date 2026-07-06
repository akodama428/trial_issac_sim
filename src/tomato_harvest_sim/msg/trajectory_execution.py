"""軌跡実行ポート型定義。api/trajectory_execution.py から移設。"""
from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Protocol

from tomato_harvest_sim.msg.contracts import ExecutionPhaseSpec, JointTrajectory, Pose3D


class TrajectoryExecutionState(StrEnum):
    IDLE = "idle"
    ACCEPTED = "accepted"
    ACTIVE = "active"
    SUCCEEDED = "succeeded"
    ABORTED = "aborted"
    CANCELED = "canceled"
    REJECTED = "rejected"


@dataclass(frozen=True)
class TrajectoryExecutionRequest:
    controller_name: str
    command_name: str
    planner_name: str
    trajectory: JointTrajectory
    target_pose: Pose3D | None = None
    position_tolerance_m: float | None = None
    gripper_closed: bool | None = None
    execution_phase_spec: ExecutionPhaseSpec | None = None


@dataclass(frozen=True)
class TrajectoryExecutionFeedback:
    controller_name: str
    state: TrajectoryExecutionState
    desired_positions_rad: tuple[float, ...]
    actual_positions_rad: tuple[float, ...]
    desired_velocities_rad_s: tuple[float, ...]
    actual_velocities_rad_s: tuple[float, ...]
    error_norm_rad: float
    timestamp_sec: float


@dataclass(frozen=True)
class TrajectoryExecutionResult:
    controller_name: str
    state: TrajectoryExecutionState
    message: str
    timestamp_sec: float


class TrajectoryExecutionPort(Protocol):
    def send_goal(self, request: TrajectoryExecutionRequest) -> bool: ...

    def cancel_goal(self) -> None: ...

    def step(self) -> None: ...

    def active_request(self) -> TrajectoryExecutionRequest | None: ...

    def current_feedback(self) -> TrajectoryExecutionFeedback | None: ...

    def current_result(self) -> TrajectoryExecutionResult | None: ...
