from __future__ import annotations

from typing import TYPE_CHECKING

__all__ = [
    "ExecutionMonitor",
    "ExecutionStateStore",
    "FollowJointTrajectoryActionClient",
    "FrankaExecutionDriverProtocol",
    "FrankaMotionProgress",
    "FrankaTrajectoryExecutionManager",
    "ObservationData",
    "PhaseSpecLoader",
    "TrackingCommand",
    "TrackingStepResult",
    "TrajectoryReferenceState",
    "TrajectorySegment",
    "TrajectoryTracker",
    "TrajectoryTrackingCoordinator",
    "TrajectoryTrackingState",
    "build_joint_trajectory_segments",
    "compute_trajectory_reference_state",
    "is_pose_reached",
    "joint_positions_reached",
    "pose_distance_m",
    "step_toward_joint_positions",
]

if TYPE_CHECKING:
    from tomato_harvest_sim.robot.msg.trajectory_tracking import (
        FrankaExecutionDriverProtocol,
        FrankaMotionProgress,
        ObservationData,
        TrackingCommand,
        TrackingStepResult,
        TrajectoryReferenceState,
        TrajectorySegment,
        TrajectoryTrackingState,
    )
    from tomato_harvest_sim.robot.trajectory_tracking.action_client import FollowJointTrajectoryActionClient
    from tomato_harvest_sim.robot.trajectory_tracking.coordinator import (
        FrankaTrajectoryExecutionManager,
        TrajectoryTrackingCoordinator,
    )
    from tomato_harvest_sim.robot.trajectory_tracking.execution_monitor import ExecutionMonitor
    from tomato_harvest_sim.robot.trajectory_tracking.phase_spec_loader import PhaseSpecLoader
    from tomato_harvest_sim.robot.trajectory_tracking.reference_tracking import (
        build_joint_trajectory_segments,
        compute_trajectory_reference_state,
        joint_positions_reached,
        step_toward_joint_positions,
    )
    from tomato_harvest_sim.robot.trajectory_tracking.state_store import ExecutionStateStore
    from tomato_harvest_sim.robot.trajectory_tracking.tracker import TrajectoryTracker, is_pose_reached, pose_distance_m


def __getattr__(name: str):
    if name in {
        "FrankaExecutionDriverProtocol",
        "FrankaMotionProgress",
        "ObservationData",
        "TrackingCommand",
        "TrackingStepResult",
        "TrajectoryReferenceState",
        "TrajectorySegment",
        "TrajectoryTrackingState",
    }:
        from tomato_harvest_sim.robot.api import trajectory_tracking as api_module

        return getattr(api_module, name)

    if name in {"FrankaTrajectoryExecutionManager", "TrajectoryTrackingCoordinator"}:
        from tomato_harvest_sim.robot.trajectory_tracking import coordinator as coordinator_module

        return getattr(coordinator_module, name)

    if name in {"FollowJointTrajectoryActionClient"}:
        from tomato_harvest_sim.robot.trajectory_tracking import action_client as module

        return getattr(module, name)

    if name in {"ExecutionMonitor"}:
        from tomato_harvest_sim.robot.trajectory_tracking import execution_monitor as module

        return getattr(module, name)

    if name in {
        "build_joint_trajectory_segments",
        "compute_trajectory_reference_state",
        "joint_positions_reached",
        "step_toward_joint_positions",
    }:
        from tomato_harvest_sim.robot.trajectory_tracking import reference_tracking as module

        return getattr(module, name)

    if name in {"ExecutionStateStore"}:
        from tomato_harvest_sim.robot.trajectory_tracking import state_store as module

        return getattr(module, name)

    if name in {"PhaseSpecLoader"}:
        from tomato_harvest_sim.robot.trajectory_tracking import phase_spec_loader as module

        return getattr(module, name)

    if name in {"TrajectoryTracker", "is_pose_reached", "pose_distance_m"}:
        from tomato_harvest_sim.robot.trajectory_tracking import tracker as module

        return getattr(module, name)

    raise AttributeError(name)
