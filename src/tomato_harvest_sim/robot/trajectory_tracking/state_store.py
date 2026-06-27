from __future__ import annotations

from tomato_harvest_sim.api.contracts import JointTrajectory, Pose3D, ScenePhase, SceneSnapshot
from tomato_harvest_sim.robot.api.trajectory_tracking import TrajectoryTrackingState


class TrajectoryTrackingStateStore:
    def __init__(self) -> None:
        self._state = TrajectoryTrackingState()

    @property
    def state(self) -> TrajectoryTrackingState:
        return self._state

    def normalize_snapshot(self, snapshot: SceneSnapshot) -> None:
        state = self._state
        state.gripper_closed = snapshot.gripper_closed
        cycle_changed = state.last_snapshot_cycle_id != snapshot.cycle_id
        state.last_snapshot_cycle_id = snapshot.cycle_id
        motion_signature = self.motion_signature_from_snapshot(snapshot)

        if state.blocked_motion_signature is not None:
            if motion_signature != state.blocked_motion_signature:
                self.clear_replan_block()
            elif snapshot.target_tool_pose is not None:
                state.target_pose = snapshot.target_tool_pose
                state.home_command_pending = False
                self.clear_joint_trajectory_state(clear_raw=True)
                self.clear_waypoint_state(clear_raw=True)
                return

        if snapshot.target_tool_pose is not None:
            if state.target_pose != snapshot.target_tool_pose:
                state.target_announced = False
                state.reached_announced = False
            if state.motion_waypoints != snapshot.motion_waypoints:
                self.clear_waypoint_state(clear_raw=False)
            if state.joint_trajectory != snapshot.motion_joint_trajectory:
                self.clear_joint_trajectory_state(clear_raw=False)
            state.target_pose = snapshot.target_tool_pose
            state.motion_waypoints = snapshot.motion_waypoints
            state.snapshot_active_waypoint_index = snapshot.active_waypoint_index
            state.joint_trajectory = snapshot.motion_joint_trajectory
            state.home_command_pending = False
            return

        state.target_pose = None
        state.motion_waypoints = ()
        state.snapshot_active_waypoint_index = None
        state.joint_trajectory = None
        self.clear_joint_trajectory_state(clear_raw=False)
        self.clear_waypoint_state(clear_raw=False)
        state.target_announced = False
        state.reached_announced = False
        if cycle_changed and snapshot.phase in {ScenePhase.READY, ScenePhase.STOPPED}:
            state.home_command_pending = True
            state.home_progress_announced = False
            return
        if snapshot.phase not in {ScenePhase.READY, ScenePhase.STOPPED}:
            state.home_command_pending = False
            state.home_progress_announced = False

    def clear_joint_trajectory_state(self, *, clear_raw: bool) -> None:
        state = self._state
        if clear_raw:
            state.joint_trajectory = None
        state.joint_trajectory_targets = ()
        state.joint_trajectory_segments = ()
        state.active_trajectory_point_index = 0
        state.last_control_time_sec = None
        state.trajectory_start_time_sec = None
        state.trajectory_expected_duration_sec = None
        state.trajectory_allowed_duration_sec = None
        state.last_observed_joint_positions = None
        state.last_observed_joint_time_sec = None

    def clear_waypoint_state(self, *, clear_raw: bool) -> None:
        state = self._state
        if clear_raw:
            state.motion_waypoints = ()
            state.snapshot_active_waypoint_index = None
        state.joint_waypoint_targets = ()
        state.waypoint_signature = None
        state.active_waypoint_index = 0

    def clear_replan_block(self) -> None:
        state = self._state
        state.blocked_motion_signature = None
        state.replan_status_announced = False

    def current_motion_signature(self) -> tuple[Pose3D | None, tuple[Pose3D, ...], JointTrajectory | None]:
        state = self._state
        return (state.target_pose, state.motion_waypoints, state.joint_trajectory)

    @staticmethod
    def motion_signature_from_snapshot(
        snapshot: SceneSnapshot,
    ) -> tuple[Pose3D | None, tuple[Pose3D, ...], JointTrajectory | None]:
        return (snapshot.target_tool_pose, snapshot.motion_waypoints, snapshot.motion_joint_trajectory)
