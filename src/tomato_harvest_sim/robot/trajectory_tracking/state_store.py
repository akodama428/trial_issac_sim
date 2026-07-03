from __future__ import annotations

from tomato_harvest_sim.api.contracts import (
    ExecutionPhaseSpec,
    JointTrajectory,
    PhaseMotionPlan,
    Pose3D,
    ScenePhase,
    SceneSnapshot,
)
from tomato_harvest_sim.robot.api.trajectory_tracking import TrajectoryTrackingState
from tomato_harvest_sim.robot.trajectory_tracking.phase_spec_loader import PhaseSpecLoader


class ExecutionStateStore:
    def __init__(self, *, phase_spec_loader: PhaseSpecLoader | None = None) -> None:
        self._state = TrajectoryTrackingState()
        self._phase_spec_loader = phase_spec_loader or PhaseSpecLoader()

    @property
    def state(self) -> TrajectoryTrackingState:
        return self._state

    def normalize_snapshot(self, snapshot: SceneSnapshot) -> None:
        state = self._state
        state.gripper_closed = snapshot.gripper_closed
        cycle_changed = state.last_snapshot_cycle_id != snapshot.cycle_id
        state.last_snapshot_cycle_id = snapshot.cycle_id
        motion_signature = self.motion_signature_from_snapshot(snapshot)
        active_phase_motion_plan = snapshot.active_phase_motion_plan
        active_spec = self._build_execution_phase_spec(active_phase_motion_plan)
        active_target_pose = snapshot.target_tool_pose
        active_waypoints: tuple[Pose3D, ...] = ()
        active_joint_trajectory: JointTrajectory | None = None
        if active_spec is not None:
            active_target_pose = active_spec.motion.phase_goal_pose
            active_waypoints = active_spec.motion.active_waypoints
            active_joint_trajectory = active_spec.motion.joint_trajectory

        if state.blocked_motion_signature is not None:
            if motion_signature != state.blocked_motion_signature:
                self.clear_replan_block()
            elif active_target_pose is not None:
                state.target_pose = active_target_pose
                state.active_phase_motion_plan = active_phase_motion_plan
                state.execution_phase_spec = active_spec
                state.position_tolerance_m = (
                    None if active_spec is None else active_spec.intent.success.position_tolerance_m
                )
                state.home_command_pending = False
                self.clear_joint_trajectory_state(clear_raw=True)
                self.clear_waypoint_state(clear_raw=True)
                return

        if active_target_pose is not None:
            if state.target_pose != active_target_pose:
                state.target_announced = False
                state.reached_announced = False
            if state.motion_waypoints != active_waypoints:
                self.clear_waypoint_state(clear_raw=False)
            if state.joint_trajectory != active_joint_trajectory:
                self.clear_joint_trajectory_state(clear_raw=False)
            state.target_pose = active_target_pose
            state.motion_waypoints = active_waypoints
            state.snapshot_active_waypoint_index = None
            state.joint_trajectory = active_joint_trajectory
            state.active_phase_motion_plan = active_phase_motion_plan
            state.execution_phase_spec = active_spec
            state.position_tolerance_m = None if active_spec is None else active_spec.intent.success.position_tolerance_m
            state.home_command_pending = False
            state.arm_hold_joint_positions = None
            return

        if state.target_pose is not None:
            if state.joint_trajectory_targets:
                state.arm_hold_joint_positions = state.joint_trajectory_targets[-1].copy()
            elif state.joint_waypoint_targets:
                state.arm_hold_joint_positions = state.joint_waypoint_targets[-1].copy()

        state.target_pose = None
        state.motion_waypoints = ()
        state.snapshot_active_waypoint_index = None
        state.joint_trajectory = None
        state.active_phase_motion_plan = None
        state.execution_phase_spec = None
        state.position_tolerance_m = None
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

    def current_motion_signature(
        self,
    ) -> tuple[Pose3D | None, tuple[Pose3D, ...], JointTrajectory | None, PhaseMotionPlan | None]:
        state = self._state
        return (state.target_pose, state.motion_waypoints, state.joint_trajectory, state.active_phase_motion_plan)

    @staticmethod
    def motion_signature_from_snapshot(
        snapshot: SceneSnapshot,
    ) -> tuple[Pose3D | None, tuple[Pose3D, ...], JointTrajectory | None, PhaseMotionPlan | None]:
        plan = snapshot.active_phase_motion_plan
        if plan is not None:
            return (
                plan.phase_goal_pose,
                plan.active_waypoints,
                plan.joint_trajectory,
                plan,
            )
        return (snapshot.target_tool_pose, (), None, None)

    def _build_execution_phase_spec(self, plan: PhaseMotionPlan | None) -> ExecutionPhaseSpec | None:
        if plan is None:
            return None
        return self._phase_spec_loader.build_spec(plan)
