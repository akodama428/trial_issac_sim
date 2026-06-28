from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import yaml

from tomato_harvest_sim.api.hardware_control import HardwareControlPort
from tomato_harvest_sim.api.trajectory_execution import (
    TrajectoryExecutionPort,
    TrajectoryExecutionRequest,
)
from tomato_harvest_sim.api.contracts import JointStateSnapshot, JointTrajectory, Pose3D, SceneSnapshot
from tomato_harvest_sim.robot.ros2_control import JointTrajectoryControllerBridge
from tomato_harvest_sim.robot.api.trajectory_tracking import (
    FrankaExecutionDriverProtocol,
    FrankaMotionProgress,
    ObservationData,
    TrackingCommand,
)
from tomato_harvest_sim.robot.trajectory_tracking.action_client import FollowJointTrajectoryActionClient
from tomato_harvest_sim.robot.trajectory_tracking.execution_monitor import ExecutionMonitor
from tomato_harvest_sim.robot.trajectory_tracking.reference_tracking import build_joint_trajectory_segments
from tomato_harvest_sim.robot.trajectory_tracking.tracker import (
    TrajectoryTracker,
    _hand_pose_from_grasp_center_pose,
    is_pose_reached,
    pose_distance_m,
)
from tomato_harvest_sim.robot.trajectory_tracking.state_store import ExecutionStateStore


def _joint_limits_path() -> Path:
    return Path(__file__).resolve().parents[1] / "moveit_config" / "joint_limits.yaml"


def _load_arm_joint_velocity_limits_rad_s(joint_names: tuple[str, ...]) -> np.ndarray:
    try:
        payload = yaml.safe_load(_joint_limits_path().read_text(encoding="utf-8"))
    except Exception:
        return np.full(len(joint_names), np.inf, dtype=float)

    limits = payload.get("joint_limits") if isinstance(payload, dict) else None
    if not isinstance(limits, dict):
        return np.full(len(joint_names), np.inf, dtype=float)

    values: list[float] = []
    for joint_name in joint_names:
        joint_limit = limits.get(joint_name, {})
        if not isinstance(joint_limit, dict) or not joint_limit.get("has_velocity_limits", False):
            values.append(float("inf"))
            continue
        values.append(float(joint_limit.get("max_velocity", float("inf"))))
    return np.asarray(values, dtype=float)


class TrajectoryTrackingCoordinator:
    TRAJECTORY_TIME_EPSILON_SEC = 1e-3
    ARM_JOINT_NAMES = (
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    )
    GRASP_TARGET_OFFSET_FROM_HAND_M = (0.0, 0.0, 0.0584)
    DEBUG_TRAJECTORY_ENV = "TOMATO_HARVEST_DEBUG_TRAJECTORY"

    def __init__(
        self,
        *,
        driver: FrankaExecutionDriverProtocol,
        hardware_control_port: HardwareControlPort,
        trajectory_execution_port: TrajectoryExecutionPort | None = None,
        position_tolerance_m: float = 0.03,
        max_joint_step_rad: float = 0.05,
        max_gripper_step_rad: float = 0.01,
        joint_tolerance_rad: float = 0.03,
    ) -> None:
        self._driver = driver
        self._hardware_control_port = hardware_control_port
        resolved_trajectory_execution_port = trajectory_execution_port or JointTrajectoryControllerBridge(
            hardware=self._hardware_control_port
        )
        self._action_client = FollowJointTrajectoryActionClient(port=resolved_trajectory_execution_port)
        self._execution_monitor = ExecutionMonitor()
        self._position_tolerance_m = position_tolerance_m
        self._joint_tolerance_rad = joint_tolerance_rad
        self._trajectory_debug_enabled = os.environ.get(self.DEBUG_TRAJECTORY_ENV, "").strip() not in {"", "0", "false", "False"}
        self._state_store = ExecutionStateStore()
        self._pending_snapshot: SceneSnapshot | None = None
        self._pending_snapshot_prepared = False
        self._arm_joint_velocity_limits_rad_s = _load_arm_joint_velocity_limits_rad_s(self.ARM_JOINT_NAMES)
        self._tracker = TrajectoryTracker(
            position_tolerance_m=position_tolerance_m,
            max_joint_step_rad=max_joint_step_rad,
            max_gripper_step_rad=max_gripper_step_rad,
            joint_tolerance_rad=joint_tolerance_rad,
            debug_log=self._debug_log,
        )
        self._active_trajectory: JointTrajectory | None = None

    def run_cycle(self, snapshot: SceneSnapshot) -> str | None:
        self._pending_snapshot = snapshot
        self._pending_snapshot_prepared = False
        if not self._initialize_if_needed():
            return None

        self._state_store.normalize_snapshot(snapshot)
        observation = self._get_observation()
        state = self._state_store.state

        if self._should_use_controller_trajectory():
            return self._run_controller_trajectory_cycle()

        preparation = self._tracker.prepare_tracking_state(
            store=self._state_store,
            observation=observation,
            solve_joint_targets_for_waypoints=self._solve_joint_targets_for_waypoints,
        )
        if preparation is not None:
            self._apply_tracking_command(preparation.command)
            return preparation.log_message

        result = self._tracker.compute_step(
            store=self._state_store,
            observation=observation,
            home_joint_positions=self._driver.home_joint_positions(),
            solve_joint_targets_for_pose=self._solve_joint_targets_for_pose,
        )
        self._apply_tracking_command(result.command)
        if state.blocked_motion_signature is None and result.replan_reason is not None:
            state.blocked_motion_signature = self._state_store.current_motion_signature()
            state.pending_replan_reason = result.replan_reason
        return result.log_message

    def sync_with_snapshot(self, snapshot: SceneSnapshot) -> None:
        self._pending_snapshot = snapshot
        self._pending_snapshot_prepared = False
        self._state_store.normalize_snapshot(snapshot)
        observation = self._get_observation_for_sync()
        if self._should_use_controller_trajectory():
            self._prime_controller_trajectory_state(observation.joint_positions)
            self._pending_snapshot_prepared = True
            return
        preparation = self._tracker.prepare_tracking_state(
            store=self._state_store,
            observation=observation,
            solve_joint_targets_for_waypoints=self._solve_joint_targets_for_waypoints,
        )
        self._apply_tracking_command(None if preparation is None else preparation.command)
        self._pending_snapshot_prepared = True

    def step(self) -> str | None:
        if self._pending_snapshot is None:
            return None
        return self.run_cycle(self._pending_snapshot)

    def progress(self) -> FrankaMotionProgress:
        if self._pending_snapshot is None or not self._initialize_if_needed():
            return FrankaMotionProgress(active_target=False, reached=False, distance_m=None)
        state = self._state_store.state
        if state.target_pose is None:
            return FrankaMotionProgress(active_target=False, reached=False, distance_m=None)
        current_pose = self.current_end_effector_pose()
        if current_pose is None:
            return FrankaMotionProgress(active_target=True, reached=False, distance_m=None)
        distance_m = pose_distance_m(current_pose, state.target_pose)
        position_tolerance_m = state.position_tolerance_m or self._position_tolerance_m
        return FrankaMotionProgress(
            active_target=True,
            reached=distance_m <= position_tolerance_m,
            distance_m=distance_m,
        )

    def current_end_effector_pose(self) -> Pose3D | None:
        if not self._initialize_if_needed():
            return None
        hardware_state = self._hardware_control_port.read_state()
        return None if hardware_state is None else hardware_state.end_effector_pose

    def current_joint_state_snapshot(self) -> JointStateSnapshot | None:
        if not self._initialize_if_needed():
            return None
        hardware_state = self._hardware_control_port.read_state()
        return None if hardware_state is None else hardware_state.joint_state_snapshot

    def preview_end_effector_path_for_joint_trajectory(self, trajectory: JointTrajectory | None) -> tuple[Pose3D, ...]:
        if trajectory is None or not trajectory.points or not self._initialize_if_needed():
            return ()
        cached_preview = self._state_store.state.trajectory_preview_cache.get(trajectory)
        if cached_preview is not None:
            return cached_preview

        preview_path_provider = getattr(self._driver, "preview_end_effector_path_for_joint_trajectory", None)
        if callable(preview_path_provider):
            preview_path = tuple(preview_path_provider(trajectory))
            self._state_store.state.trajectory_preview_cache[trajectory] = preview_path
            return preview_path

        preview_pose_provider = getattr(self._driver, "preview_end_effector_pose_for_joint_positions", None)
        if not callable(preview_pose_provider):
            return ()

        preview_path: list[Pose3D] = []
        for point in trajectory.points:
            preview_pose = preview_pose_provider(np.asarray(point.positions_rad, dtype=float))
            if preview_pose is None:
                continue
            if not preview_path or preview_path[-1] != preview_pose:
                preview_path.append(preview_pose)
        cached_path = tuple(preview_path)
        self._state_store.state.trajectory_preview_cache[trajectory] = cached_path
        return cached_path

    def consume_replan_request(self) -> str | None:
        reason = self._state_store.state.pending_replan_reason
        self._state_store.state.pending_replan_reason = None
        return reason

    def current_controller_state(self) -> object | None:
        return self._action_client.current_controller_state()

    def log_post_update_debug_snapshot(self) -> None:
        if not self._trajectory_debug_enabled or not self._initialize_if_needed():
            return
        observation = self._get_observation()
        self._debug_log(
            "[Simulator][TrajectoryDebug][post_update] "
            f"current_q={self._format_joint_positions(observation.joint_positions[:7]) if observation.joint_positions is not None else 'n/a'} "
            f"ee_xyz={self._format_pose_xyz(observation.end_effector_pose)} "
            f"target_xyz={self._format_pose_xyz(self._state_store.state.target_pose)}"
        )

    def _initialize_if_needed(self) -> bool:
        return self._driver.initialize_if_needed() and self._hardware_control_port.initialize_if_needed()

    def _get_observation(self) -> ObservationData:
        hardware_state = self._hardware_control_port.read_state()
        if hardware_state is not None:
            return ObservationData(
                joint_positions=np.asarray(hardware_state.positions_rad, dtype=float),
                joint_velocities=np.asarray(hardware_state.velocities_rad_s, dtype=float),
                end_effector_pose=hardware_state.end_effector_pose,
                joint_state_snapshot=hardware_state.joint_state_snapshot,
            )
        observation_provider = getattr(self._driver, "get_observation", None)
        if callable(observation_provider):
            return observation_provider()
        return self._get_observation_for_sync()

    def _get_observation_for_sync(self) -> ObservationData:
        return ObservationData(
            joint_positions=self._driver.current_joint_positions(),
            joint_velocities=self._driver.current_joint_velocities(),
            end_effector_pose=self._driver.current_end_effector_pose(),
            joint_state_snapshot=self._driver.current_joint_state_snapshot(),
        )

    def _apply_tracking_command(self, command: TrackingCommand | None) -> None:
        if command is None:
            return
        if command.velocities is None:
            self._driver.set_joint_positions_with_debug(command.positions, context=command.context)
        else:
            self._driver.set_joint_velocity_targets_with_debug(
                positions=command.positions,
                velocities=command.velocities,
                context=command.context,
            )
        hw = self._hardware_control_port.read_state()
        if hw is not None:
            n = len(hw.positions_rad)
            desired_pos = tuple(float(v) for v in command.positions[:n])
            zero_vel = tuple(0.0 for _ in range(n))
            desired_vel = (
                tuple(float(v) for v in command.velocities[:n])
                if command.velocities is not None
                else zero_vel
            )
            self._action_client.update_external_command_state(
                desired_positions=desired_pos,
                desired_velocities=desired_vel,
                actual_positions=tuple(float(v) for v in hw.positions_rad),
                actual_velocities=tuple(float(v) for v in hw.velocities_rad_s),
                timestamp_sec=hw.timestamp_sec,
            )

    def _solve_joint_targets_for_pose(self, target_pose: Pose3D) -> np.ndarray | None:
        return self._driver.solve_joint_targets_for_pose(
            target_pose,
            position_tolerance_m=self._state_store.state.position_tolerance_m or self._position_tolerance_m,
        )

    def _solve_joint_targets_for_waypoints(self, waypoints: tuple[Pose3D, ...]) -> tuple[np.ndarray, ...]:
        joint_targets: list[np.ndarray] = []
        for waypoint in waypoints:
            solver_target_pose = _hand_pose_from_grasp_center_pose(
                waypoint,
                grasp_center_offset_from_hand_m=self.GRASP_TARGET_OFFSET_FROM_HAND_M,
            )
            target = self._solve_joint_targets_for_pose(solver_target_pose)
            if target is None:
                return ()
            joint_targets.append(target)
        return tuple(joint_targets)

    def _expand_joint_targets(self, joint_positions: np.ndarray) -> np.ndarray:
        return self._driver.expand_joint_targets(joint_positions)

    def _run_controller_trajectory_cycle(self) -> str | None:
        state = self._state_store.state
        trajectory = state.joint_trajectory
        if trajectory is None or not trajectory.points:
            if self._action_client.active_request() is not None:
                self._action_client.cancel_goal()
            self._active_trajectory = None
            return None

        self._execution_monitor.reset_for_trajectory(trajectory)
        self._prime_controller_trajectory_state(self._get_observation().joint_positions)
        if self._active_trajectory != trajectory and self._action_client.active_request() is None:
            accepted = self._action_client.send_goal(
                TrajectoryExecutionRequest(
                    controller_name="joint_trajectory_controller",
                    command_name="joint_trajectory",
                    planner_name="trajectory_tracking",
                    trajectory=trajectory,
                    target_pose=state.target_pose,
                    position_tolerance_m=state.position_tolerance_m or self._position_tolerance_m,
                    gripper_closed=state.gripper_closed,
                    execution_phase_spec=state.execution_phase_spec,
                )
            )
            if not accepted:
                result = self._action_client.current_result()
                log_message, replan_reason = self._execution_monitor.result_update(result)
                if replan_reason is not None:
                    self._mark_replan_block(replan_reason)
                return log_message
            self._active_trajectory = trajectory

        self._action_client.step()
        active_segment_index = self._action_client.active_segment_index()
        if active_segment_index is not None:
            state.active_trajectory_point_index = active_segment_index
        feedback = self._action_client.current_feedback()
        acceptance_log = self._execution_monitor.acceptance_log(feedback)
        if acceptance_log is not None:
            return acceptance_log

        result = self._action_client.current_result()
        log_message, replan_reason = self._execution_monitor.result_update(result)
        if result is not None and result.state.value == "succeeded":
            return log_message
        if replan_reason is not None:
            self._mark_replan_block(replan_reason)
            self._active_trajectory = None
            return log_message
        return None

    def _mark_replan_block(self, reason: str) -> None:
        state = self._state_store.state
        state.blocked_motion_signature = self._state_store.current_motion_signature()
        state.pending_replan_reason = reason
        state.replan_status_announced = False
        state.joint_trajectory_targets = ()
        state.joint_trajectory_segments = ()
        state.active_trajectory_point_index = 0

    def _should_use_controller_trajectory(self) -> bool:
        state = self._state_store.state
        return (
            state.joint_trajectory is not None
            and bool(state.joint_trajectory.points)
            and state.blocked_motion_signature is None
        )

    def _prime_controller_trajectory_state(self, current_positions: np.ndarray | None) -> None:
        state = self._state_store.state
        trajectory = state.joint_trajectory
        if trajectory is None or not trajectory.points:
            state.joint_trajectory_targets = ()
            state.joint_trajectory_segments = ()
            state.active_trajectory_point_index = 0
            return
        expanded_targets = tuple(
            self._expand_joint_targets(np.asarray(point.positions_rad, dtype=float))
            for point in trajectory.points
        )
        state.joint_trajectory_targets = expanded_targets
        state.joint_trajectory_segments, _ = build_joint_trajectory_segments(
            trajectory=trajectory,
            expanded_targets=expanded_targets,
            current_positions=current_positions,
            joint_tolerance_rad=self._joint_tolerance_rad,
            time_epsilon_sec=self.TRAJECTORY_TIME_EPSILON_SEC,
            arm_joint_velocity_limits_rad_s=self._arm_joint_velocity_limits_rad_s,
        )
        state.active_trajectory_point_index = min(state.active_trajectory_point_index, max(len(expanded_targets) - 1, 0))

    def _debug_log(self, message: str) -> None:
        if self._trajectory_debug_enabled:
            print(message, flush=True)

    @staticmethod
    def _format_joint_positions(values: tuple[float, ...] | np.ndarray) -> str:
        return "[" + ", ".join(f"{float(value):.4f}" for value in values) + "]"

    @staticmethod
    def _format_pose_xyz(pose: Pose3D | None) -> str:
        if pose is None:
            return "(n/a)"
        return f"({pose.x:.4f}, {pose.y:.4f}, {pose.z:.4f})"

    @property
    def _active_trajectory_point_index(self) -> int:
        return self._state_store.state.active_trajectory_point_index

    @_active_trajectory_point_index.setter
    def _active_trajectory_point_index(self, value: int) -> None:
        self._state_store.state.active_trajectory_point_index = value

    @property
    def _joint_trajectory_segments(self):
        return self._state_store.state.joint_trajectory_segments

    @property
    def _joint_waypoint_targets(self):
        return self._state_store.state.joint_waypoint_targets

    @property
    def _joint_trajectory_targets(self):
        return self._state_store.state.joint_trajectory_targets


FrankaTrajectoryExecutionManager = TrajectoryTrackingCoordinator
