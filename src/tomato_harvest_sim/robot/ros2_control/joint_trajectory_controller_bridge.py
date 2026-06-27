from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import yaml

from tomato_harvest_sim.api.hardware_control import HardwareCommandSample, HardwareControlPort, HardwareStateSample
from tomato_harvest_sim.api.contracts import AbortPolicy, Pose3D, SuccessJudge
from tomato_harvest_sim.api.trajectory_execution import (
    TrajectoryExecutionFeedback,
    TrajectoryExecutionPort,
    TrajectoryExecutionRequest,
    TrajectoryExecutionResult,
    TrajectoryExecutionState,
)
from tomato_harvest_sim.robot.ros2_control.controller_manager import ControllerManager
from tomato_harvest_sim.robot.ros2_control.controller_state import JointTrajectoryControllerState
from tomato_harvest_sim.robot.trajectory_tracking.reference_tracking import (
    build_joint_trajectory_segments,
    joint_positions_reached,
    sample_trajectory_reference_state,
)


def _joint_limits_path() -> Path:
    return Path(__file__).resolve().parents[1] / "moveit_config" / "joint_limits.yaml"


def _load_arm_joint_velocity_limits_rad_s(arm_dof: int) -> np.ndarray:
    try:
        payload = yaml.safe_load(_joint_limits_path().read_text(encoding="utf-8"))
    except Exception:
        return np.full(arm_dof, np.inf, dtype=float)

    limits = payload.get("joint_limits") if isinstance(payload, dict) else None
    if not isinstance(limits, dict):
        return np.full(arm_dof, np.inf, dtype=float)

    values: list[float] = []
    for joint_index in range(arm_dof):
        joint_name = f"panda_joint{joint_index + 1}"
        joint_limit = limits.get(joint_name, {})
        if not isinstance(joint_limit, dict) or not joint_limit.get("has_velocity_limits", False):
            values.append(float("inf"))
            continue
        values.append(float(joint_limit.get("max_velocity", float("inf"))))
    return np.asarray(values, dtype=float)


class JointTrajectoryControllerBridge(TrajectoryExecutionPort):
    ARM_DOF = 7
    PROGRESS_EPSILON_RAD = 1e-3
    STALL_TIMEOUT_SEC = 0.5

    def __init__(
        self,
        *,
        hardware: HardwareControlPort,
        controller_manager: ControllerManager | None = None,
        controller_name: str = "joint_trajectory_controller",
        goal_tolerance_rad: float = 0.03,
        path_tolerance_rad: float = 0.30,
        goal_time_tolerance_sec: float = 0.5,
        path_tolerance_grace_sec: float = 0.05,
        monotonic_time_sec: callable | None = None,
    ) -> None:
        self._hardware = hardware
        self._controller_manager = controller_manager or ControllerManager()
        self._controller_name = controller_name
        self._goal_tolerance_rad = goal_tolerance_rad
        self._path_tolerance_rad = path_tolerance_rad
        self._goal_time_tolerance_sec = goal_time_tolerance_sec
        self._path_tolerance_grace_sec = path_tolerance_grace_sec
        self._monotonic_time_sec = monotonic_time_sec or time.monotonic
        self._arm_joint_velocity_limits_rad_s = _load_arm_joint_velocity_limits_rad_s(self.ARM_DOF)

        self._active_request: TrajectoryExecutionRequest | None = None
        self._active_segments: tuple[object, ...] = ()
        self._active_segment_index = 0
        self._goal_start_time_sec: float | None = None
        self._segment_start_time_sec: float | None = None
        self._feedback: TrajectoryExecutionFeedback | None = None
        self._result: TrajectoryExecutionResult | None = None
        self._controller_state: JointTrajectoryControllerState | None = None
        self._goal_best_error_norm_rad: float | None = None
        self._goal_last_progress_time_sec: float | None = None
        self._segment_best_error_norm_rad: float | None = None
        self._segment_last_progress_time_sec: float | None = None

    def send_goal(self, request: TrajectoryExecutionRequest) -> bool:
        self._result = None
        if not self._controller_manager.ensure_controller(request.controller_name):
            self._result = TrajectoryExecutionResult(
                controller_name=request.controller_name,
                state=TrajectoryExecutionState.REJECTED,
                message="controller_unavailable",
                timestamp_sec=self._monotonic_time_sec(),
            )
            return False

        state = self._hardware.read_state()
        if state is None or not request.trajectory.points:
            self._result = TrajectoryExecutionResult(
                controller_name=request.controller_name,
                state=TrajectoryExecutionState.REJECTED,
                message="hardware_state_unavailable",
                timestamp_sec=self._monotonic_time_sec(),
            )
            return False

        expanded_targets = tuple(self._expand_positions(state.positions_rad, point.positions_rad) for point in request.trajectory.points)
        segments, _ = build_joint_trajectory_segments(
            trajectory=request.trajectory,
            expanded_targets=expanded_targets,
            current_positions=np.asarray(state.positions_rad, dtype=float),
            joint_tolerance_rad=self._goal_tolerance_rad,
            time_epsilon_sec=1e-3,
            arm_joint_velocity_limits_rad_s=self._arm_joint_velocity_limits_rad_s,
        )
        self._active_request = request
        self._active_segments = segments
        self._active_segment_index = 0
        self._goal_start_time_sec = None
        self._segment_start_time_sec = None
        self._goal_best_error_norm_rad = None
        self._goal_last_progress_time_sec = None
        self._segment_best_error_norm_rad = None
        self._segment_last_progress_time_sec = None
        self._feedback = TrajectoryExecutionFeedback(
            controller_name=request.controller_name,
            state=TrajectoryExecutionState.ACCEPTED,
            desired_positions_rad=tuple(state.positions_rad),
            actual_positions_rad=tuple(state.positions_rad),
            desired_velocities_rad_s=tuple(state.velocities_rad_s),
            actual_velocities_rad_s=tuple(state.velocities_rad_s),
            error_norm_rad=0.0,
            timestamp_sec=self._monotonic_time_sec(),
        )
        return True

    def cancel_goal(self) -> None:
        if self._active_request is None:
            return
        hardware_state = self._hardware.read_state()
        if hardware_state is not None:
            self._write_terminal_hold_command(hardware_state, context_suffix="canceled")
        self._result = TrajectoryExecutionResult(
            controller_name=self._active_request.controller_name,
            state=TrajectoryExecutionState.CANCELED,
            message="goal_canceled",
            timestamp_sec=self._monotonic_time_sec(),
        )
        self._clear_active_goal()

    def step(self) -> None:
        if self._active_request is None or not self._active_segments:
            return

        hardware_state = self._hardware.read_state()
        if hardware_state is None:
            return
        now_sec = self._monotonic_time_sec()
        if self._goal_start_time_sec is None:
            self._goal_start_time_sec = now_sec
        active_segment = self._active_segments[self._active_segment_index]
        if self._segment_start_time_sec is None:
            self._segment_start_time_sec = now_sec
        if getattr(active_segment, "start_time_sec", None) is None:
            active_segment.start_time_sec = now_sec

        current_positions = np.asarray(hardware_state.positions_rad, dtype=float)
        current_velocities = np.asarray(hardware_state.velocities_rad_s, dtype=float)
        segment_error_norm_rad = self._segment_error_norm_rad(
            segment=active_segment,
            current_positions=current_positions,
        )
        progress_error, progress_epsilon = self._progress_tracking_values(
            hardware_state=hardware_state,
            segment_error_norm_rad=segment_error_norm_rad,
        )
        self._initialize_segment_progress_if_needed(
            segment=active_segment,
            progress_error=progress_error,
            now_sec=now_sec,
        )
        self._record_progress(progress_error=progress_error, progress_epsilon=progress_epsilon, now_sec=now_sec)

        if self._segment_reached(active_segment, current_positions):
            if self._active_segment_index >= len(self._active_segments) - 1:
                if self._goal_target_pose_reached(hardware_state):
                    self._write_terminal_hold_command(hardware_state, context_suffix="succeeded")
                    self._result = TrajectoryExecutionResult(
                        controller_name=self._active_request.controller_name,
                        state=TrajectoryExecutionState.SUCCEEDED,
                        message="goal_reached",
                        timestamp_sec=now_sec,
                    )
                    self._clear_active_goal()
                    return
                self._write_terminal_hold_command(hardware_state, context_suffix="ee_settle")
                self._publish_hold_feedback(hardware_state=hardware_state, error_norm_rad=segment_error_norm_rad, now_sec=now_sec)
                if self._goal_has_timed_out(now_sec):
                    self._write_terminal_hold_command(hardware_state, context_suffix="timeout")
                    self._result = TrajectoryExecutionResult(
                        controller_name=self._active_request.controller_name,
                        state=TrajectoryExecutionState.ABORTED,
                        message="goal_timeout",
                        timestamp_sec=now_sec,
                    )
                    self._clear_active_goal()
                return
            self._activate_next_segment(now_sec=now_sec, current_positions=current_positions)
            return

        desired_positions, desired_velocities, alpha = self._compute_reference_command(
            segment=active_segment,
            now_sec=now_sec,
            current_positions=current_positions,
            current_velocities=current_velocities,
        )
        path_error_norm_rad = float(
            np.max(np.abs(desired_positions[: self.ARM_DOF] - np.asarray(current_positions[: self.ARM_DOF], dtype=float)))
        )

        self._hardware.write_command(
            HardwareCommandSample(
                joint_names=hardware_state.joint_names,
                positions_rad=tuple(float(value) for value in desired_positions),
                velocities_rad_s=tuple(float(value) for value in desired_velocities),
                context=f"{self._active_request.command_name}:{self._active_segment_index}",
                gripper_closed=self._active_request.gripper_closed,
            )
        )

        self._controller_state = JointTrajectoryControllerState(
            controller_name=self._active_request.controller_name,
            desired_positions_rad=tuple(float(value) for value in desired_positions),
            actual_positions_rad=tuple(float(value) for value in hardware_state.positions_rad),
            desired_velocities_rad_s=tuple(float(value) for value in desired_velocities),
            actual_velocities_rad_s=tuple(float(value) for value in hardware_state.velocities_rad_s),
            error_norm_rad=segment_error_norm_rad,
            timestamp_sec=now_sec,
        )
        self._feedback = TrajectoryExecutionFeedback(
            controller_name=self._active_request.controller_name,
            state=TrajectoryExecutionState.ACTIVE,
            desired_positions_rad=self._controller_state.desired_positions_rad,
            actual_positions_rad=self._controller_state.actual_positions_rad,
            desired_velocities_rad_s=self._controller_state.desired_velocities_rad_s,
            actual_velocities_rad_s=self._controller_state.actual_velocities_rad_s,
            error_norm_rad=segment_error_norm_rad,
            timestamp_sec=now_sec,
        )

        if self._goal_has_timed_out(now_sec):
            self._write_terminal_hold_command(hardware_state, context_suffix="timeout")
            self._result = TrajectoryExecutionResult(
                controller_name=self._active_request.controller_name,
                state=TrajectoryExecutionState.ABORTED,
                message="goal_timeout",
                timestamp_sec=now_sec,
            )
            self._clear_active_goal()
            return

        if self._path_tolerance_is_violated(now_sec=now_sec, path_error_norm_rad=path_error_norm_rad):
            self._write_terminal_hold_command(hardware_state, context_suffix="path_tolerance_violation")
            self._result = TrajectoryExecutionResult(
                controller_name=self._active_request.controller_name,
                state=TrajectoryExecutionState.ABORTED,
                message="path_tolerance_violation",
                timestamp_sec=now_sec,
            )
            self._clear_active_goal()
            return

        if alpha >= 1.0 and self._active_segment_index < len(self._active_segments) - 1 and self._segment_has_stalled(now_sec):
            self._activate_next_segment(now_sec=now_sec, current_positions=current_positions)

    def active_request(self) -> TrajectoryExecutionRequest | None:
        return self._active_request

    def current_feedback(self) -> TrajectoryExecutionFeedback | None:
        return self._feedback

    def current_result(self) -> TrajectoryExecutionResult | None:
        return self._result

    def current_controller_state(self) -> JointTrajectoryControllerState | None:
        return self._controller_state

    @property
    def active_segment_index(self) -> int:
        return self._active_segment_index

    def _clear_active_goal(self) -> None:
        self._active_request = None
        self._active_segments = ()
        self._active_segment_index = 0
        self._goal_start_time_sec = None
        self._segment_start_time_sec = None
        self._goal_best_error_norm_rad = None
        self._goal_last_progress_time_sec = None
        self._segment_best_error_norm_rad = None
        self._segment_last_progress_time_sec = None

    def _trajectory_duration_sec(self) -> float:
        return float(sum(getattr(segment, "duration_sec", 0.0) for segment in self._active_segments))

    def _compute_reference_command(
        self,
        *,
        segment: object,
        now_sec: float,
        current_positions: np.ndarray,
        current_velocities: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        reference_state, command_velocities = sample_trajectory_reference_state(
            active_segment=segment,
            current_positions=current_positions,
            now_sec=now_sec,
            time_epsilon_sec=1e-3,
            arm_joint_velocity_limits_rad_s=self._arm_joint_velocity_limits_rad_s,
        )
        return reference_state.reference_positions, command_velocities, reference_state.alpha

    @staticmethod
    def _expand_positions(current_positions: tuple[float, ...], arm_positions: tuple[float, ...]) -> np.ndarray:
        current = np.asarray(current_positions, dtype=float)
        arm = np.asarray(arm_positions, dtype=float)
        if arm.shape == current.shape:
            return arm.copy()
        expanded = current.copy()
        expanded[: arm.shape[0]] = arm
        return expanded

    def _segment_reached(self, segment: object, current_positions: np.ndarray) -> bool:
        target_positions = np.asarray(getattr(segment, "target_positions"), dtype=float)
        return joint_positions_reached(
            np.asarray(current_positions[: self.ARM_DOF], dtype=float),
            np.asarray(target_positions[: self.ARM_DOF], dtype=float),
            tolerance_rad=self._goal_tolerance_rad,
        )

    def _segment_error_norm_rad(self, *, segment: object, current_positions: np.ndarray) -> float:
        target_positions = np.asarray(getattr(segment, "target_positions"), dtype=float)
        return float(
            np.max(np.abs(np.asarray(target_positions[: self.ARM_DOF], dtype=float) - np.asarray(current_positions[: self.ARM_DOF], dtype=float)))
        )

    def _initialize_segment_progress_if_needed(
        self,
        *,
        segment: object,
        progress_error: float,
        now_sec: float,
    ) -> None:
        if getattr(segment, "initial_error_max", None) is None:
            segment.initial_error_max = progress_error
        if self._segment_best_error_norm_rad is None:
            self._segment_best_error_norm_rad = progress_error
            self._segment_last_progress_time_sec = now_sec
        if self._goal_best_error_norm_rad is None:
            self._goal_best_error_norm_rad = progress_error
            self._goal_last_progress_time_sec = now_sec

    def _record_progress(self, *, progress_error: float, progress_epsilon: float, now_sec: float) -> None:
        if (
            self._segment_best_error_norm_rad is None
            or progress_error < self._segment_best_error_norm_rad - progress_epsilon
        ):
            self._segment_best_error_norm_rad = progress_error
            self._segment_last_progress_time_sec = now_sec
        if self._goal_best_error_norm_rad is None or progress_error < self._goal_best_error_norm_rad - progress_epsilon:
            self._goal_best_error_norm_rad = progress_error
            self._goal_last_progress_time_sec = now_sec

    def _segment_has_stalled(self, now_sec: float) -> bool:
        if self._segment_last_progress_time_sec is None:
            return False
        return (now_sec - self._segment_last_progress_time_sec) >= self._stall_timeout_sec()

    def _goal_has_timed_out(self, now_sec: float) -> bool:
        if self._goal_start_time_sec is None or self._goal_last_progress_time_sec is None:
            return False
        nominal_deadline_sec = self._trajectory_duration_sec() + self._goal_time_tolerance_sec
        active_abort = self._active_abort_policy()
        if active_abort is not None and active_abort.nominal_timeout_sec is not None:
            nominal_deadline_sec = active_abort.nominal_timeout_sec
        if now_sec - self._goal_start_time_sec <= nominal_deadline_sec:
            return False
        return (now_sec - self._goal_last_progress_time_sec) >= self._stall_timeout_sec()

    def _path_tolerance_is_violated(self, *, now_sec: float, path_error_norm_rad: float) -> bool:
        if self._goal_start_time_sec is None:
            return False
        if now_sec - self._goal_start_time_sec <= self._path_tolerance_grace_sec:
            return False
        if path_error_norm_rad <= self._active_path_tolerance_rad():
            return False
        return self._segment_has_stalled(now_sec)

    def _activate_next_segment(self, *, now_sec: float, current_positions: np.ndarray) -> None:
        self._active_segment_index += 1
        self._segment_start_time_sec = now_sec
        next_segment = self._active_segments[self._active_segment_index]
        next_segment.start_time_sec = now_sec
        next_segment.initial_error_max = self._segment_error_norm_rad(
            segment=next_segment,
            current_positions=current_positions,
        )
        self._segment_best_error_norm_rad = next_segment.initial_error_max
        self._segment_last_progress_time_sec = now_sec

    def _write_terminal_hold_command(self, hardware_state: HardwareStateSample, *, context_suffix: str) -> None:
        if self._active_request is None:
            return
        hold_positions = self._terminal_hold_positions(hardware_state, context_suffix=context_suffix)
        zero_velocities = np.zeros(len(hardware_state.positions_rad), dtype=float)
        self._hardware.write_command(
            HardwareCommandSample(
                joint_names=hardware_state.joint_names,
                positions_rad=tuple(float(value) for value in hold_positions),
                velocities_rad_s=tuple(float(value) for value in zero_velocities),
                context=f"{self._active_request.command_name}:{context_suffix}",
                gripper_closed=self._active_request.gripper_closed,
            )
        )

    def _terminal_hold_positions(self, hardware_state: HardwareStateSample, *, context_suffix: str) -> np.ndarray:
        if context_suffix in {"succeeded", "ee_settle"} and self._active_segments:
            target_positions = np.asarray(
                getattr(self._active_segments[min(self._active_segment_index, len(self._active_segments) - 1)], "target_positions"),
                dtype=float,
            )
            if target_positions.shape[0] == len(hardware_state.positions_rad):
                return target_positions
        return np.asarray(hardware_state.positions_rad, dtype=float)

    def _goal_target_pose_reached(self, hardware_state: HardwareStateSample) -> bool:
        if self._active_request is None:
            return True
        spec = self._active_request.execution_phase_spec
        if spec is not None and spec.intent.success.judge is not SuccessJudge.END_EFFECTOR_POSE:
            return True
        target_pose = self._active_request.target_pose
        tolerance_m = self._active_request.position_tolerance_m
        if spec is not None and spec.intent.success.position_tolerance_m is not None:
            tolerance_m = spec.intent.success.position_tolerance_m
        current_pose = hardware_state.end_effector_pose
        if target_pose is None or tolerance_m is None or current_pose is None:
            return True
        return self._pose_distance_m(current_pose, target_pose) <= tolerance_m

    def _progress_tracking_values(
        self,
        *,
        hardware_state: HardwareStateSample,
        segment_error_norm_rad: float,
    ) -> tuple[float, float]:
        if self._active_request is None:
            return segment_error_norm_rad, self.PROGRESS_EPSILON_RAD
        spec = self._active_request.execution_phase_spec
        if (
            spec is None
            or spec.intent.success.judge is not SuccessJudge.END_EFFECTOR_POSE
            or hardware_state.end_effector_pose is None
            or spec.motion.phase_goal_pose is None
        ):
            return segment_error_norm_rad, self.PROGRESS_EPSILON_RAD
        abort_policy = spec.intent.abort
        return (
            self._pose_distance_m(hardware_state.end_effector_pose, spec.motion.phase_goal_pose),
            abort_policy.min_progress_delta_m or self.PROGRESS_EPSILON_RAD,
        )

    def _active_abort_policy(self) -> AbortPolicy | None:
        if self._active_request is None or self._active_request.execution_phase_spec is None:
            return None
        return self._active_request.execution_phase_spec.intent.abort

    def _stall_timeout_sec(self) -> float:
        active_abort = self._active_abort_policy()
        if active_abort is not None and active_abort.stall_timeout_sec is not None:
            return active_abort.stall_timeout_sec
        return self.STALL_TIMEOUT_SEC

    def _active_path_tolerance_rad(self) -> float:
        active_abort = self._active_abort_policy()
        if active_abort is not None and active_abort.joint_path_tolerance_rad is not None:
            return active_abort.joint_path_tolerance_rad
        return self._path_tolerance_rad

    def _publish_hold_feedback(
        self,
        *,
        hardware_state: HardwareStateSample,
        error_norm_rad: float,
        now_sec: float,
    ) -> None:
        if self._active_request is None:
            return
        desired_positions = tuple(float(value) for value in self._terminal_hold_positions(hardware_state, context_suffix="ee_settle"))
        desired_velocities = tuple(0.0 for _ in hardware_state.positions_rad)
        self._controller_state = JointTrajectoryControllerState(
            controller_name=self._active_request.controller_name,
            desired_positions_rad=desired_positions,
            actual_positions_rad=tuple(float(value) for value in hardware_state.positions_rad),
            desired_velocities_rad_s=desired_velocities,
            actual_velocities_rad_s=tuple(float(value) for value in hardware_state.velocities_rad_s),
            error_norm_rad=error_norm_rad,
            timestamp_sec=now_sec,
        )
        self._feedback = TrajectoryExecutionFeedback(
            controller_name=self._active_request.controller_name,
            state=TrajectoryExecutionState.ACTIVE,
            desired_positions_rad=self._controller_state.desired_positions_rad,
            actual_positions_rad=self._controller_state.actual_positions_rad,
            desired_velocities_rad_s=self._controller_state.desired_velocities_rad_s,
            actual_velocities_rad_s=self._controller_state.actual_velocities_rad_s,
            error_norm_rad=error_norm_rad,
            timestamp_sec=now_sec,
        )

    @staticmethod
    def _pose_distance_m(current_pose: Pose3D, target_pose: Pose3D) -> float:
        dx = current_pose.x - target_pose.x
        dy = current_pose.y - target_pose.y
        dz = current_pose.z - target_pose.z
        return float(np.sqrt(dx * dx + dy * dy + dz * dz))
