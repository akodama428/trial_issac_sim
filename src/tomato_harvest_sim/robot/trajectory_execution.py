from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np
import yaml

from tomato_harvest_sim.api.contracts import JointStateSnapshot, JointTrajectory, Pose3D, ScenePhase, SceneSnapshot
from tomato_harvest_sim.robot.trajectory_tracking import (
    TrajectoryReferenceState,
    TrajectorySegment,
    build_joint_trajectory_segments,
    compute_trajectory_reference_state,
    joint_positions_reached,
    step_toward_joint_positions,
)


@dataclass(frozen=True)
class FrankaMotionProgress:
    active_target: bool
    reached: bool
    distance_m: float | None


class FrankaExecutionDriverProtocol(Protocol):
    def initialize_if_needed(self) -> bool: ...

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
        positions: np.ndarray,
        velocities: np.ndarray,
        context: str,
    ) -> None: ...


def _joint_limits_path() -> Path:
    return Path(__file__).resolve().parents[0] / "moveit_config" / "joint_limits.yaml"


def _load_arm_joint_velocity_limits_rad_s() -> np.ndarray:
    try:
        payload = yaml.safe_load(_joint_limits_path().read_text(encoding="utf-8"))
    except Exception:
        return np.full(7, np.inf, dtype=float)

    limits = payload.get("joint_limits") if isinstance(payload, dict) else None
    if not isinstance(limits, dict):
        return np.full(7, np.inf, dtype=float)

    values: list[float] = []
    for joint_name in FrankaTrajectoryExecutionManager.ARM_JOINT_NAMES:
        joint_limit = limits.get(joint_name, {})
        if not isinstance(joint_limit, dict) or not joint_limit.get("has_velocity_limits", False):
            values.append(float("inf"))
            continue
        values.append(float(joint_limit.get("max_velocity", float("inf"))))
    return np.asarray(values, dtype=float)


def pose_distance_m(current_pose: Pose3D, target_pose: Pose3D) -> float:
    dx = current_pose.x - target_pose.x
    dy = current_pose.y - target_pose.y
    dz = current_pose.z - target_pose.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def is_pose_reached(
    current_pose: Pose3D,
    target_pose: Pose3D,
    *,
    position_tolerance_m: float,
) -> bool:
    return pose_distance_m(current_pose, target_pose) <= position_tolerance_m


def _hand_pose_from_grasp_center_pose(
    grasp_center_pose: Pose3D,
    *,
    grasp_center_offset_from_hand_m: tuple[float, float, float],
) -> Pose3D:
    inverse_offset_m = tuple(-value for value in grasp_center_offset_from_hand_m)
    return _shift_pose_by_local_offset(grasp_center_pose, inverse_offset_m)


def _shift_pose_by_local_offset(
    pose: Pose3D,
    local_offset_m: tuple[float, float, float],
) -> Pose3D:
    offset_x, offset_y, offset_z = _rotate_local_offset(local_offset_m, pose)
    return Pose3D(
        x=round(pose.x + offset_x, 6),
        y=round(pose.y + offset_y, 6),
        z=round(pose.z + offset_z, 6),
        roll=pose.roll,
        pitch=pose.pitch,
        yaw=pose.yaw,
    )


def _rotate_local_offset(
    local_offset_m: tuple[float, float, float],
    pose: Pose3D,
) -> tuple[float, float, float]:
    x, y, z = local_offset_m
    roll = math.radians(pose.roll)
    pitch = math.radians(pose.pitch)
    yaw = math.radians(pose.yaw)

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr

    return (
        r00 * x + r01 * y + r02 * z,
        r10 * x + r11 * y + r12 * z,
        r20 * x + r21 * y + r22 * z,
    )


class FrankaTrajectoryExecutionManager:
    """Robot-side execution manager for Franka trajectory and IK commands."""

    DEFAULT_CONTROL_DT_SEC = 1.0 / 60.0
    TRAJECTORY_TIME_EPSILON_SEC = 1e-3
    TRAJECTORY_TIMEOUT_SCALE = 2.0
    TRAJECTORY_TIMEOUT_FLOOR_SEC = 0.5
    TRAJECTORY_STALL_WINDOW_SEC = 0.5
    TRAJECTORY_STALL_MIN_IMPROVEMENT_RAD = 0.01
    TRACKING_POSITION_GAIN = 2.0
    TRACKING_VELOCITY_GAIN = 0.1
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
    USE_JOINT_TRAJECTORY_EXECUTION_ENV = "TOMATO_HARVEST_USE_JOINT_TRAJECTORY_EXECUTION"

    def __init__(
        self,
        *,
        driver: FrankaExecutionDriverProtocol,
        position_tolerance_m: float = 0.03,
        max_joint_step_rad: float = 0.05,
        max_gripper_step_rad: float = 0.01,
        joint_tolerance_rad: float = 0.03,
    ) -> None:
        self._driver = driver
        self._position_tolerance_m = position_tolerance_m
        self._max_joint_step_rad = max_joint_step_rad
        self._max_gripper_step_rad = max_gripper_step_rad
        self._joint_tolerance_rad = joint_tolerance_rad
        self._target_pose: Pose3D | None = None
        self._motion_waypoints: tuple[Pose3D, ...] = ()
        self._active_waypoint_index: int = 0
        self._joint_waypoint_targets: tuple[np.ndarray, ...] = ()
        self._waypoint_signature: tuple[Pose3D, ...] | None = None
        self._joint_trajectory: JointTrajectory | None = None
        self._joint_trajectory_targets: tuple[np.ndarray, ...] = ()
        self._joint_trajectory_segments: tuple[TrajectorySegment, ...] = ()
        self._active_trajectory_point_index: int = 0
        self._trajectory_debug_enabled = os.environ.get(self.DEBUG_TRAJECTORY_ENV, "").strip() not in {"", "0", "false", "False"}
        self._joint_trajectory_execution_enabled = (
            os.environ.get(self.USE_JOINT_TRAJECTORY_EXECUTION_ENV, "0").strip() not in {"", "0", "false", "False"}
        )
        self._last_debug_joint_positions: np.ndarray | None = None
        self._last_debug_target_positions: np.ndarray | None = None
        self._last_snapshot_cycle_id: int | None = None
        self._target_announced = False
        self._reached_announced = False
        self._home_command_pending = False
        self._home_progress_announced = False
        self._gripper_closed = False
        self._last_control_time_sec: float | None = None
        self._trajectory_progress_window_started_sec: float | None = None
        self._trajectory_progress_window_error_max: float | None = None
        self._arm_joint_velocity_limits_rad_s = _load_arm_joint_velocity_limits_rad_s()
        self._last_observed_joint_positions: np.ndarray | None = None
        self._last_observed_joint_time_sec: float | None = None
        self._rejected_joint_trajectory: JointTrajectory | None = None
        self._rejected_joint_trajectory_cycle_id: int | None = None

    def sync_with_snapshot(self, snapshot: SceneSnapshot) -> None:
        self._gripper_closed = snapshot.gripper_closed
        cycle_changed = self._last_snapshot_cycle_id != snapshot.cycle_id
        self._last_snapshot_cycle_id = snapshot.cycle_id

        if snapshot.target_tool_pose is not None:
            if self._target_pose != snapshot.target_tool_pose:
                self._target_pose = snapshot.target_tool_pose
                self._target_announced = False
                self._reached_announced = False
            if self._joint_trajectory_execution_enabled:
                self._sync_motion_waypoints(snapshot, allow_target_fallback=False)
                self._sync_joint_trajectory(snapshot)
            else:
                self._sync_motion_waypoints(snapshot)
                self._clear_joint_trajectory_state()
            self._home_command_pending = False
            return

        self._target_pose = None
        self._motion_waypoints = ()
        self._joint_waypoint_targets = ()
        self._waypoint_signature = None
        self._clear_joint_trajectory_state()
        self._target_announced = False
        self._reached_announced = False
        if cycle_changed and snapshot.phase in {ScenePhase.READY, ScenePhase.STOPPED}:
            self._home_command_pending = True
            self._home_progress_announced = False
            return
        if snapshot.phase not in {ScenePhase.READY, ScenePhase.STOPPED}:
            self._home_command_pending = False
            self._home_progress_announced = False

    def step(self) -> str | None:
        if not self._initialize_if_needed():
            return None

        if self._home_command_pending:
            return self._step_home_motion()

        if self._target_pose is None:
            self._apply_gripper_state()
            return None

        if self._joint_trajectory_segments:
            return self._step_joint_trajectory()

        if self._joint_waypoint_targets:
            return self._step_joint_waypoint_path()

        current_pose = self._get_end_effector_pose()
        if current_pose is not None and is_pose_reached(
            current_pose,
            self._target_pose,
            position_tolerance_m=self._position_tolerance_m,
        ):
            if self._reached_announced:
                return None
            self._reached_announced = True
            distance_m = pose_distance_m(current_pose, self._target_pose)
            return (
                "[Simulator] Franka target reached "
                f"(ee_xyz=({current_pose.x:.4f}, {current_pose.y:.4f}, {current_pose.z:.4f}), "
                f"error={distance_m:.4f} m)."
            )

        self._apply_inverse_kinematics(self._target_pose)
        if self._target_announced:
            return None
        self._target_announced = True
        return (
            "[Simulator] Executing MoveIt2-ready target "
            f"({self._target_pose.x:.4f}, {self._target_pose.y:.4f}, {self._target_pose.z:.4f})."
        )

    def progress(self) -> FrankaMotionProgress:
        if self._target_pose is None or not self._initialize_if_needed():
            return FrankaMotionProgress(active_target=False, reached=False, distance_m=None)

        current_pose = self._get_end_effector_pose()
        if current_pose is None:
            return FrankaMotionProgress(active_target=True, reached=False, distance_m=None)

        distance_m = pose_distance_m(current_pose, self._target_pose)
        return FrankaMotionProgress(
            active_target=True,
            reached=distance_m <= self._position_tolerance_m,
            distance_m=distance_m,
        )

    def current_end_effector_pose(self) -> Pose3D | None:
        if not self._initialize_if_needed():
            return None
        return self._get_end_effector_pose()

    def current_joint_state_snapshot(self) -> JointStateSnapshot | None:
        if not self._initialize_if_needed():
            return None
        return self._driver.current_joint_state_snapshot()

    def log_post_update_debug_snapshot(self) -> None:
        if not self._trajectory_debug_enabled:
            return
        current_positions = self._current_joint_positions()
        current_pose = self._get_end_effector_pose()
        self._debug_log(
            "[Simulator][TrajectoryDebug][post_update] "
            f"current_q={self._format_joint_positions(current_positions[:7]) if current_positions is not None else 'n/a'} "
            f"ee_xyz={self._format_pose_xyz(current_pose)} "
            f"target_xyz={self._format_pose_xyz(self._target_pose)}"
        )

    def _initialize_if_needed(self) -> bool:
        return self._driver.initialize_if_needed()

    def _apply_home_joint_positions(self) -> None:
        home_joint_positions = self._driver.home_joint_positions()
        current_positions = self._current_joint_positions()
        if current_positions is None or home_joint_positions is None:
            return
        target_positions = current_positions.copy()
        target_positions[:7] = home_joint_positions[:7]
        next_positions = step_toward_joint_positions(
            current_positions,
            target_positions,
            max_step_rad=self._max_joint_step_rad,
        )
        next_positions = self._merge_gripper_targets_into_positions(
            next_positions,
            current_positions=current_positions,
        )
        self._set_joint_positions_with_debug(next_positions, context="home_step")

    def _apply_gripper_state(self) -> None:
        current_positions = self._current_joint_positions()
        if current_positions is None:
            return
        target_positions = np.asarray(current_positions, dtype=float).reshape(-1)
        if target_positions.shape[0] < 9:
            return
        desired_finger_position = 0.0 if self._gripper_closed else 0.04
        finger_targets = np.array([desired_finger_position, desired_finger_position], dtype=float)
        next_fingers = step_toward_joint_positions(
            target_positions[7:9].copy(),
            finger_targets,
            max_step_rad=self._max_gripper_step_rad,
        )
        target_positions[7] = next_fingers[0]
        target_positions[8] = next_fingers[1]
        self._set_joint_positions_with_debug(target_positions, context="gripper_step")
        self._debug_log_gripper_step(
            current_fingers=current_positions[7:9].copy(),
            target_fingers=finger_targets,
            command_fingers=next_fingers,
        )

    def _hold_arm_pose_and_apply_gripper(self, current_positions: np.ndarray, *, context: str) -> None:
        hold_positions = np.asarray(current_positions, dtype=float).copy()
        hold_positions = self._merge_gripper_targets_into_positions(
            hold_positions,
            current_positions=current_positions,
        )
        if np.allclose(hold_positions, current_positions):
            return
        self._set_joint_positions_with_debug(hold_positions, context=context)

    def _apply_inverse_kinematics(self, target_pose: Pose3D) -> None:
        solver_target_pose = _hand_pose_from_grasp_center_pose(
            target_pose,
            grasp_center_offset_from_hand_m=self.GRASP_TARGET_OFFSET_FROM_HAND_M,
        )
        joint_targets = self._solve_joint_targets_for_pose(solver_target_pose)
        if joint_targets is None:
            return
        current_positions = self._current_joint_positions()
        if current_positions is None:
            return
        next_positions = step_toward_joint_positions(
            current_positions,
            joint_targets,
            max_step_rad=self._max_joint_step_rad,
        )
        next_positions = self._merge_gripper_targets_into_positions(
            next_positions,
            current_positions=current_positions,
        )
        self._set_joint_positions_with_debug(next_positions, context="ik_step")

    def _get_end_effector_pose(self) -> Pose3D | None:
        return self._driver.current_end_effector_pose()

    def _expand_joint_targets(self, joint_positions: np.ndarray) -> np.ndarray:
        return self._driver.expand_joint_targets(joint_positions)

    def _step_home_motion(self) -> str | None:
        current_positions = self._current_joint_positions()
        home_joint_positions = self._driver.home_joint_positions()
        if current_positions is None or home_joint_positions is None:
            return None

        if joint_positions_reached(
            current_positions[:7],
            home_joint_positions[:7],
            tolerance_rad=self._joint_tolerance_rad,
        ):
            self._home_command_pending = False
            if self._home_progress_announced:
                self._home_progress_announced = False
                return "[Simulator] Franka returned to the home joint pose."
            return None

        self._apply_home_joint_positions()
        if self._home_progress_announced:
            return None
        self._home_progress_announced = True
        return "[Simulator] Returning Franka to the home joint pose."

    def _current_joint_positions(self) -> np.ndarray | None:
        return self._driver.current_joint_positions()

    def _current_joint_velocities(self) -> np.ndarray | None:
        return self._driver.current_joint_velocities()

    def _clear_joint_trajectory_state(self) -> None:
        self._joint_trajectory = None
        self._joint_trajectory_targets = ()
        self._joint_trajectory_segments = ()
        self._active_trajectory_point_index = 0
        self._last_debug_joint_positions = None
        self._last_debug_target_positions = None
        self._last_control_time_sec = None
        self._trajectory_progress_window_started_sec = None
        self._trajectory_progress_window_error_max = None
        self._last_observed_joint_positions = None
        self._last_observed_joint_time_sec = None

    def _reset_trajectory_progress_window(self) -> None:
        self._trajectory_progress_window_started_sec = None
        self._trajectory_progress_window_error_max = None

    def _monotonic_time_sec(self) -> float:
        return time.monotonic()

    def _control_dt_sec(self, now_sec: float) -> float:
        if self._last_control_time_sec is None:
            dt_sec = self.DEFAULT_CONTROL_DT_SEC
        else:
            dt_sec = max(now_sec - self._last_control_time_sec, self.TRAJECTORY_TIME_EPSILON_SEC)
        self._last_control_time_sec = now_sec
        return dt_sec

    def _sync_motion_waypoints(self, snapshot: SceneSnapshot, *, allow_target_fallback: bool = True) -> None:
        waypoints = snapshot.motion_waypoints
        if not waypoints and allow_target_fallback and snapshot.target_tool_pose is not None:
            waypoints = (snapshot.target_tool_pose,)
        if not waypoints:
            self._motion_waypoints = ()
            self._joint_waypoint_targets = ()
            self._waypoint_signature = None
            return

        active_index = snapshot.active_waypoint_index if snapshot.active_waypoint_index is not None else len(waypoints) - 1
        if waypoints != self._waypoint_signature or not self._joint_waypoint_targets:
            joint_targets = self._solve_joint_targets_for_waypoints(waypoints)
            if not joint_targets:
                self._motion_waypoints = ()
                self._joint_waypoint_targets = ()
                self._waypoint_signature = None
                return
            self._motion_waypoints = waypoints
            self._joint_waypoint_targets = joint_targets
            self._waypoint_signature = waypoints
            self._active_waypoint_index = min(active_index, len(self._joint_waypoint_targets) - 1)
            return

        snapshot_index = min(active_index, len(self._joint_waypoint_targets) - 1)
        self._active_waypoint_index = max(self._active_waypoint_index, snapshot_index)

    def _sync_joint_trajectory(self, snapshot: SceneSnapshot) -> None:
        trajectory = snapshot.motion_joint_trajectory
        if trajectory is None or not trajectory.points:
            self._clear_joint_trajectory_state()
            return

        if (
            self._rejected_joint_trajectory == trajectory
            and self._rejected_joint_trajectory_cycle_id == snapshot.cycle_id
        ):
            self._debug_log(
                "[Simulator][TrajectoryDebug][trajectory_rejected_same_cycle] "
                f"cycle={snapshot.cycle_id} target_xyz={self._format_pose_xyz(self._target_pose)}"
            )
            self._clear_joint_trajectory_state()
            return

        if trajectory == self._joint_trajectory and self._joint_trajectory_segments:
            return

        expanded_targets = tuple(
            self._expand_joint_targets(np.asarray(point.positions_rad, dtype=float))
            for point in trajectory.points
        )
        current_positions = self._current_joint_positions()
        self._joint_trajectory = trajectory
        self._joint_trajectory_targets = expanded_targets
        self._joint_trajectory_segments, synthetic_start_logged = build_joint_trajectory_segments(
            trajectory=trajectory,
            expanded_targets=expanded_targets,
            current_positions=current_positions,
            joint_tolerance_rad=self._joint_tolerance_rad,
            time_epsilon_sec=self.TRAJECTORY_TIME_EPSILON_SEC,
            arm_joint_velocity_limits_rad_s=self._arm_joint_velocity_limits_rad_s,
        )
        self._active_trajectory_point_index = 0
        self._last_debug_joint_positions = None
        self._last_debug_target_positions = None
        self._last_control_time_sec = None
        self._reset_trajectory_progress_window()
        if synthetic_start_logged and current_positions is not None:
            self._debug_log(
                "[Simulator][TrajectoryDebug] inserted synthetic trajectory start "
                f"from current_q={self._format_joint_positions(current_positions[:7])} "
                f"to first_q={self._format_joint_positions(expanded_targets[0][:7])}."
            )
        self._debug_log_trajectory_sync(trajectory)

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

    def _solve_joint_targets_for_pose(self, target_pose: Pose3D) -> np.ndarray | None:
        return self._driver.solve_joint_targets_for_pose(
            target_pose,
            position_tolerance_m=self._position_tolerance_m,
        )

    def _step_joint_waypoint_path(self) -> str | None:
        if not self._joint_waypoint_targets:
            return None
        current_positions = self._current_joint_positions()
        if current_positions is None:
            return None

        active_joint_target = self._joint_waypoint_targets[self._active_waypoint_index]
        if joint_positions_reached(
            current_positions[:7],
            active_joint_target[:7],
            tolerance_rad=self._joint_tolerance_rad,
        ):
            if self._active_waypoint_index < len(self._joint_waypoint_targets) - 1:
                self._active_waypoint_index += 1
                active_joint_target = self._joint_waypoint_targets[self._active_waypoint_index]
            else:
                current_pose = self._get_end_effector_pose()
                self._hold_arm_pose_and_apply_gripper(current_positions, context="waypoint_hold_gripper")
                if current_pose is not None and is_pose_reached(
                    current_pose,
                    self._target_pose,
                    position_tolerance_m=self._position_tolerance_m,
                ):
                    if self._reached_announced:
                        return None
                    self._reached_announced = True
                    distance_m = pose_distance_m(current_pose, self._target_pose)
                    return (
                        "[Simulator] Franka target reached "
                        f"(ee_xyz=({current_pose.x:.4f}, {current_pose.y:.4f}, {current_pose.z:.4f}), "
                        f"error={distance_m:.4f} m)."
                    )

        next_positions = step_toward_joint_positions(
            current_positions,
            active_joint_target,
            max_step_rad=self._max_joint_step_rad,
        )
        next_positions = self._merge_gripper_targets_into_positions(
            next_positions,
            current_positions=current_positions,
        )
        self._set_joint_positions_with_debug(next_positions, context="waypoint_step")
        if self._target_announced:
            return None
        self._target_announced = True
        return (
            "[Simulator] Executing MoveIt2 waypoint path "
            f"({self._active_waypoint_index + 1}/{len(self._joint_waypoint_targets)}) "
            f"toward ({self._target_pose.x:.4f}, {self._target_pose.y:.4f}, {self._target_pose.z:.4f})."
        )

    def _step_joint_trajectory(self) -> str | None:
        if not self._joint_trajectory_segments:
            return None
        current_positions = self._current_joint_positions()
        if current_positions is None:
            return None
        current_pose = self._get_end_effector_pose()
        current_error_m = None
        if current_pose is not None and self._target_pose is not None:
            current_error_m = pose_distance_m(current_pose, self._target_pose)

        active_segment = self._joint_trajectory_segments[self._active_trajectory_point_index]
        active_joint_target = active_segment.target_positions

        while joint_positions_reached(
            current_positions[:7],
            active_joint_target[:7],
            tolerance_rad=self._joint_tolerance_rad,
        ):
            if self._active_trajectory_point_index < len(self._joint_trajectory_targets) - 1:
                self._active_trajectory_point_index += 1
                self._reset_trajectory_progress_window()
                active_segment = self._joint_trajectory_segments[self._active_trajectory_point_index]
                active_joint_target = active_segment.target_positions
                self._debug_log(
                    "[Simulator][TrajectoryDebug] advanced to next trajectory point "
                    f"{self._active_trajectory_point_index + 1}/{len(self._joint_trajectory_targets)}."
                )
            else:
                self._debug_log(
                    "[Simulator][TrajectoryDebug] final trajectory joint target reached. "
                    f"ee_error={self._format_optional_float(current_error_m)} m."
                )
                self._hold_trajectory_pose_and_apply_gripper(current_positions, context="trajectory_hold_gripper")
                if self._reached_announced:
                    return None
                self._reached_announced = True
                if current_pose is None or self._target_pose is None:
                    return "[Simulator] Franka trajectory completed."
                return (
                    "[Simulator] Franka trajectory completed "
                    f"(ee_xyz=({current_pose.x:.4f}, {current_pose.y:.4f}, {current_pose.z:.4f}), "
                    f"error={current_error_m:.4f} m)."
                )

        now_sec = self._monotonic_time_sec()
        control_dt_sec = self._control_dt_sec(now_sec)
        joint_error_max = float(np.max(np.abs(active_joint_target[:7] - current_positions[:7])))
        self._initialize_active_trajectory_segment(
            active_segment=active_segment,
            now_sec=now_sec,
            joint_error_max=joint_error_max,
        )
        fallback_reason = self._trajectory_fallback_reason(
            active_segment=active_segment,
            joint_error_max=joint_error_max,
            now_sec=now_sec,
        )
        if fallback_reason is not None:
            return self._fallback_from_joint_trajectory(reason=fallback_reason, current_positions=current_positions)

        current_velocities = self._estimate_current_joint_velocities(
            current_positions=current_positions,
            now_sec=now_sec,
            driver_velocities=self._current_joint_velocities(),
        )
        reference_state, command_velocities = self._compute_joint_velocity_command(
            current_positions=current_positions,
            current_velocities=current_velocities,
            active_segment=active_segment,
            now_sec=now_sec,
        )
        next_positions = self._integrate_joint_velocity_command(
            current_positions=current_positions,
            command_velocities=command_velocities,
            control_dt_sec=control_dt_sec,
            active_joint_target=active_joint_target,
        )
        next_positions = self._merge_gripper_targets_into_positions(
            next_positions,
            current_positions=current_positions,
        )
        self._debug_log_trajectory_step(
            current_positions=current_positions,
            active_joint_target=active_joint_target,
            next_positions=next_positions,
            command_velocities=command_velocities,
            remaining_time_sec=max(active_segment.duration_sec - reference_state.elapsed_time_sec, control_dt_sec),
            joint_error_max=joint_error_max,
            current_pose=current_pose,
            current_error_m=current_error_m,
            reference_state=reference_state,
            current_velocities=current_velocities,
        )
        self._set_joint_velocity_targets_with_debug(
            positions=next_positions,
            velocities=command_velocities,
            context="trajectory_step",
        )
        if self._target_announced:
            return None
        self._target_announced = True
        return (
            "[Simulator] Executing MoveIt2 joint trajectory "
            f"({self._active_trajectory_point_index + 1}/{len(self._joint_trajectory_targets)}) "
            f"toward ({self._target_pose.x:.4f}, {self._target_pose.y:.4f}, {self._target_pose.z:.4f})."
        )

    def _initialize_active_trajectory_segment(
        self,
        *,
        active_segment: TrajectorySegment,
        now_sec: float,
        joint_error_max: float,
    ) -> None:
        if active_segment.start_time_sec is None:
            active_segment.start_time_sec = now_sec
            active_segment.deadline_sec = now_sec + max(
                active_segment.duration_sec * self.TRAJECTORY_TIMEOUT_SCALE,
                self.TRAJECTORY_TIMEOUT_FLOOR_SEC,
            )
            active_segment.initial_error_max = joint_error_max
            self._trajectory_progress_window_started_sec = now_sec
            self._trajectory_progress_window_error_max = joint_error_max

    def _trajectory_fallback_reason(
        self,
        *,
        active_segment: TrajectorySegment,
        joint_error_max: float,
        now_sec: float,
    ) -> str | None:
        if active_segment.deadline_sec is not None and now_sec >= active_segment.deadline_sec:
            return (
                "segment_timeout "
                f"index={self._active_trajectory_point_index + 1} "
                f"duration={active_segment.duration_sec:.4f}s "
                f"joint_error_max={joint_error_max:.4f}"
            )

        window_started_sec = self._trajectory_progress_window_started_sec
        window_error_max = self._trajectory_progress_window_error_max
        if window_started_sec is None or window_error_max is None:
            self._trajectory_progress_window_started_sec = now_sec
            self._trajectory_progress_window_error_max = joint_error_max
            return None

        if now_sec - window_started_sec < self.TRAJECTORY_STALL_WINDOW_SEC:
            return None

        improvement = window_error_max - joint_error_max
        self._trajectory_progress_window_started_sec = now_sec
        self._trajectory_progress_window_error_max = joint_error_max
        if improvement < self.TRAJECTORY_STALL_MIN_IMPROVEMENT_RAD:
            return (
                "segment_stall "
                f"index={self._active_trajectory_point_index + 1} "
                f"improvement={improvement:.4f} "
                f"joint_error_max={joint_error_max:.4f}"
            )
        return None

    def _estimate_current_joint_velocities(
        self,
        *,
        current_positions: np.ndarray,
        now_sec: float,
        driver_velocities: np.ndarray | None,
    ) -> np.ndarray | None:
        if driver_velocities is not None:
            self._last_observed_joint_positions = np.asarray(current_positions, dtype=float).copy()
            self._last_observed_joint_time_sec = now_sec
            return np.asarray(driver_velocities, dtype=float).reshape(-1)

        if self._last_observed_joint_positions is not None and self._last_observed_joint_time_sec is not None:
            dt_sec = max(now_sec - self._last_observed_joint_time_sec, self.TRAJECTORY_TIME_EPSILON_SEC)
            estimated_velocities = (
                np.asarray(current_positions, dtype=float) - np.asarray(self._last_observed_joint_positions, dtype=float)
            ) / dt_sec
        else:
            estimated_velocities = None
        self._last_observed_joint_positions = np.asarray(current_positions, dtype=float).copy()
        self._last_observed_joint_time_sec = now_sec
        return estimated_velocities

    def _compute_joint_velocity_command(
        self,
        *,
        current_positions: np.ndarray,
        current_velocities: np.ndarray | None,
        active_segment: TrajectorySegment,
        now_sec: float,
    ) -> tuple[TrajectoryReferenceState, np.ndarray]:
        return compute_trajectory_reference_state(
            active_segment=active_segment,
            current_positions=current_positions,
            current_velocities=current_velocities,
            now_sec=now_sec,
            time_epsilon_sec=self.TRAJECTORY_TIME_EPSILON_SEC,
            arm_joint_velocity_limits_rad_s=self._arm_joint_velocity_limits_rad_s,
            proportional_gain=self.TRACKING_POSITION_GAIN,
            derivative_gain=self.TRACKING_VELOCITY_GAIN,
        )

    def _integrate_joint_velocity_command(
        self,
        *,
        current_positions: np.ndarray,
        command_velocities: np.ndarray,
        control_dt_sec: float,
        active_joint_target: np.ndarray,
    ) -> np.ndarray:
        next_positions = np.asarray(current_positions, dtype=float).copy()
        arm_limit = min(7, next_positions.shape[0], active_joint_target.shape[0], command_velocities.shape[0])
        next_positions[:arm_limit] = next_positions[:arm_limit] + command_velocities[:arm_limit] * control_dt_sec
        target_arm = np.asarray(active_joint_target[:arm_limit], dtype=float)
        lower = np.minimum(np.asarray(current_positions[:arm_limit], dtype=float), target_arm)
        upper = np.maximum(np.asarray(current_positions[:arm_limit], dtype=float), target_arm)
        next_positions[:arm_limit] = np.clip(next_positions[:arm_limit], lower, upper)
        return next_positions

    def _hold_trajectory_pose_and_apply_gripper(self, current_positions: np.ndarray, *, context: str) -> None:
        hold_positions = np.asarray(current_positions, dtype=float).copy()
        hold_positions = self._merge_gripper_targets_into_positions(
            hold_positions,
            current_positions=current_positions,
        )
        self._set_joint_velocity_targets_with_debug(
            positions=hold_positions,
            velocities=np.zeros_like(current_positions, dtype=float),
            context=context,
        )

    def _fallback_from_joint_trajectory(self, *, reason: str, current_positions: np.ndarray) -> str | None:
        self._debug_log(
            "[Simulator][TrajectoryDebug][fallback] "
            f"reason={reason} "
            f"segment={self._active_trajectory_point_index + 1}/{len(self._joint_trajectory_segments)} "
            f"target_xyz={self._format_pose_xyz(self._target_pose)}"
        )
        self._hold_trajectory_pose_and_apply_gripper(current_positions, context="trajectory_fallback_hold")
        self._rejected_joint_trajectory = self._joint_trajectory
        self._rejected_joint_trajectory_cycle_id = self._last_snapshot_cycle_id
        self._clear_joint_trajectory_state()
        if self._joint_waypoint_targets:
            self._debug_log("[Simulator][TrajectoryDebug][fallback] started waypoint IK execution.")
            self._step_joint_waypoint_path()
            return "[Simulator] MoveIt2 joint trajectory stalled; falling back to waypoint IK execution."

        if self._target_pose is None:
            return None
        self._debug_log("[Simulator][TrajectoryDebug][fallback] started direct IK execution.")
        self._apply_inverse_kinematics(self._target_pose)
        return "[Simulator] MoveIt2 joint trajectory stalled; falling back to direct IK execution."

    def _debug_log_trajectory_sync(self, trajectory: JointTrajectory) -> None:
        if not self._trajectory_debug_enabled:
            return
        first = trajectory.points[0].positions_rad
        last = trajectory.points[-1].positions_rad
        self._debug_log(
            "[Simulator][TrajectoryDebug] synced MoveIt trajectory "
            f"points={len(trajectory.points)} joints={trajectory.joint_names} "
            f"first_q={self._format_joint_positions(first)} last_q={self._format_joint_positions(last)} "
            f"target_xyz={self._format_pose_xyz(self._target_pose)}."
        )

    def _debug_log_trajectory_step(
        self,
        *,
        current_positions: np.ndarray,
        active_joint_target: np.ndarray,
        next_positions: np.ndarray,
        command_velocities: np.ndarray,
        remaining_time_sec: float,
        joint_error_max: float,
        current_pose: Pose3D | None,
        current_error_m: float | None,
        reference_state: TrajectoryReferenceState,
        current_velocities: np.ndarray | None,
    ) -> None:
        if not self._trajectory_debug_enabled:
            return
        current_arm = np.asarray(current_positions[:7], dtype=float)
        target_arm = np.asarray(active_joint_target[:7], dtype=float)
        reference_arm = np.asarray(reference_state.reference_positions[:7], dtype=float)
        reference_qdot = np.asarray(reference_state.reference_velocities[:7], dtype=float)
        next_arm = np.asarray(next_positions[:7], dtype=float)
        joint_command_delta_max = float(np.max(np.abs(next_arm - current_arm)))
        command_qdot_max = float(np.max(np.abs(command_velocities[:7]))) if command_velocities.shape[0] >= 7 else 0.0
        observed_delta_max = None
        if self._last_debug_joint_positions is not None:
            observed_delta_max = float(np.max(np.abs(current_arm - self._last_debug_joint_positions[:7])))
        self._last_debug_joint_positions = np.asarray(current_positions, dtype=float).copy()
        self._last_debug_target_positions = np.asarray(active_joint_target, dtype=float).copy()
        current_qdot = np.zeros(7, dtype=float) if current_velocities is None else np.asarray(current_velocities[:7], dtype=float)
        self._debug_log(
            "[Simulator][TrajectoryDebug] "
            f"segment={self._active_trajectory_point_index + 1}/{len(self._joint_trajectory_segments)} "
            f"point={self._active_trajectory_point_index + 1}/{len(self._joint_trajectory_targets)} "
            f"current_q={self._format_joint_positions(current_arm)} "
            f"reference_q={self._format_joint_positions(reference_arm)} "
            f"target_q={self._format_joint_positions(target_arm)} "
            f"command_q={self._format_joint_positions(next_arm)} "
            f"current_qdot={self._format_joint_positions(current_qdot)} "
            f"reference_qdot={self._format_joint_positions(reference_qdot)} "
            f"qdot_cmd={self._format_joint_positions(command_velocities[:7]) if command_velocities.shape[0] >= 7 else '[]'} "
            f"joint_error_max={joint_error_max:.4f} "
            f"command_delta_max={joint_command_delta_max:.4f} "
            f"qdot_max={command_qdot_max:.4f} "
            f"remaining_time_sec={remaining_time_sec:.4f} "
            f"traj_alpha={reference_state.alpha:.4f} "
            f"observed_delta_max={self._format_optional_float(observed_delta_max)} "
            f"ee_xyz={self._format_pose_xyz(current_pose)} "
            f"target_xyz={self._format_pose_xyz(self._target_pose)} "
            f"ee_error={self._format_optional_float(current_error_m)}"
        )

    def _debug_log(self, message: str) -> None:
        if self._trajectory_debug_enabled:
            print(message, flush=True)

    @staticmethod
    def _warn_log(message: str) -> None:
        print(message, flush=True)

    def _debug_log_gripper_step(
        self,
        *,
        current_fingers: np.ndarray,
        target_fingers: np.ndarray,
        command_fingers: np.ndarray,
    ) -> None:
        if not self._trajectory_debug_enabled:
            return
        readback = self._current_joint_positions()
        readback_fingers = "n/a"
        if readback is not None and readback.shape[0] >= 9:
            readback_fingers = self._format_joint_positions(readback[7:9])
        self._debug_log(
            "[Simulator][GripperDebug] "
            f"closed={self._gripper_closed} "
            f"current={self._format_joint_positions(current_fingers)} "
            f"target={self._format_joint_positions(target_fingers)} "
            f"command={self._format_joint_positions(command_fingers)} "
            f"readback={readback_fingers}"
        )

    def _set_joint_positions_with_debug(self, positions: np.ndarray, *, context: str) -> None:
        self._driver.set_joint_positions_with_debug(positions, context=context)

    def _set_joint_velocity_targets_with_debug(
        self,
        *,
        positions: np.ndarray,
        velocities: np.ndarray,
        context: str,
    ) -> None:
        self._driver.set_joint_velocity_targets_with_debug(
            positions=positions,
            velocities=velocities,
            context=context,
        )

    def _merge_gripper_targets_into_positions(
        self,
        positions: np.ndarray,
        *,
        current_positions: np.ndarray,
    ) -> np.ndarray:
        merged_positions = np.asarray(positions, dtype=float).copy()
        if merged_positions.shape[0] < 9 or current_positions.shape[0] < 9:
            return merged_positions
        desired_finger_position = 0.0 if self._gripper_closed else 0.04
        finger_targets = np.array([desired_finger_position, desired_finger_position], dtype=float)
        next_fingers = step_toward_joint_positions(
            np.asarray(current_positions[7:9], dtype=float).copy(),
            finger_targets,
            max_step_rad=self._max_gripper_step_rad,
        )
        merged_positions[7] = next_fingers[0]
        merged_positions[8] = next_fingers[1]
        return merged_positions

    @staticmethod
    def _format_joint_positions(values: tuple[float, ...] | np.ndarray) -> str:
        return "[" + ", ".join(f"{float(value):.4f}" for value in values) + "]"

    @staticmethod
    def _format_pose_xyz(pose: Pose3D | None) -> str:
        if pose is None:
            return "(n/a)"
        return f"({pose.x:.4f}, {pose.y:.4f}, {pose.z:.4f})"

    @staticmethod
    def _format_optional_float(value: float | None) -> str:
        if value is None:
            return "n/a"
        return f"{value:.4f}"
