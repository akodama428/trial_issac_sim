from __future__ import annotations

import math
from typing import Callable

import numpy as np

from tomato_harvest_sim.api.contracts import Pose3D
from tomato_harvest_sim.robot.api.trajectory_tracking import (
    ObservationData,
    TrackingCommand,
    TrackingStepResult,
)
from tomato_harvest_sim.robot.trajectory_tracking.reference_tracking import (
    joint_positions_reached,
    step_toward_joint_positions,
)
from tomato_harvest_sim.robot.trajectory_tracking.state_store import ExecutionStateStore


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


class TrajectoryTracker:
    GRASP_TARGET_OFFSET_FROM_HAND_M = (0.0, 0.0, 0.0584)

    def __init__(
        self,
        *,
        position_tolerance_m: float,
        max_joint_step_rad: float,
        max_gripper_step_rad: float,
        joint_tolerance_rad: float,
        debug_log: Callable[[str], None] = lambda _message: None,
    ) -> None:
        self._position_tolerance_m = position_tolerance_m
        self._max_joint_step_rad = max_joint_step_rad
        self._max_gripper_step_rad = max_gripper_step_rad
        self._joint_tolerance_rad = joint_tolerance_rad
        self._debug_log = debug_log

    def prepare_tracking_state(
        self,
        *,
        store: ExecutionStateStore,
        observation: ObservationData,
        solve_joint_targets_for_waypoints: Callable[[tuple[Pose3D, ...]], tuple[np.ndarray, ...]],
    ) -> TrackingStepResult | None:
        state = store.state
        if state.target_pose is None:
            return None
        del observation
        self._sync_motion_waypoints(
            store=store,
            allow_target_fallback=True,
            solve_joint_targets_for_waypoints=solve_joint_targets_for_waypoints,
        )
        store.clear_joint_trajectory_state(clear_raw=False)
        return None

    def compute_step(
        self,
        *,
        store: ExecutionStateStore,
        observation: ObservationData,
        home_joint_positions: np.ndarray | None,
        solve_joint_targets_for_pose: Callable[[Pose3D], np.ndarray | None],
    ) -> TrackingStepResult:
        state = store.state
        current_positions = observation.joint_positions
        current_pose = observation.end_effector_pose

        if state.home_command_pending:
            return self._step_home_motion(
                store=store,
                current_positions=current_positions,
                home_joint_positions=home_joint_positions,
            )

        if state.target_pose is None:
            return self._apply_gripper_state(state=state, current_positions=current_positions)

        if state.blocked_motion_signature is not None:
            hold_result = self._hold_replan_position(state=state, current_positions=current_positions)
            if state.replan_status_announced:
                return hold_result
            state.replan_status_announced = True
            return TrackingStepResult(
                command=hold_result.command,
                log_message="[Simulator] MoveIt2 joint trajectory aborted; waiting for replanned motion command.",
            )

        if state.joint_waypoint_targets:
            return self._step_joint_waypoint_path(
                store=store,
                current_positions=current_positions,
                current_pose=current_pose,
            )

        if current_pose is not None and is_pose_reached(
            current_pose,
            state.target_pose,
            position_tolerance_m=self._position_tolerance_m,
        ):
            if state.reached_announced:
                return TrackingStepResult(reached=True)
            state.reached_announced = True
            distance_m = pose_distance_m(current_pose, state.target_pose)
            return TrackingStepResult(
                reached=True,
                log_message=(
                    "[Simulator] Franka target reached "
                    f"(ee_xyz=({current_pose.x:.4f}, {current_pose.y:.4f}, {current_pose.z:.4f}), "
                    f"error={distance_m:.4f} m)."
                ),
            )

        command = self._apply_inverse_kinematics(
            state=state,
            current_positions=current_positions,
            solve_joint_targets_for_pose=solve_joint_targets_for_pose,
        )
        if state.target_announced:
            return TrackingStepResult(command=command)
        state.target_announced = True
        return TrackingStepResult(
            command=command,
            log_message=(
                "[Simulator] Executing MoveIt2-ready target "
                f"({state.target_pose.x:.4f}, {state.target_pose.y:.4f}, {state.target_pose.z:.4f})."
            ),
        )

    def _sync_motion_waypoints(
        self,
        *,
        store: ExecutionStateStore,
        allow_target_fallback: bool,
        solve_joint_targets_for_waypoints: Callable[[tuple[Pose3D, ...]], tuple[np.ndarray, ...]],
    ) -> None:
        state = store.state
        waypoints = state.motion_waypoints
        if not waypoints and allow_target_fallback and state.target_pose is not None:
            waypoints = (state.target_pose,)
        if not waypoints:
            store.clear_waypoint_state(clear_raw=False)
            return

        active_index = state.snapshot_active_waypoint_index if state.snapshot_active_waypoint_index is not None else len(waypoints) - 1
        if waypoints != state.waypoint_signature or not state.joint_waypoint_targets:
            joint_targets = solve_joint_targets_for_waypoints(waypoints)
            if not joint_targets:
                store.clear_waypoint_state(clear_raw=False)
                return
            state.joint_waypoint_targets = joint_targets
            state.waypoint_signature = waypoints
            state.active_waypoint_index = min(active_index, len(joint_targets) - 1)
            return

        snapshot_index = min(active_index, len(state.joint_waypoint_targets) - 1)
        state.active_waypoint_index = max(state.active_waypoint_index, snapshot_index)

    def _step_home_motion(
        self,
        *,
        store: ExecutionStateStore,
        current_positions: np.ndarray | None,
        home_joint_positions: np.ndarray | None,
    ) -> TrackingStepResult:
        state = store.state
        if current_positions is None or home_joint_positions is None:
            return TrackingStepResult()

        if joint_positions_reached(
            current_positions[:7],
            home_joint_positions[:7],
            tolerance_rad=self._joint_tolerance_rad,
        ):
            state.home_command_pending = False
            if state.home_progress_announced:
                state.home_progress_announced = False
                return TrackingStepResult(log_message="[Simulator] Franka returned to the home joint pose.")
            return TrackingStepResult()

        target_positions = np.asarray(current_positions, dtype=float).copy()
        target_positions[:7] = home_joint_positions[:7]
        next_positions = step_toward_joint_positions(
            current_positions,
            target_positions,
            max_step_rad=self._max_joint_step_rad,
        )
        next_positions = self._merge_gripper_targets_into_positions(
            state=state,
            positions=next_positions,
            current_positions=current_positions,
        )
        if state.home_progress_announced:
            return TrackingStepResult(command=TrackingCommand(next_positions, context="home_step"))
        state.home_progress_announced = True
        return TrackingStepResult(
            command=TrackingCommand(next_positions, context="home_step"),
            log_message="[Simulator] Returning Franka to the home joint pose.",
        )

    def _step_joint_waypoint_path(
        self,
        *,
        store: ExecutionStateStore,
        current_positions: np.ndarray | None,
        current_pose: Pose3D | None,
    ) -> TrackingStepResult:
        state = store.state
        if not state.joint_waypoint_targets or current_positions is None:
            return TrackingStepResult()

        active_joint_target = state.joint_waypoint_targets[state.active_waypoint_index]
        if joint_positions_reached(
            current_positions[:7],
            active_joint_target[:7],
            tolerance_rad=self._joint_tolerance_rad,
        ):
            if state.active_waypoint_index < len(state.joint_waypoint_targets) - 1:
                state.active_waypoint_index += 1
                active_joint_target = state.joint_waypoint_targets[state.active_waypoint_index]
            else:
                hold_command = self._hold_arm_pose_and_apply_gripper(
                    state=state,
                    current_positions=current_positions,
                    context="waypoint_hold_gripper",
                )
                if current_pose is not None and state.target_pose is not None and is_pose_reached(
                    current_pose,
                    state.target_pose,
                    position_tolerance_m=self._position_tolerance_m,
                ):
                    if state.reached_announced:
                        return TrackingStepResult(command=hold_command, reached=True)
                    state.reached_announced = True
                    distance_m = pose_distance_m(current_pose, state.target_pose)
                    return TrackingStepResult(
                        command=hold_command,
                        reached=True,
                        log_message=(
                            "[Simulator] Franka target reached "
                            f"(ee_xyz=({current_pose.x:.4f}, {current_pose.y:.4f}, {current_pose.z:.4f}), "
                            f"error={distance_m:.4f} m)."
                        ),
                    )

        next_positions = step_toward_joint_positions(
            current_positions,
            active_joint_target,
            max_step_rad=self._max_joint_step_rad,
        )
        next_positions = self._merge_gripper_targets_into_positions(
            state=state,
            positions=next_positions,
            current_positions=current_positions,
        )
        if state.target_announced:
            return TrackingStepResult(command=TrackingCommand(next_positions, context="waypoint_step"))
        state.target_announced = True
        return TrackingStepResult(
            command=TrackingCommand(next_positions, context="waypoint_step"),
            log_message=(
                "[Simulator] Executing MoveIt2 waypoint path "
                f"({state.active_waypoint_index + 1}/{len(state.joint_waypoint_targets)}) "
                f"toward ({state.target_pose.x:.4f}, {state.target_pose.y:.4f}, {state.target_pose.z:.4f})."
            ),
        )

    def _apply_gripper_state(self, *, state, current_positions: np.ndarray | None) -> TrackingStepResult:
        if current_positions is None:
            return TrackingStepResult()
        target_positions = np.asarray(current_positions, dtype=float).copy()
        if target_positions.shape[0] < 9:
            return TrackingStepResult()
        hold = state.arm_hold_joint_positions
        if hold is not None and hold.shape[0] >= 7:
            target_positions[:7] = hold[:7]
        desired_finger_position = 0.0 if state.gripper_closed else 0.04
        finger_targets = np.array([desired_finger_position, desired_finger_position], dtype=float)
        next_fingers = step_toward_joint_positions(
            current_positions[7:9].copy(),
            finger_targets,
            max_step_rad=self._max_gripper_step_rad,
        )
        target_positions[7] = next_fingers[0]
        target_positions[8] = next_fingers[1]
        self._debug_log_gripper_step(
            current_fingers=current_positions[7:9].copy(),
            target_fingers=finger_targets,
            command_fingers=next_fingers,
        )
        return TrackingStepResult(command=TrackingCommand(target_positions, context="gripper_step"))

    def _hold_arm_pose_and_apply_gripper(
        self,
        *,
        state,
        current_positions: np.ndarray,
        context: str,
    ) -> TrackingCommand | None:
        hold_positions = self._merge_gripper_targets_into_positions(
            state=state,
            positions=np.asarray(current_positions, dtype=float).copy(),
            current_positions=current_positions,
        )
        if np.allclose(hold_positions, current_positions):
            return None
        return TrackingCommand(hold_positions, context=context)

    def _hold_trajectory_pose_and_apply_gripper(
        self,
        *,
        state,
        current_positions: np.ndarray,
        context: str,
    ) -> TrackingCommand:
        hold_positions = self._merge_gripper_targets_into_positions(
            state=state,
            positions=np.asarray(current_positions, dtype=float).copy(),
            current_positions=current_positions,
        )
        return TrackingCommand(
            positions=hold_positions,
            velocities=np.zeros_like(current_positions, dtype=float),
            context=context,
        )

    def _hold_replan_position(self, *, state, current_positions: np.ndarray | None) -> TrackingStepResult:
        if current_positions is None:
            return self._apply_gripper_state(state=state, current_positions=current_positions)
        return TrackingStepResult(
            command=self._hold_trajectory_pose_and_apply_gripper(
                state=state,
                current_positions=current_positions,
                context="trajectory_replan_hold",
            )
        )

    def _apply_inverse_kinematics(
        self,
        *,
        state,
        current_positions: np.ndarray | None,
        solve_joint_targets_for_pose: Callable[[Pose3D], np.ndarray | None],
    ) -> TrackingCommand | None:
        if state.target_pose is None or current_positions is None:
            return None
        solver_target_pose = _hand_pose_from_grasp_center_pose(
            state.target_pose,
            grasp_center_offset_from_hand_m=self.GRASP_TARGET_OFFSET_FROM_HAND_M,
        )
        joint_targets = solve_joint_targets_for_pose(solver_target_pose)
        if joint_targets is None:
            return None
        next_positions = step_toward_joint_positions(
            current_positions,
            joint_targets,
            max_step_rad=self._max_joint_step_rad,
        )
        next_positions = self._merge_gripper_targets_into_positions(
            state=state,
            positions=next_positions,
            current_positions=current_positions,
        )
        return TrackingCommand(next_positions, context="ik_step")

    def _merge_gripper_targets_into_positions(
        self,
        *,
        state,
        positions: np.ndarray,
        current_positions: np.ndarray,
    ) -> np.ndarray:
        merged_positions = np.asarray(positions, dtype=float).copy()
        if merged_positions.shape[0] < 9 or current_positions.shape[0] < 9:
            return merged_positions
        desired_finger_position = 0.0 if state.gripper_closed else 0.04
        finger_targets = np.array([desired_finger_position, desired_finger_position], dtype=float)
        next_fingers = step_toward_joint_positions(
            np.asarray(current_positions[7:9], dtype=float).copy(),
            finger_targets,
            max_step_rad=self._max_gripper_step_rad,
        )
        merged_positions[7] = next_fingers[0]
        merged_positions[8] = next_fingers[1]
        return merged_positions

    def _debug_log_gripper_step(
        self,
        *,
        current_fingers: np.ndarray,
        target_fingers: np.ndarray,
        command_fingers: np.ndarray,
    ) -> None:
        self._debug_log(
            "[Simulator][GripperDebug] "
            f"current={self._format_joint_positions(current_fingers)} "
            f"target={self._format_joint_positions(target_fingers)} "
            f"command={self._format_joint_positions(command_fingers)}"
        )

    @staticmethod
    def _format_joint_positions(values: tuple[float, ...] | np.ndarray) -> str:
        return "[" + ", ".join(f"{float(value):.4f}" for value in values) + "]"

    @staticmethod
    def _format_pose_xyz(pose: Pose3D | None) -> str:
        if pose is None:
            return "(n/a)"
        return f"({pose.x:.4f}, {pose.y:.4f}, {pose.z:.4f})"
