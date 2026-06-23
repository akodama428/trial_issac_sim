from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from tomato_harvest_sim.api.contracts import JointStateSnapshot, JointTrajectory, Pose3D, ScenePhase, SceneSnapshot

if TYPE_CHECKING:
    import numpy as np


@dataclass(frozen=True)
class FrankaMotionProgress:
    active_target: bool
    reached: bool
    distance_m: float | None


def step_toward_joint_positions(
    current_positions: np.ndarray,
    target_positions: np.ndarray,
    *,
    max_step_rad: float,
) -> np.ndarray:
    limited_delta = np.clip(target_positions - current_positions, -max_step_rad, max_step_rad)
    next_positions = current_positions + limited_delta
    close_mask = np.abs(target_positions - current_positions) <= max_step_rad
    next_positions[close_mask] = target_positions[close_mask]
    return next_positions


def joint_positions_reached(
    current_positions: np.ndarray,
    target_positions: np.ndarray,
    *,
    tolerance_rad: float,
) -> bool:
    return float(np.max(np.abs(target_positions - current_positions))) <= tolerance_rad


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


class IsaacFrankaMotionExecutor:
    """Moves the actual Franka articulation and gripper toward the latest scene command."""

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
        robot_prim_path: str,
        position_tolerance_m: float = 0.03,
        max_joint_step_rad: float = 0.05,
        max_gripper_step_rad: float = 0.01,
        joint_tolerance_rad: float = 0.01,
    ) -> None:
        self._robot_prim_path = robot_prim_path
        self._position_tolerance_m = position_tolerance_m
        self._max_joint_step_rad = max_joint_step_rad
        self._max_gripper_step_rad = max_gripper_step_rad
        self._joint_tolerance_rad = joint_tolerance_rad
        self._initialized = False
        self._target_pose: Pose3D | None = None
        self._motion_waypoints: tuple[Pose3D, ...] = ()
        self._active_waypoint_index: int = 0
        self._joint_waypoint_targets: tuple[np.ndarray, ...] = ()
        self._waypoint_signature: tuple[Pose3D, ...] | None = None
        self._joint_trajectory: JointTrajectory | None = None
        self._joint_trajectory_targets: tuple[np.ndarray, ...] = ()
        self._active_trajectory_point_index: int = 0
        self._trajectory_debug_enabled = os.environ.get(self.DEBUG_TRAJECTORY_ENV, "").strip() not in {"", "0", "false", "False"}
        self._last_debug_joint_positions: np.ndarray | None = None
        self._last_debug_target_positions: np.ndarray | None = None
        self._last_snapshot_cycle_id: int | None = None
        self._target_announced = False
        self._reached_announced = False
        self._home_command_pending = False
        self._home_progress_announced = False
        self._gripper_closed = False
        self._home_joint_positions: np.ndarray | None = None
        self._articulation = None
        self._articulation_kinematics_solver = None
        self._kinematics_solver = None

    def sync_with_snapshot(self, snapshot: SceneSnapshot) -> None:
        self._gripper_closed = snapshot.gripper_closed
        cycle_changed = self._last_snapshot_cycle_id != snapshot.cycle_id
        self._last_snapshot_cycle_id = snapshot.cycle_id

        if snapshot.target_tool_pose is not None:
            if self._target_pose != snapshot.target_tool_pose:
                self._target_pose = snapshot.target_tool_pose
                self._target_announced = False
                self._reached_announced = False
            self._sync_joint_trajectory(snapshot)
            if self._joint_trajectory_targets:
                self._motion_waypoints = ()
                self._joint_waypoint_targets = ()
                self._waypoint_signature = None
            else:
                self._sync_motion_waypoints(snapshot)
            self._home_command_pending = False
            return

        self._target_pose = None
        self._motion_waypoints = ()
        self._joint_waypoint_targets = ()
        self._waypoint_signature = None
        self._joint_trajectory = None
        self._joint_trajectory_targets = ()
        self._active_trajectory_point_index = 0
        self._last_debug_joint_positions = None
        self._last_debug_target_positions = None
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

        self._apply_gripper_state()

        if self._target_pose is None:
            return None

        if self._joint_trajectory_targets:
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
        if self._target_pose is None or not self._initialized:
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
        current_positions = self._current_joint_positions()
        if current_positions is None or current_positions.shape[0] < 7:
            return None
        return JointStateSnapshot(
            joint_names=self.ARM_JOINT_NAMES,
            positions_rad=tuple(float(value) for value in current_positions[:7]),
        )

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
        if self._initialized:
            return True

        try:
            self._do_initialize()
        except Exception as exc:
            print(f"[Simulator] Franka executor initialization is pending: {exc}", flush=True)
            return False

        self._initialized = True
        return True

    def _do_initialize(self) -> None:
        import numpy as np
        import omni.kit.app
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.extensions import get_extension_path_from_name
        from isaacsim.robot_motion.motion_generation import (
            ArticulationKinematicsSolver,
            LulaKinematicsSolver,
        )

        extension_manager = omni.kit.app.get_app().get_extension_manager()
        extension_manager.set_extension_enabled_immediate("isaacsim.robot_motion.motion_generation", True)

        self._articulation = SingleArticulation(self._robot_prim_path)
        self._articulation.initialize()
        joint_positions = self._articulation.get_joint_positions()
        if joint_positions is None:
            raise RuntimeError("joint positions are not available yet")

        motion_generation_path = Path(get_extension_path_from_name("isaacsim.robot_motion.motion_generation"))
        config_root = motion_generation_path / "motion_policy_configs" / "franka"
        self._kinematics_solver = LulaKinematicsSolver(
            robot_description_path=str(config_root / "rmpflow" / "robot_descriptor.yaml"),
            urdf_path=str(config_root / "lula_franka_gen.urdf"),
        )
        self._articulation_kinematics_solver = ArticulationKinematicsSolver(
            self._articulation,
            self._kinematics_solver,
            "panda_hand",
        )
        self._home_joint_positions = np.array(joint_positions, dtype=float)

    def _apply_home_joint_positions(self) -> None:
        if self._articulation is None or self._home_joint_positions is None:
            return
        current_positions = self._current_joint_positions()
        if current_positions is None:
            return
        target_positions = current_positions.copy()
        target_positions[:7] = self._home_joint_positions[:7]
        next_positions = step_toward_joint_positions(
            current_positions,
            target_positions,
            max_step_rad=self._max_joint_step_rad,
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

    def _apply_inverse_kinematics(self, target_pose: Pose3D) -> None:
        solver_target_pose = _hand_pose_from_grasp_center_pose(
            target_pose,
            grasp_center_offset_from_hand_m=self.GRASP_TARGET_OFFSET_FROM_HAND_M,
        )
        joint_targets = self._solve_joint_targets_for_pose(solver_target_pose)
        if joint_targets is None or self._articulation is None:
            return
        current_positions = self._current_joint_positions()
        if current_positions is None:
            return
        next_positions = step_toward_joint_positions(
            current_positions,
            joint_targets,
            max_step_rad=self._max_joint_step_rad,
        )
        self._set_joint_positions_with_debug(next_positions, context="ik_step")

    def _get_end_effector_pose(self) -> Pose3D | None:
        if self._articulation_kinematics_solver is None:
            return None
        end_effector_position, _ = self._articulation_kinematics_solver.compute_end_effector_pose(position_only=True)
        if end_effector_position is None:
            return None
        hand_pose = Pose3D(
            x=float(end_effector_position[0]),
            y=float(end_effector_position[1]),
            z=float(end_effector_position[2]),
            roll=180.0,
            pitch=0.0,
            yaw=0.0,
        )
        return _shift_pose_by_local_offset(hand_pose, self.GRASP_TARGET_OFFSET_FROM_HAND_M)

    def _expand_joint_targets(self, joint_positions: np.ndarray) -> np.ndarray:
        if self._articulation is None:
            return joint_positions

        flat_targets = joint_positions.reshape(-1)
        current_positions = self._articulation.get_joint_positions()
        if current_positions is None:
            return flat_targets
        current_flat = np.asarray(current_positions, dtype=float).reshape(-1)
        if flat_targets.shape == current_flat.shape:
            return flat_targets
        if flat_targets.shape[0] == 7 and current_flat.shape[0] >= 9:
            merged = current_flat.copy()
            merged[:7] = flat_targets
            return merged
        return flat_targets

    def _step_home_motion(self) -> str | None:
        current_positions = self._current_joint_positions()
        if current_positions is None or self._home_joint_positions is None:
            return None

        self._apply_gripper_state()
        if joint_positions_reached(
            current_positions[:7],
            self._home_joint_positions[:7],
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
        if self._articulation is None:
            return None
        current_positions = self._articulation.get_joint_positions()
        if current_positions is None:
            return None
        return np.asarray(current_positions, dtype=float).reshape(-1)

    def _sync_motion_waypoints(self, snapshot: SceneSnapshot) -> None:
        waypoints = snapshot.motion_waypoints or ((snapshot.target_tool_pose,) if snapshot.target_tool_pose is not None else ())
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
            self._joint_trajectory = None
            self._joint_trajectory_targets = ()
            self._active_trajectory_point_index = 0
            self._last_debug_joint_positions = None
            self._last_debug_target_positions = None
            return

        if trajectory == self._joint_trajectory and self._joint_trajectory_targets:
            return

        expanded_targets: list[np.ndarray] = []
        for point in trajectory.points:
            expanded_targets.append(self._expand_joint_targets(np.asarray(point.positions_rad, dtype=float)))
        self._joint_trajectory = trajectory
        self._joint_trajectory_targets = tuple(expanded_targets)
        self._active_trajectory_point_index = 0
        self._last_debug_joint_positions = None
        self._last_debug_target_positions = None
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
        import numpy as np
        from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats

        if self._articulation is None or self._articulation_kinematics_solver is None or self._kinematics_solver is None:
            return None

        robot_base_translation, robot_base_orientation = self._articulation.get_world_pose()
        self._kinematics_solver.set_robot_base_pose(robot_base_translation, robot_base_orientation)

        target_position = np.array((target_pose.x, target_pose.y, target_pose.z), dtype=float)
        target_orientation = euler_angles_to_quats(
            np.radians((target_pose.roll, target_pose.pitch, target_pose.yaw))
        )
        action, success = self._articulation_kinematics_solver.compute_inverse_kinematics(
            target_position,
            target_orientation,
            position_tolerance=self._position_tolerance_m,
        )
        if not success:
            return None
        target_joint_positions = getattr(action, "joint_positions", None)
        if target_joint_positions is not None:
            return self._expand_joint_targets(np.asarray(target_joint_positions, dtype=float))
        return None

    def _step_joint_waypoint_path(self) -> str | None:
        if self._articulation is None or not self._joint_waypoint_targets:
            return None
        current_positions = self._current_joint_positions()
        if current_positions is None:
            return None

        active_joint_target = self._joint_waypoint_targets[self._active_waypoint_index]
        if joint_positions_reached(
            current_positions,
            active_joint_target,
            tolerance_rad=self._joint_tolerance_rad,
        ):
            if self._active_waypoint_index < len(self._joint_waypoint_targets) - 1:
                self._active_waypoint_index += 1
                active_joint_target = self._joint_waypoint_targets[self._active_waypoint_index]
            else:
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

        next_positions = step_toward_joint_positions(
            current_positions,
            active_joint_target,
            max_step_rad=self._max_joint_step_rad,
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
        if self._articulation is None or not self._joint_trajectory_targets:
            return None
        current_positions = self._current_joint_positions()
        if current_positions is None:
            return None
        current_pose = self._get_end_effector_pose()
        current_error_m = None
        if current_pose is not None and self._target_pose is not None:
            current_error_m = pose_distance_m(current_pose, self._target_pose)

        active_joint_target = self._joint_trajectory_targets[self._active_trajectory_point_index]
        if joint_positions_reached(
            current_positions,
            active_joint_target,
            tolerance_rad=self._joint_tolerance_rad,
        ):
            if self._active_trajectory_point_index < len(self._joint_trajectory_targets) - 1:
                self._active_trajectory_point_index += 1
                active_joint_target = self._joint_trajectory_targets[self._active_trajectory_point_index]
                self._debug_log(
                    "[Simulator][TrajectoryDebug] advanced to next trajectory point "
                    f"{self._active_trajectory_point_index + 1}/{len(self._joint_trajectory_targets)}."
                )
            else:
                self._debug_log(
                    "[Simulator][TrajectoryDebug] final trajectory joint target reached. "
                    f"ee_error={self._format_optional_float(current_error_m)} m."
                )
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

        next_positions = step_toward_joint_positions(
            current_positions,
            active_joint_target,
            max_step_rad=self._max_joint_step_rad,
        )
        self._debug_log_trajectory_step(
            current_positions=current_positions,
            active_joint_target=active_joint_target,
            next_positions=next_positions,
            current_pose=current_pose,
            current_error_m=current_error_m,
        )
        self._set_joint_positions_with_debug(next_positions, context="trajectory_step")
        if self._target_announced:
            return None
        self._target_announced = True
        return (
            "[Simulator] Executing MoveIt2 joint trajectory "
            f"({self._active_trajectory_point_index + 1}/{len(self._joint_trajectory_targets)}) "
            f"toward ({self._target_pose.x:.4f}, {self._target_pose.y:.4f}, {self._target_pose.z:.4f})."
        )

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
        current_pose: Pose3D | None,
        current_error_m: float | None,
    ) -> None:
        if not self._trajectory_debug_enabled:
            return
        current_arm = np.asarray(current_positions[:7], dtype=float)
        target_arm = np.asarray(active_joint_target[:7], dtype=float)
        next_arm = np.asarray(next_positions[:7], dtype=float)
        joint_error_max = float(np.max(np.abs(target_arm - current_arm)))
        joint_command_delta_max = float(np.max(np.abs(next_arm - current_arm)))
        observed_delta_max = None
        if self._last_debug_joint_positions is not None:
            observed_delta_max = float(np.max(np.abs(current_arm - self._last_debug_joint_positions[:7])))
        self._last_debug_joint_positions = np.asarray(current_positions, dtype=float).copy()
        self._last_debug_target_positions = np.asarray(active_joint_target, dtype=float).copy()
        self._debug_log(
            "[Simulator][TrajectoryDebug] "
            f"point={self._active_trajectory_point_index + 1}/{len(self._joint_trajectory_targets)} "
            f"current_q={self._format_joint_positions(current_arm)} "
            f"target_q={self._format_joint_positions(target_arm)} "
            f"command_q={self._format_joint_positions(next_arm)} "
            f"joint_error_max={joint_error_max:.4f} "
            f"command_delta_max={joint_command_delta_max:.4f} "
            f"observed_delta_max={self._format_optional_float(observed_delta_max)} "
            f"ee_xyz={self._format_pose_xyz(current_pose)} "
            f"target_xyz={self._format_pose_xyz(self._target_pose)} "
            f"ee_error={self._format_optional_float(current_error_m)}"
        )

    def _debug_log(self, message: str) -> None:
        if self._trajectory_debug_enabled:
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
        if self._articulation is None:
            return
        self._articulation.set_joint_positions(positions)
        if not self._trajectory_debug_enabled:
            return
        readback = self._current_joint_positions()
        self._debug_log(
            "[Simulator][TrajectoryDebug][set_joint_positions] "
            f"context={context} "
            f"command_q={self._format_joint_positions(positions[:7])} "
            f"readback_q={self._format_joint_positions(readback[:7]) if readback is not None else 'n/a'}"
        )

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


IsaacFrankaPreGraspExecutor = IsaacFrankaMotionExecutor
