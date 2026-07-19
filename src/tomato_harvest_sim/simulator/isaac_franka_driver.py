from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from tomato_harvest_sim.msg.contracts import JointStateSnapshot, JointTrajectory, Pose3D
from tomato_harvest_sim.robot.msg.trajectory_tracking import ObservationData


@dataclass(frozen=True)
class _FallbackArticulationAction:
    joint_positions: np.ndarray | None = None
    joint_velocities: np.ndarray | None = None
    joint_efforts: np.ndarray | None = None
    joint_indices: np.ndarray | None = None


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


class IsaacFrankaDriver:
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

    def __init__(
        self,
        *,
        robot_prim_path: str,
        trajectory_debug_enabled: bool = False,
        arm_command_mode: str = "position_velocity",
    ) -> None:
        if arm_command_mode not in {"position_velocity", "effort"}:
            raise ValueError(f"unsupported arm command mode: {arm_command_mode}")
        self._robot_prim_path = robot_prim_path
        self._trajectory_debug_enabled = trajectory_debug_enabled
        self._arm_command_mode = arm_command_mode
        self._initialized = False
        self._articulation = None
        self._articulation_kinematics_solver = None
        self._kinematics_solver = None
        self._home_joint_positions: np.ndarray | None = None
        self._arm_effort_control_activated = False

    def initialize_if_needed(self) -> bool:
        if self._initialized:
            return True
        try:
            self._do_initialize()
        except Exception as exc:
            print(f"[Simulator] Franka executor initialization is pending: {exc}", flush=True)
            return False
        self._initialized = True
        return True

    def current_joint_positions(self) -> np.ndarray | None:
        if self._articulation is None:
            return None
        current_positions = self._articulation.get_joint_positions()
        if current_positions is None:
            return None
        return np.asarray(current_positions, dtype=float).reshape(-1)

    def current_joint_velocities(self) -> np.ndarray | None:
        if self._articulation is None or not hasattr(self._articulation, "get_joint_velocities"):
            return None
        current_velocities = self._articulation.get_joint_velocities()
        if current_velocities is None:
            return None
        return np.asarray(current_velocities, dtype=float).reshape(-1)

    def current_joint_efforts(self) -> np.ndarray | None:
        """Isaac APIが提供する実関節effort readbackを返す。"""
        if self._articulation is None:
            return None
        for method_name in ("get_measured_joint_efforts", "get_applied_joint_efforts"):
            method = getattr(self._articulation, method_name, None)
            if not callable(method):
                continue
            efforts = method()
            if efforts is not None:
                return np.asarray(efforts, dtype=float).reshape(-1)
        return None

    def get_observation(self) -> ObservationData:
        return ObservationData(
            joint_positions=self.current_joint_positions(),
            joint_velocities=self.current_joint_velocities(),
            end_effector_pose=self.current_end_effector_pose(),
            joint_state_snapshot=self.current_joint_state_snapshot(),
        )

    def current_end_effector_pose(self) -> Pose3D | None:
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

    def current_joint_state_snapshot(self) -> JointStateSnapshot | None:
        current_positions = self.current_joint_positions()
        if current_positions is None or current_positions.shape[0] < 7:
            return None
        return JointStateSnapshot(
            joint_names=self.ARM_JOINT_NAMES,
            positions_rad=tuple(float(value) for value in current_positions[:7]),
        )

    def preview_end_effector_path_for_joint_trajectory(self, trajectory: JointTrajectory) -> tuple[Pose3D, ...]:
        preview_path: list[Pose3D] = []
        for point in trajectory.points:
            preview_pose = self.preview_end_effector_pose_for_joint_positions(np.asarray(point.positions_rad, dtype=float))
            if preview_pose is None:
                continue
            if not preview_path or preview_path[-1] != preview_pose:
                preview_path.append(preview_pose)
        return tuple(preview_path)

    def preview_end_effector_pose_for_joint_positions(self, joint_positions: np.ndarray) -> Pose3D | None:
        if self._kinematics_solver is None:
            return None
        expanded_positions = self.expand_joint_targets(np.asarray(joint_positions, dtype=float))
        preview_pose = self._try_forward_kinematics_preview(expanded_positions)
        if preview_pose is not None:
            return preview_pose
        return self._preview_pose_by_temporarily_setting_articulation(expanded_positions)

    def home_joint_positions(self) -> np.ndarray | None:
        if self._home_joint_positions is None:
            return None
        return np.asarray(self._home_joint_positions, dtype=float).copy()

    def expand_joint_targets(self, joint_positions: np.ndarray) -> np.ndarray:
        flat_targets = joint_positions.reshape(-1)
        current_positions = self.current_joint_positions()
        if current_positions is None:
            return flat_targets
        if flat_targets.shape == current_positions.shape:
            return flat_targets
        if flat_targets.shape[0] == 7 and current_positions.shape[0] >= 9:
            merged = current_positions.copy()
            merged[:7] = flat_targets
            return merged
        return flat_targets

    def solve_joint_targets_for_pose(
        self,
        target_pose: Pose3D,
        *,
        position_tolerance_m: float,
    ) -> np.ndarray | None:
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
            position_tolerance=position_tolerance_m,
        )
        if not success:
            return None
        target_joint_positions = getattr(action, "joint_positions", None)
        if target_joint_positions is None:
            return None
        return self.expand_joint_targets(np.asarray(target_joint_positions, dtype=float))

    def set_joint_positions_with_debug(self, positions: np.ndarray, *, context: str) -> None:
        if self._articulation is None:
            return
        if hasattr(self._articulation, "apply_action"):
            self._articulation.apply_action(self._create_articulation_action(positions=positions))
            method = "apply_action"
        else:
            self._articulation.set_joint_positions(positions)
            method = "set_joint_positions"
        if not self._trajectory_debug_enabled:
            return
        readback = self.current_joint_positions()
        self._debug_log(
            "[Simulator][TrajectoryDebug][set_joint_positions] "
            f"method={method} "
            f"context={context} "
            f"command_q={self._format_joint_positions(positions[:7])} "
            f"readback_q={self._format_joint_positions(readback[:7]) if readback is not None else 'n/a'}"
        )

    def set_joint_velocity_targets_with_debug(
        self,
        *,
        positions: np.ndarray | None,
        velocities: np.ndarray,
        context: str,
    ) -> None:
        if self._articulation is None:
            return

        method = "set_joint_positions_fallback"
        used_velocity_command = False
        if hasattr(self._articulation, "apply_action"):
            try:
                action = self._create_articulation_action(positions=positions, velocities=velocities)
                self._articulation.apply_action(action)
                method = "apply_action_velocity"
                used_velocity_command = True
            except Exception:
                try:
                    self._articulation.apply_action(self._create_articulation_action(positions=positions))
                    method = "apply_action_position_fallback"
                except Exception:
                    self._articulation.set_joint_positions(positions)
        else:
            self._articulation.set_joint_positions(positions)

        if not self._trajectory_debug_enabled:
            return
        readback = self.current_joint_positions()
        self._debug_log(
            "[Simulator][TrajectoryDebug][set_joint_velocity] "
            f"method={method} "
            f"context={context} "
            f"used_velocity_command={used_velocity_command} "
            f"command_q={self._format_joint_positions(positions[:7]) if positions is not None else 'velocity_only'} "
            f"command_qdot={self._format_joint_positions(velocities[:7]) if velocities.shape[0] >= 7 else '[]'} "
            f"readback_q={self._format_joint_positions(readback[:7]) if readback is not None else 'n/a'}"
        )

    def set_arm_efforts_with_debug(
        self,
        *,
        efforts: np.ndarray,
        context: str,
    ) -> None:
        """arm 7軸へeffortだけを適用する。"""
        if self._articulation is None:
            return
        effort_array = np.asarray(efforts, dtype=float).reshape(-1)
        if effort_array.shape != (7,) or not np.all(np.isfinite(effort_array)):
            return
        action = self._create_articulation_action(
            efforts=effort_array,
            joint_indices=np.arange(7, dtype=np.int64),
        )
        self._articulation.apply_action(action)
        self._debug_log(
            "[Simulator][TrajectoryDebug][set_arm_efforts] "
            f"context={context} "
            f"command_tau={self._format_joint_positions(effort_array)}"
        )

    def set_finger_positions_with_debug(
        self,
        *,
        positions: np.ndarray,
        context: str,
    ) -> None:
        """finger 2軸へpositionだけを適用する。"""
        if self._articulation is None:
            return
        position_array = np.asarray(positions, dtype=float).reshape(-1)
        if position_array.shape != (2,) or not np.all(np.isfinite(position_array)):
            return
        action = self._create_articulation_action(
            positions=position_array,
            joint_indices=np.asarray((7, 8), dtype=np.int64),
        )
        self._articulation.apply_action(action)
        self._debug_log(
            "[Simulator][TrajectoryDebug][set_finger_positions] "
            f"context={context} "
            f"command_q={self._format_joint_positions(position_array)}"
        )

    def _do_initialize(self) -> None:
        import omni.kit.app
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.extensions import get_extension_path_from_name
        from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver

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
        self._home_joint_positions = np.asarray(joint_positions, dtype=float)

    def activate_arm_command_mode(self) -> None:
        """home配置後に選択済みarm制御方式を有効化する。"""
        if self._arm_command_mode != "effort" or self._arm_effort_control_activated:
            return
        self._disable_arm_drive_gains_for_effort_control()
        self._arm_effort_control_activated = True

    def _disable_arm_drive_gains_for_effort_control(self) -> None:
        """armのUSD drive PDを無効化し、effortとの二重制御を防ぐ。"""
        controller = self._articulation.get_articulation_controller()
        kps, kds = controller.get_gains()
        effort_kps = np.asarray(kps, dtype=float).copy()
        effort_kds = np.asarray(kds, dtype=float).copy()
        if effort_kps.shape[0] < 7 or effort_kds.shape[0] < 7:
            raise RuntimeError("Franka articulation gains do not contain seven arm joints")
        effort_kps[:7] = 0.0
        effort_kds[:7] = 0.0
        controller.set_gains(kps=effort_kps, kds=effort_kds, save_to_usd=False)

    def _create_articulation_action(
        self,
        *,
        positions: np.ndarray | None = None,
        velocities: np.ndarray | None = None,
        efforts: np.ndarray | None = None,
        joint_indices: np.ndarray | None = None,
    ) -> object:
        position_array = None if positions is None else np.asarray(positions, dtype=float)
        velocity_array = None if velocities is None else np.asarray(velocities, dtype=float)
        effort_array = None if efforts is None else np.asarray(efforts, dtype=float)
        index_array = None if joint_indices is None else np.asarray(joint_indices, dtype=np.int64)
        try:
            from isaacsim.core.utils.types import ArticulationAction

            kwargs = {}
            if position_array is not None:
                kwargs["joint_positions"] = position_array
            if velocity_array is not None:
                kwargs["joint_velocities"] = velocity_array
            if effort_array is not None:
                kwargs["joint_efforts"] = effort_array
            if index_array is not None:
                kwargs["joint_indices"] = index_array
            return ArticulationAction(**kwargs)
        except Exception:
            return _FallbackArticulationAction(
                joint_positions=position_array,
                joint_velocities=velocity_array,
                joint_efforts=effort_array,
                joint_indices=index_array,
            )

    def _debug_log(self, message: str) -> None:
        if self._trajectory_debug_enabled:
            print(message, flush=True)

    @staticmethod
    def _format_joint_positions(values: tuple[float, ...] | np.ndarray) -> str:
        return "[" + ", ".join(f"{float(value):.4f}" for value in values) + "]"

    def _try_forward_kinematics_preview(self, joint_positions: np.ndarray) -> Pose3D | None:
        if self._kinematics_solver is None:
            return None
        arm_joint_positions = np.asarray(joint_positions[:7], dtype=float)
        for method_name in ("compute_forward_kinematics", "get_end_effector_pose", "compute_end_effector_pose"):
            method = getattr(self._kinematics_solver, method_name, None)
            if not callable(method):
                continue
            for args in (
                ("panda_hand", arm_joint_positions),
                (arm_joint_positions, "panda_hand"),
                ("panda_hand", arm_joint_positions.tolist()),
                (arm_joint_positions.tolist(), "panda_hand"),
            ):
                try:
                    preview_pose = self._pose_from_forward_kinematics_result(method(*args))
                except Exception:
                    continue
                if preview_pose is not None:
                    return preview_pose
        return None

    def _preview_pose_by_temporarily_setting_articulation(self, joint_positions: np.ndarray) -> Pose3D | None:
        if self._articulation is None or not hasattr(self._articulation, "set_joint_positions"):
            return None
        current_positions = self.current_joint_positions()
        if current_positions is None:
            return None
        try:
            self._articulation.set_joint_positions(joint_positions)
            return self.current_end_effector_pose()
        except Exception:
            return None
        finally:
            try:
                self._articulation.set_joint_positions(current_positions)
            except Exception:
                pass

    def _pose_from_forward_kinematics_result(self, result: object) -> Pose3D | None:
        if result is None:
            return None
        position = None
        if isinstance(result, tuple) and result:
            position = result[0]
        elif hasattr(result, "position"):
            position = getattr(result, "position")
        elif hasattr(result, "translation"):
            position = getattr(result, "translation")
        if position is None:
            return None
        coordinates = np.asarray(position, dtype=float).reshape(-1)
        if coordinates.shape[0] < 3:
            return None
        hand_pose = Pose3D(
            x=float(coordinates[0]),
            y=float(coordinates[1]),
            z=float(coordinates[2]),
            roll=180.0,
            pitch=0.0,
            yaw=0.0,
        )
        return _shift_pose_by_local_offset(hand_pose, self.GRASP_TARGET_OFFSET_FROM_HAND_M)
