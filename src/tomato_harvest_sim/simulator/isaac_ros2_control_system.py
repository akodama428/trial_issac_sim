from __future__ import annotations

import time

import numpy as np

from tomato_harvest_sim.api.hardware_control import HardwareCommandSample, HardwareControlPort, HardwareStateSample
from tomato_harvest_sim.simulator.isaac_franka_driver import IsaacFrankaDriver


class IsaacRos2ControlSystem(HardwareControlPort):
    GRIPPER_OPEN_POSITION_RAD = 0.04
    GRIPPER_CLOSED_POSITION_RAD = 0.0

    def __init__(self, *, driver: IsaacFrankaDriver, monotonic_time_sec: callable | None = None) -> None:
        self._driver = driver
        self._last_gripper_closed_command: bool | None = None

    def initialize_if_needed(self) -> bool:
        return self._driver.initialize_if_needed()

    def read_state(self) -> HardwareStateSample | None:
        if not self.initialize_if_needed():
            return None
        positions = self._driver.current_joint_positions()
        if positions is None:
            return None
        velocities = self._driver.current_joint_velocities()
        if velocities is None:
            velocities = np.zeros_like(positions, dtype=float)
        joint_state_snapshot = self._driver.current_joint_state_snapshot()
        joint_names = joint_state_snapshot.joint_names if joint_state_snapshot is not None else self._driver.ARM_JOINT_NAMES
        if len(joint_names) < positions.shape[0]:
            joint_names = tuple(joint_names) + tuple(f"joint_{index}" for index in range(len(joint_names), positions.shape[0]))
        return HardwareStateSample(
            joint_names=tuple(joint_names),
            positions_rad=tuple(float(value) for value in positions),
            velocities_rad_s=tuple(float(value) for value in velocities),
            timestamp_sec=time.monotonic(),
            end_effector_pose=self._driver.current_end_effector_pose(),
            joint_state_snapshot=joint_state_snapshot,
        )

    def write_command(self, command: HardwareCommandSample) -> None:
        if not self.initialize_if_needed():
            return
        current_positions = self._driver.current_joint_positions()
        if current_positions is None:
            return

        merged_positions = np.asarray(current_positions, dtype=float).copy()
        if command.positions_rad is not None:
            position_array = np.asarray(command.positions_rad, dtype=float)
            merged_positions[: position_array.shape[0]] = position_array
        if merged_positions.shape[0] >= 9 and command.gripper_closed is not None:
            finger_value = self.GRIPPER_CLOSED_POSITION_RAD if command.gripper_closed else self.GRIPPER_OPEN_POSITION_RAD
            merged_positions[7] = finger_value
            merged_positions[8] = finger_value

        if command.velocities_rad_s is None:
            self._driver.set_joint_positions_with_debug(merged_positions, context=command.context)
            self._last_gripper_closed_command = command.gripper_closed
            return

        velocity_array = np.asarray(command.velocities_rad_s, dtype=float)
        merged_velocities = np.zeros_like(merged_positions)
        merged_velocities[: velocity_array.shape[0]] = velocity_array
        self._apply_gripper_position_command_if_needed(
            command=command,
            current_positions=np.asarray(current_positions, dtype=float),
        )
        velocity_command_positions = None if command.positions_rad is None else merged_positions
        self._driver.set_joint_velocity_targets_with_debug(
            positions=velocity_command_positions,
            velocities=merged_velocities,
            context=command.context,
        )

    def _apply_gripper_position_command_if_needed(
        self,
        *,
        command: HardwareCommandSample,
        current_positions: np.ndarray,
    ) -> None:
        if command.gripper_closed is None or current_positions.shape[0] < 9:
            return
        if self._last_gripper_closed_command is command.gripper_closed:
            return
        finger_value = self.GRIPPER_CLOSED_POSITION_RAD if command.gripper_closed else self.GRIPPER_OPEN_POSITION_RAD
        gripper_positions = np.asarray(current_positions, dtype=float).copy()
        gripper_positions[7] = finger_value
        gripper_positions[8] = finger_value
        self._driver.set_joint_positions_with_debug(
            gripper_positions,
            context=f"{command.context}:gripper",
        )
        self._last_gripper_closed_command = command.gripper_closed
