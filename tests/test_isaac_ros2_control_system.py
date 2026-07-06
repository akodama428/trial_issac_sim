from __future__ import annotations

import unittest

import numpy as np

from tomato_harvest_sim.msg.contracts import JointStateSnapshot
from tomato_harvest_sim.msg.hardware_control import HardwareCommandSample
from tomato_harvest_sim.simulator.isaac_ros2_control_system import IsaacRos2ControlSystem


class _DriverStub:
    ARM_JOINT_NAMES = (
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    )

    def __init__(self) -> None:
        self.positions = np.asarray((0.1, 0.2, 0.3, -1.0, 0.4, 1.2, 0.7, 0.04, 0.04), dtype=float)
        self.velocity_calls: list[tuple[np.ndarray | None, np.ndarray, str]] = []
        self.position_calls: list[tuple[np.ndarray, str]] = []

    def initialize_if_needed(self) -> bool:
        return True

    def current_joint_positions(self) -> np.ndarray | None:
        return self.positions.copy()

    def current_joint_velocities(self) -> np.ndarray | None:
        return np.zeros_like(self.positions)

    def current_joint_state_snapshot(self) -> JointStateSnapshot:
        return JointStateSnapshot(
            joint_names=self.ARM_JOINT_NAMES,
            positions_rad=tuple(float(value) for value in self.positions[:7]),
        )

    def current_end_effector_pose(self):
        return None

    def set_joint_positions_with_debug(self, positions: np.ndarray, *, context: str) -> None:
        self.position_calls.append((np.asarray(positions, dtype=float).copy(), context))
        self.positions = np.asarray(positions, dtype=float).copy()

    def set_joint_velocity_targets_with_debug(
        self,
        *,
        positions: np.ndarray | None,
        velocities: np.ndarray,
        context: str,
    ) -> None:
        position_copy = None if positions is None else np.asarray(positions, dtype=float).copy()
        velocity_copy = np.asarray(velocities, dtype=float).copy()
        self.velocity_calls.append((position_copy, velocity_copy, context))


class IsaacRos2ControlSystemTest(unittest.TestCase):
    def test_velocity_command_forwards_supplied_positions_and_velocities(self) -> None:
        driver = _DriverStub()
        system = IsaacRos2ControlSystem(driver=driver)

        system.write_command(
            HardwareCommandSample(
                joint_names=driver.ARM_JOINT_NAMES + ("finger_left", "finger_right"),
                positions_rad=tuple(float(value) for value in (0.9, 0.8, 0.7, -1.5, 0.2, 1.0, 0.5, 0.04, 0.04)),
                velocities_rad_s=tuple(float(value) for value in (0.3, -0.2, 0.1, -0.4, 0.2, -0.1, 0.5, 0.0, 0.0)),
                context="joint_trajectory:0",
                gripper_closed=False,
            )
        )

        self.assertEqual(len(driver.velocity_calls), 1)
        velocity_positions, velocity_values, _ = driver.velocity_calls[0]
        expected_positions = np.asarray((0.9, 0.8, 0.7, -1.5, 0.2, 1.0, 0.5, 0.04, 0.04), dtype=float)
        np.testing.assert_allclose(velocity_positions, expected_positions)
        np.testing.assert_allclose(velocity_values[:7], np.asarray((0.3, -0.2, 0.1, -0.4, 0.2, -0.1, 0.5), dtype=float))

    def test_velocity_command_updates_gripper_only_on_state_change(self) -> None:
        driver = _DriverStub()
        system = IsaacRos2ControlSystem(driver=driver)

        command = HardwareCommandSample(
            joint_names=driver.ARM_JOINT_NAMES + ("finger_left", "finger_right"),
            positions_rad=None,
            velocities_rad_s=tuple(float(value) for value in (0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            context="joint_trajectory:1",
            gripper_closed=True,
        )

        system.write_command(command)
        system.write_command(command)

        self.assertEqual(len(driver.position_calls), 1)
        gripper_positions, context = driver.position_calls[0]
        self.assertEqual(context, "joint_trajectory:1:gripper")
        self.assertAlmostEqual(float(gripper_positions[7]), 0.0)
        self.assertAlmostEqual(float(gripper_positions[8]), 0.0)
        self.assertEqual(len(driver.velocity_calls), 2)


if __name__ == "__main__":
    unittest.main()
