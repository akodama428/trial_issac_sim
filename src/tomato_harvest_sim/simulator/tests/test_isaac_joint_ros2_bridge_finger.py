"""IsaacJointRos2Bridge がグリッパー開閉状態を毎ステップ適用するテスト"""
from __future__ import annotations

import unittest

import numpy as np


class _DriverStub:
    ARM_JOINT_NAMES = (
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7",
    )

    def __init__(self) -> None:
        self.positions = np.zeros(9, dtype=float)
        self.last_velocity_call: tuple | None = None
        self.last_finger_positions_only_call: tuple | None = None
        self._initialized = True

    def initialize_if_needed(self) -> bool:
        return True

    def current_joint_positions(self) -> np.ndarray:
        return self.positions.copy()

    def current_joint_velocities(self) -> np.ndarray:
        return np.zeros_like(self.positions)

    def set_joint_velocity_targets_with_debug(self, *, positions, velocities, context):
        self.last_velocity_call = (positions.copy(), velocities.copy(), context)
        self.positions = np.asarray(positions, dtype=float).copy()

    def set_finger_positions_only(self, finger_positions, *, joint_indices):
        self.last_finger_positions_only_call = (np.asarray(finger_positions).copy(), list(joint_indices))
        for i, idx in enumerate(joint_indices):
            self.positions[idx] = float(finger_positions[i])


class IsaacJointRos2BridgeFingerTest(unittest.TestCase):
    def _make_bridge(self, driver):
        from tomato_harvest_sim.simulator.isaac_joint_ros2_bridge import IsaacJointRos2Bridge
        bridge = IsaacJointRos2Bridge.__new__(IsaacJointRos2Bridge)
        bridge._driver = driver
        bridge._pending_command = None
        bridge._gripper_closed = False
        return bridge

    def test_joint_command_stores_only_arm_positions(self) -> None:
        driver = _DriverStub()
        bridge = self._make_bridge(driver)

        class _Msg:
            name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                    "panda_joint5", "panda_joint6", "panda_joint7",
                    "panda_finger_joint1", "panda_finger_joint2"]
            position = [0.1, 0.2, 0.3, -1.5, 0.1, 1.2, 0.5, 0.02, 0.02]
            velocity = [0.0] * 9

        bridge._on_joint_command(_Msg())

        self.assertIsNotNone(bridge._pending_command)
        arm_pos, arm_vel = bridge._pending_command
        self.assertEqual(len(arm_pos), 7)
        self.assertAlmostEqual(float(arm_pos[0]), 0.1)
        self.assertAlmostEqual(float(arm_pos[3]), -1.5)

    def test_combined_command_applies_open_gripper_via_finger_positions_only(self) -> None:
        driver = _DriverStub()
        bridge = self._make_bridge(driver)
        bridge._gripper_closed = False

        bridge._apply_combined_command()

        self.assertIsNotNone(driver.last_finger_positions_only_call)
        finger_pos, indices = driver.last_finger_positions_only_call
        self.assertEqual(indices, [7, 8])
        self.assertAlmostEqual(float(finger_pos[0]), 0.04)
        self.assertAlmostEqual(float(finger_pos[1]), 0.04)
        self.assertAlmostEqual(float(driver.positions[7]), 0.04)
        self.assertAlmostEqual(float(driver.positions[8]), 0.04)

    def test_combined_command_applies_closed_gripper_via_finger_positions_only(self) -> None:
        driver = _DriverStub()
        bridge = self._make_bridge(driver)
        bridge._gripper_closed = True

        bridge._apply_combined_command()

        self.assertIsNotNone(driver.last_finger_positions_only_call)
        finger_pos, indices = driver.last_finger_positions_only_call
        self.assertEqual(indices, [7, 8])
        self.assertAlmostEqual(float(finger_pos[0]), 0.0)
        self.assertAlmostEqual(float(finger_pos[1]), 0.0)
        self.assertAlmostEqual(float(driver.positions[7]), 0.0)
        self.assertAlmostEqual(float(driver.positions[8]), 0.0)

    def test_combined_command_applies_arm_from_pending_and_finger_from_gripper_state(self) -> None:
        driver = _DriverStub()
        bridge = self._make_bridge(driver)
        bridge._gripper_closed = True

        class _Msg:
            name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                    "panda_joint5", "panda_joint6", "panda_joint7"]
            position = [0.0, -0.4, 0.0, -2.1, 0.0, 1.7, 0.8]
            velocity = [0.0] * 7

        bridge._on_joint_command(_Msg())
        bridge._apply_combined_command()

        self.assertIsNotNone(driver.last_velocity_call)
        self.assertAlmostEqual(float(driver.positions[1]), -0.4)
        self.assertAlmostEqual(float(driver.positions[3]), -2.1)
        self.assertAlmostEqual(float(driver.positions[7]), 0.0)
        self.assertAlmostEqual(float(driver.positions[8]), 0.0)

    def test_combined_command_uses_finger_positions_only_without_jtc_command(self) -> None:
        driver = _DriverStub()
        driver.positions[:7] = [0.1, -0.5, 0.0, -2.0, 0.1, 1.5, 0.8]
        bridge = self._make_bridge(driver)
        bridge._gripper_closed = True

        bridge._apply_combined_command()

        self.assertIsNotNone(driver.last_finger_positions_only_call)
        self.assertAlmostEqual(float(driver.positions[0]), 0.1)
        self.assertAlmostEqual(float(driver.positions[1]), -0.5)
        self.assertAlmostEqual(float(driver.positions[7]), 0.0)
        self.assertAlmostEqual(float(driver.positions[8]), 0.0)
        self.assertIsNone(driver.last_velocity_call)


if __name__ == "__main__":
    unittest.main()
