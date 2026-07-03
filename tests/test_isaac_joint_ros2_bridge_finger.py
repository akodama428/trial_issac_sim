"""IsaacJointRos2Bridge が finger[7-8] を /isaac_joint_commands から受け取るテスト"""
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


class IsaacJointRos2BridgeFingerTest(unittest.TestCase):
    def _make_bridge(self, driver):
        from tomato_harvest_sim.simulator.isaac_joint_ros2_bridge import IsaacJointRos2Bridge
        bridge = IsaacJointRos2Bridge.__new__(IsaacJointRos2Bridge)
        bridge._driver = driver
        bridge._pending_command = None
        return bridge

    def test_finger_positions_are_extracted_from_joint_command(self) -> None:
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
        positions, velocities = bridge._pending_command
        # finger[7-8] が反映されている
        self.assertAlmostEqual(float(positions[7]), 0.02)
        self.assertAlmostEqual(float(positions[8]), 0.02)

    def test_pending_command_applies_finger_positions_to_driver(self) -> None:
        driver = _DriverStub()
        bridge = self._make_bridge(driver)

        class _Msg:
            name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                    "panda_joint5", "panda_joint6", "panda_joint7",
                    "panda_finger_joint1", "panda_finger_joint2"]
            position = [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.8, 0.03, 0.03]
            velocity = [0.0] * 9

        bridge._on_joint_command(_Msg())
        bridge._apply_pending_command()

        self.assertAlmostEqual(float(driver.positions[7]), 0.03)
        self.assertAlmostEqual(float(driver.positions[8]), 0.03)


if __name__ == "__main__":
    unittest.main()
