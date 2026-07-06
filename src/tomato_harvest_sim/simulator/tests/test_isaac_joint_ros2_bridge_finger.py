"""IsaacJointRos2Bridge が finger を HWI 経由の joint command として扱うテスト"""
from __future__ import annotations

import unittest

import numpy as np


class _PublisherStub:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class _ClockStub:
    class _NowStub:
        @staticmethod
        def to_msg() -> object:
            return object()

    @staticmethod
    def now() -> "_ClockStub._NowStub":
        return _ClockStub._NowStub()


class _NodeStub:
    @staticmethod
    def get_clock() -> _ClockStub:
        return _ClockStub()


class _JointStateStub:
    def __init__(self) -> None:
        self.header = type("Header", (), {"stamp": None})()
        self.name: list[str] = []
        self.position: list[float] = []
        self.velocity: list[float] = []


class _DriverStub:
    ARM_JOINT_NAMES = (
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7",
    )

    def __init__(self) -> None:
        self.positions = np.asarray((0.0, -0.4, 0.0, -2.1, 0.0, 1.7, 0.8, 0.04, 0.04), dtype=float)
        self.velocities = np.zeros(9, dtype=float)
        self.last_velocity_call: tuple | None = None
        self._initialized = True

    def initialize_if_needed(self) -> bool:
        return True

    def current_joint_positions(self) -> np.ndarray:
        return self.positions.copy()

    def current_joint_velocities(self) -> np.ndarray:
        return self.velocities.copy()

    def set_joint_velocity_targets_with_debug(self, *, positions, velocities, context):
        self.last_velocity_call = (np.asarray(positions).copy(), np.asarray(velocities).copy(), context)
        self.positions = np.asarray(positions, dtype=float).copy()
        self.velocities = np.asarray(velocities, dtype=float).copy()


class IsaacJointRos2BridgeFingerTest(unittest.TestCase):
    def _make_bridge(self, driver):
        from tomato_harvest_sim.simulator.isaac_joint_ros2_bridge import IsaacJointRos2Bridge

        bridge = IsaacJointRos2Bridge.__new__(IsaacJointRos2Bridge)
        bridge._driver = driver
        bridge._pending_command = None
        bridge._node = _NodeStub()
        bridge._pub = _PublisherStub()
        bridge._JointState = _JointStateStub
        return bridge

    def test_publish_state_includes_finger_joint_names(self) -> None:
        driver = _DriverStub()
        bridge = self._make_bridge(driver)

        bridge._publish_state()

        self.assertEqual(len(bridge._pub.messages), 1)
        message = bridge._pub.messages[0]
        self.assertEqual(
            message.name,
            [
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
                "panda_finger_joint1",
                "panda_finger_joint2",
            ],
        )
        self.assertAlmostEqual(message.position[7], 0.04)
        self.assertAlmostEqual(message.position[8], 0.04)

    def test_joint_command_stores_arm_and_finger_positions(self) -> None:
        driver = _DriverStub()
        bridge = self._make_bridge(driver)

        class _Msg:
            name = [
                "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                "panda_joint5", "panda_joint6", "panda_joint7",
                "panda_finger_joint1", "panda_finger_joint2",
            ]
            position = [0.1, 0.2, 0.3, -1.5, 0.1, 1.2, 0.5, 0.02, 0.02]
            velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0]

        bridge._on_joint_command(_Msg())

        self.assertIsNotNone(bridge._pending_command)
        positions, velocities = bridge._pending_command
        self.assertEqual(len(positions), 9)
        self.assertAlmostEqual(float(positions[0]), 0.1)
        self.assertAlmostEqual(float(positions[7]), 0.02)
        self.assertAlmostEqual(float(positions[8]), 0.02)
        self.assertAlmostEqual(float(velocities[6]), 0.1)

    def test_joint_command_preserves_current_finger_positions_when_message_omits_them(self) -> None:
        driver = _DriverStub()
        bridge = self._make_bridge(driver)

        class _Msg:
            name = [
                "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                "panda_joint5", "panda_joint6", "panda_joint7",
            ]
            position = [0.2, 0.1, -0.1, -2.0, 0.0, 1.5, 0.7]
            velocity = [0.0] * 7

        bridge._on_joint_command(_Msg())

        positions, _ = bridge._pending_command
        self.assertAlmostEqual(float(positions[7]), 0.04)
        self.assertAlmostEqual(float(positions[8]), 0.04)

    def test_apply_pending_command_forwards_full_joint_command(self) -> None:
        driver = _DriverStub()
        bridge = self._make_bridge(driver)

        class _Msg:
            name = [
                "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                "panda_joint5", "panda_joint6", "panda_joint7",
                "panda_finger_joint1", "panda_finger_joint2",
            ]
            position = [0.0, -0.5, 0.0, -2.2, 0.0, 1.8, 0.9, 0.0, 0.0]
            velocity = [0.0] * 9

        bridge._on_joint_command(_Msg())
        bridge._apply_pending_command()

        self.assertIsNotNone(driver.last_velocity_call)
        full_positions, full_velocities, context = driver.last_velocity_call
        self.assertEqual(context, "isaac_joint_command")
        self.assertEqual(full_positions.shape[0], 9)
        self.assertAlmostEqual(float(full_positions[1]), -0.5)
        self.assertAlmostEqual(float(full_positions[7]), 0.0)
        self.assertAlmostEqual(float(full_positions[8]), 0.0)
        self.assertTrue(np.allclose(full_velocities, 0.0))
        self.assertIsNone(bridge._pending_command)

    def test_apply_pending_command_without_pending_command_is_noop(self) -> None:
        driver = _DriverStub()
        bridge = self._make_bridge(driver)

        bridge._apply_pending_command()

        self.assertIsNone(driver.last_velocity_call)


if __name__ == "__main__":
    unittest.main()
