"""Isaac Sim effort command経路の境界契約を固定するテスト。"""
from __future__ import annotations

import unittest

import numpy as np


class _DriverStub:
    ARM_JOINT_NAMES = tuple(f"panda_joint{i}" for i in range(1, 8))

    def __init__(self) -> None:
        self.positions = np.asarray(
            (0.0, -0.4, 0.0, -2.1, 0.0, 1.7, 0.8, 0.04, 0.04),
            dtype=float,
        )
        self.arm_effort_call: tuple[np.ndarray, str] | None = None
        self.finger_position_call: tuple[np.ndarray, str] | None = None

    def initialize_if_needed(self) -> bool:
        return True

    def current_joint_positions(self) -> np.ndarray:
        return self.positions.copy()

    def set_arm_efforts_with_debug(self, *, efforts, context):
        self.arm_effort_call = (np.asarray(efforts, dtype=float), context)

    def set_finger_positions_with_debug(self, *, positions, context):
        self.finger_position_call = (np.asarray(positions, dtype=float), context)


class _Message:
    def __init__(self, *, names=(), positions=(), efforts=()) -> None:
        self.name = list(names)
        self.position = list(positions)
        self.velocity: list[float] = []
        self.effort = list(efforts)


class IsaacEffortCommandTest(unittest.TestCase):
    def _make_bridge(self):
        from tomato_harvest_sim.simulator.isaac_joint_ros2_bridge import (
            IsaacJointRos2Bridge,
        )

        bridge = IsaacJointRos2Bridge.__new__(IsaacJointRos2Bridge)
        bridge._driver = _DriverStub()
        bridge._arm_command_mode = "effort"
        bridge._pending_command = None
        bridge._pending_arm_effort = None
        bridge._pending_finger_positions = None
        return bridge

    def test_effort_command_requires_all_arm_joints_in_canonical_order(self) -> None:
        bridge = self._make_bridge()

        bridge._on_arm_effort_command(
            _Message(
                names=bridge._driver.ARM_JOINT_NAMES,
                efforts=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0),
            )
        )

        self.assertTrue(
            np.allclose(bridge._pending_arm_effort, (1, 2, 3, 4, 5, 6, 7))
        )

    def test_effort_command_rejects_non_finite_values(self) -> None:
        bridge = self._make_bridge()

        bridge._on_arm_effort_command(
            _Message(
                names=bridge._driver.ARM_JOINT_NAMES,
                efforts=(1.0, 2.0, np.nan, 4.0, 5.0, 6.0, 7.0),
            )
        )

        self.assertIsNone(bridge._pending_arm_effort)

    def test_apply_effort_mode_uses_disjoint_arm_and_finger_calls(self) -> None:
        bridge = self._make_bridge()
        bridge._pending_arm_effort = np.asarray((1, 2, 3, 4, 5, 6, 7), dtype=float)
        bridge._pending_finger_positions = np.asarray((0.02, 0.02), dtype=float)

        bridge._apply_pending_command()

        self.assertIsNotNone(bridge._driver.arm_effort_call)
        self.assertIsNotNone(bridge._driver.finger_position_call)
        self.assertEqual(bridge._driver.arm_effort_call[1], "isaac_arm_effort_command")
        self.assertEqual(
            bridge._driver.finger_position_call[1],
            "isaac_finger_position_command",
        )
        self.assertIsNone(bridge._pending_arm_effort)
        self.assertIsNone(bridge._pending_finger_positions)

    def test_driver_effort_action_contains_only_effort_and_arm_indices(self) -> None:
        from tomato_harvest_sim.simulator.isaac_franka_driver import IsaacFrankaDriver

        driver = IsaacFrankaDriver(robot_prim_path="/World/Franka")
        action = driver._create_articulation_action(
            efforts=np.ones(7, dtype=float),
            joint_indices=np.arange(7, dtype=np.int64),
        )

        self.assertIsNone(action.joint_positions)
        self.assertIsNone(action.joint_velocities)
        self.assertTrue(np.allclose(action.joint_efforts, 1.0))
        self.assertTrue(np.array_equal(action.joint_indices, np.arange(7)))

    def test_driver_defers_effort_gain_switch_until_home_is_initialized(self) -> None:
        from tomato_harvest_sim.simulator.isaac_franka_driver import IsaacFrankaDriver

        driver = IsaacFrankaDriver(
            robot_prim_path="/World/Franka",
            arm_command_mode="effort",
        )
        calls: list[str] = []
        driver._disable_arm_drive_gains_for_effort_control = lambda: calls.append("gain")

        self.assertEqual(calls, [])
        driver.activate_arm_command_mode()
        driver.activate_arm_command_mode()

        self.assertEqual(calls, ["gain"])


if __name__ == "__main__":
    unittest.main()
