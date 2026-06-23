from __future__ import annotations

import unittest
from unittest.mock import patch

from tomato_harvest_sim.api.bridge import InMemoryRos2Bridge
from tomato_harvest_sim.api.contracts import ControlCommand, RobotRuntimeState, ScenePhase
from tomato_harvest_sim.app.application import create_tomato_harvest_application


class ControlContractTest(unittest.TestCase):
    def test_start_starts_both_runtimes_and_shares_scene_snapshot(self) -> None:
        system = create_tomato_harvest_application()

        system.boot()
        result = system.apply_control(ControlCommand.START)

        self.assertTrue(result.accepted)
        self.assertEqual(system.simulator.state.phase, ScenePhase.RUNNING)
        self.assertEqual(system.robot.state.runtime_state, RobotRuntimeState.RUNNING)
        self.assertEqual(system.robot.state.last_seen_phase, ScenePhase.RUNNING)
        self.assertIsNotNone(system.robot.state.last_scene_snapshot)
        self.assertEqual(system.bridge.state.last_command, ControlCommand.START)
        self.assertIsNotNone(system.bridge.state.last_scene_snapshot)
        self.assertEqual(system.bridge.state.last_scene_snapshot.phase, ScenePhase.RUNNING)

    def test_stop_stops_both_runtimes(self) -> None:
        system = create_tomato_harvest_application()

        system.boot()
        system.apply_control(ControlCommand.START)
        result = system.apply_control(ControlCommand.STOP)

        self.assertTrue(result.accepted)
        self.assertEqual(system.simulator.state.phase, ScenePhase.STOPPED)
        self.assertEqual(system.robot.state.runtime_state, RobotRuntimeState.STOPPED)
        self.assertEqual(system.robot.state.last_seen_phase, ScenePhase.STOPPED)

    def test_reset_restores_initial_scene_and_robot_state(self) -> None:
        system = create_tomato_harvest_application()

        system.boot()
        system.apply_control(ControlCommand.START)
        system.simulator.set_active_camera("hand_camera")
        system.simulator.move_robot_home(False)
        system.apply_control(ControlCommand.RESET)

        self.assertEqual(system.simulator.state.phase, ScenePhase.READY)
        self.assertEqual(system.simulator.state.active_camera, "fixed_camera")
        self.assertTrue(system.simulator.state.robot_home)
        self.assertEqual(system.robot.state.runtime_state, RobotRuntimeState.READY)
        self.assertEqual(system.robot.state.last_seen_phase, ScenePhase.READY)

    @patch("tomato_harvest_sim.app.application.MoveItServiceManager.start_if_needed")
    def test_in_memory_transport_skips_moveit_service_autostart(self, start_if_needed: object) -> None:
        system = create_tomato_harvest_application(transport="in_memory")

        self.assertIsInstance(system.bridge, InMemoryRos2Bridge)
        start_if_needed.assert_not_called()


if __name__ == "__main__":
    unittest.main()
