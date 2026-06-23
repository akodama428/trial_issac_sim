from __future__ import annotations

import unittest

from tomato_harvest_sim.api.contracts import ControlCommand, HarvestTaskPhase, TomatoStatus
from tomato_harvest_sim.app.application import create_tomato_harvest_application


class HarvestExecutionStateTest(unittest.TestCase):
    def test_success_scenario_reaches_detached_state(self) -> None:
        system = create_tomato_harvest_application(grasp_mode="success")
        system.boot()
        system.apply_control(ControlCommand.START)

        logs: list[str] = []
        for _ in range(160):
            logs.extend(system.step())
            if system.robot.state.task_phase.value in {"detached", "moving_to_place", "placed", "returning_home", "complete"}:
                break

        self.assertIn(
            system.robot.state.task_phase,
            {
                HarvestTaskPhase.DETACHED,
                HarvestTaskPhase.MOVING_TO_PLACE,
                HarvestTaskPhase.PLACED,
                HarvestTaskPhase.RETURNING_HOME,
                HarvestTaskPhase.COMPLETE,
            },
        )
        self.assertIn(system.simulator.state.tomato_status, {TomatoStatus.DETACHED, TomatoStatus.PLACED})
        self.assertTrue(any("Stable grasp established" in line for line in logs))
        self.assertTrue(any("Tomato detached from stem" in line for line in logs))

    def test_failure_scenario_reaches_failed_state(self) -> None:
        system = create_tomato_harvest_application(grasp_mode="failure")
        system.boot()
        system.apply_control(ControlCommand.START)

        logs: list[str] = []
        for _ in range(160):
            logs.extend(system.step())
            if system.robot.state.task_phase is HarvestTaskPhase.FAILED:
                break

        self.assertEqual(system.robot.state.task_phase, HarvestTaskPhase.FAILED)
        self.assertEqual(system.simulator.state.tomato_status, TomatoStatus.FALLEN)
        self.assertTrue(any("not stably grasped" in line for line in logs))

    def test_grasp_evaluation_times_out_when_physics_result_never_arrives(self) -> None:
        system = create_tomato_harvest_application(
            grasp_mode="success",
            physics_grasp_enabled=True,
            transport="in_memory",
            autostart_moveit_service=False,
        )
        system.boot()
        system.apply_control(ControlCommand.START)

        logs: list[str] = []
        for _ in range(220):
            logs.extend(system.step())
            if system.robot.state.task_phase is HarvestTaskPhase.FAILED:
                break

        self.assertEqual(system.robot.state.task_phase, HarvestTaskPhase.FAILED)
        self.assertEqual(system.simulator.state.tomato_status, TomatoStatus.ATTACHED)
        self.assertTrue(any("Waiting for the physics grasp result" in line for line in logs))
        self.assertTrue(any("Grasp evaluation timed out" in line for line in logs))

    def test_runtime_waits_briefly_at_grasp_before_closing_gripper(self) -> None:
        system = create_tomato_harvest_application(
            grasp_mode="success",
            physics_grasp_enabled=True,
            transport="in_memory",
            autostart_moveit_service=False,
        )
        system.boot()
        system.apply_control(ControlCommand.START)

        settle_logs: list[str] = []
        for _ in range(220):
            step_logs = system.step()
            settle_logs.extend(step_logs)
            if any("Settling at grasp pose before closing the gripper." in line for line in step_logs):
                break

        self.assertEqual(system.robot.state.task_phase, HarvestTaskPhase.AT_GRASP)
        self.assertTrue(any("Settling at grasp pose before closing the gripper." in line for line in settle_logs))
        self.assertNotEqual(system.bridge.state.last_motion_command.command_name, "close_gripper")

        for _ in range(system.robot.GRASP_SETTLE_STEPS):
            settle_logs.extend(system.step())
            if system.robot.state.task_phase is HarvestTaskPhase.GRASP_EVALUATION:
                break

        self.assertEqual(system.robot.state.task_phase, HarvestTaskPhase.GRASP_EVALUATION)
        self.assertEqual(system.bridge.state.last_motion_command.command_name, "close_gripper")


if __name__ == "__main__":
    unittest.main()
