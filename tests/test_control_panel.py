from __future__ import annotations

import unittest

from tomato_harvest_sim.app.application import create_tomato_harvest_application
from tomato_harvest_sim.simulator.control_panel import ControlPanelController, load_control_panel_layout_settings


class ControlPanelTest(unittest.TestCase):
    def test_layout_settings_request_property_tab_docking(self) -> None:
        settings = load_control_panel_layout_settings()

        self.assertEqual(settings.title, "Tomato Harvest Controls")
        self.assertEqual(settings.dock_target, "Property")
        self.assertEqual(settings.dock_policy, "DO_NOTHING")
        self.assertEqual(settings.dock_preference, "MAIN")

    def test_boot_sets_initial_hand_camera_and_logs_ready(self) -> None:
        messages: list[str] = []
        camera_events: list[str] = []
        controller = ControlPanelController(
            system=create_tomato_harvest_application(),
            set_viewport_camera=camera_events.append,
            log_fn=messages.append,
        )

        status = controller.boot(initial_camera_name="hand_camera")

        self.assertEqual(status.active_camera, "hand_camera")
        self.assertEqual(camera_events, ["hand_camera"])
        self.assertIn("Ready", messages[0])

    def test_start_stop_reset_update_system_state(self) -> None:
        controller = ControlPanelController(
            system=create_tomato_harvest_application(),
            set_viewport_camera=lambda camera_name: None,
            log_fn=lambda message: None,
        )
        controller.boot(initial_camera_name="fixed_camera")

        running = controller.start()
        stopped = controller.stop()
        reset = controller.reset()

        self.assertEqual(running.scene_phase, "running")
        self.assertEqual(running.robot_state, "running")
        self.assertEqual(stopped.scene_phase, "stopped")
        self.assertEqual(stopped.robot_state, "stopped")
        self.assertEqual(reset.scene_phase, "ready")
        self.assertEqual(reset.robot_state, "ready")
        self.assertEqual(reset.active_camera, "fixed_camera")

    def test_physics_start_is_queued_after_reset_for_deterministic_manual_start(self) -> None:
        messages: list[str] = []
        controller = ControlPanelController(
            system=create_tomato_harvest_application(physics_grasp_enabled=True),
            set_viewport_camera=lambda camera_name: None,
            log_fn=messages.append,
        )
        controller.boot(initial_camera_name="fixed_camera")

        prepared = controller.start()
        waiting = controller.step_runtime()
        running = waiting
        for _ in range(controller.PHYSICS_START_DELAY_FRAMES):
            running = controller.step_runtime()
            if running.scene_phase == "running":
                break

        self.assertEqual(prepared.scene_phase, "ready")
        self.assertEqual(waiting.scene_phase, "ready")
        self.assertEqual(running.scene_phase, "running")
        self.assertTrue(any("[StartPrep]" in message for message in messages))
        self.assertTrue(any("[Start] accepted=True scene=running robot=running" in message for message in messages))

    def test_select_camera_updates_runtime_and_viewport(self) -> None:
        camera_events: list[str] = []
        controller = ControlPanelController(
            system=create_tomato_harvest_application(),
            set_viewport_camera=camera_events.append,
            log_fn=lambda message: None,
        )
        controller.boot(initial_camera_name="fixed_camera")

        status = controller.select_camera("hand_camera")

        self.assertEqual(status.active_camera, "hand_camera")
        self.assertEqual(camera_events[-1], "hand_camera")


if __name__ == "__main__":
    unittest.main()
