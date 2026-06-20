from __future__ import annotations

import unittest

import numpy as np

from tomato_harvest_poc.isaac_native_runtime import IsaacNativeRuntime


class _FakeTimeline:
    def __init__(self) -> None:
        self.play_calls = 0
        self.pause_calls = 0

    def play(self) -> None:
        self.play_calls += 1

    def pause(self) -> None:
        self.pause_calls += 1


class IsaacNativeRuntimeUiActionTest(unittest.TestCase):
    def test_start_click_defers_scenario_execution_until_update(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        calls: list[str] = []
        runtime._start_scenario = lambda: calls.append("start")  # type: ignore[method-assign]

        runtime._on_start_clicked()
        self.assertEqual(calls, [])

        runtime._update_runtime()
        self.assertEqual(calls, ["start"])

    def test_camera_click_defers_camera_switch_until_update(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        calls: list[str] = []
        runtime._set_camera_view = lambda camera_view: calls.append(camera_view.value)  # type: ignore[method-assign]

        runtime._on_hand_camera_clicked()
        self.assertEqual(calls, [])

        runtime._update_runtime()
        self.assertEqual(calls, ["hand"])

    def test_complete_scenario_pauses_timeline(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._timeline = _FakeTimeline()
        tomato_positions: list[tuple[float, float, float]] = []
        runtime._set_tomato_world_position = lambda position_m: tomato_positions.append(position_m)  # type: ignore[method-assign]

        runtime._complete_scenario()

        self.assertEqual(runtime._timeline.pause_calls, 1)
        self.assertEqual(tomato_positions, [])

    def test_reset_scene_returns_robot_and_tomato_to_initial_state(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._timeline = _FakeTimeline()
        robot_positions: list[tuple[float, ...]] = []
        tomato_positions: list[tuple[float, float, float]] = []
        runtime._set_articulation_positions = lambda positions: robot_positions.append(tuple(positions))  # type: ignore[method-assign]
        runtime._set_tomato_world_position = lambda position_m: tomato_positions.append(position_m)  # type: ignore[method-assign]
        runtime._pump_updates = lambda frame_count: None  # type: ignore[method-assign]
        runtime._set_camera_view = lambda camera_view: None  # type: ignore[method-assign]
        runtime._attached_tomato = True
        runtime._attached_tomato_local_offset = (1.0, 2.0, 3.0)
        runtime._found_camera_point = (1.0, 1.0, 1.0)
        runtime._found_world_point = (2.0, 2.0, 2.0)
        runtime._stop_requested = True
        runtime._scenario_active = True
        runtime._animation = object()  # type: ignore[assignment]

        runtime._reset_scene()

        self.assertEqual(runtime._timeline.pause_calls, 1)
        self.assertEqual(robot_positions[-1], runtime._plan.home_dof_positions)
        self.assertEqual(tomato_positions[-1], runtime._plan.tomato_initial_world_m)
        self.assertFalse(runtime._attached_tomato)
        self.assertIsNone(runtime._attached_tomato_local_offset)
        self.assertIsNone(runtime._found_camera_point)
        self.assertIsNone(runtime._found_world_point)
        self.assertFalse(runtime._stop_requested)
        self.assertFalse(runtime._scenario_active)

    def test_start_scenario_resumes_timeline(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._timeline = _FakeTimeline()
        runtime._reset_scene = lambda reset_phase=False: None  # type: ignore[method-assign]
        runtime._pump_updates = lambda frame_count: None  # type: ignore[method-assign]
        runtime._set_phase = lambda phase, message: None  # type: ignore[method-assign]
        runtime._set_camera_view = lambda camera_view: None  # type: ignore[method-assign]
        queued: list[int] = []
        runtime._queue_scan_pose = lambda index: queued.append(index)  # type: ignore[method-assign]

        runtime._start_scenario()

        self.assertEqual(runtime._timeline.play_calls, 1)
        self.assertEqual(queued, [0])

    def test_hand_target_position_uses_grasp_center_offset(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._top_down_grasp_center_offset_world = np.array([0.0, 0.0, 0.1034], dtype=float)

        target_position = runtime._compute_hand_target_position((0.50, 0.00, 0.42))

        self.assertEqual(target_position, (0.5, 0.0, 0.3166))

    def test_detach_tomato_freezes_release_position_and_queues_retreat(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._attached_tomato = True
        runtime._attached_tomato_local_offset = (0.1, 0.2, 0.3)
        runtime._top_down_hand_orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        runtime._read_tomato_world_position = lambda: (0.36, -0.44, 0.386)  # type: ignore[method-assign]
        frozen_positions: list[tuple[float, float, float]] = []
        runtime._set_tomato_world_position = lambda position_m: frozen_positions.append(position_m)  # type: ignore[method-assign]
        runtime._compute_hand_target_position = lambda position_m: position_m  # type: ignore[method-assign]
        runtime._solve_ik_joint_positions = lambda position_m, target_orientation=None: np.array(position_m, dtype=float)  # type: ignore[method-assign]
        queued: list[tuple[tuple[float, ...], int, object | None]] = []
        runtime._queue_joint_animation = lambda *, target_positions, frames, on_complete: queued.append((tuple(target_positions), frames, on_complete))  # type: ignore[method-assign]
        runtime._set_phase = lambda phase, message: None  # type: ignore[method-assign]

        runtime._detach_tomato_and_retreat()

        self.assertFalse(runtime._attached_tomato)
        self.assertIsNone(runtime._attached_tomato_local_offset)
        self.assertEqual(frozen_positions, [(0.36, -0.44, 0.386)])
        self.assertEqual(queued[0][0], (0.35, -0.45, 0.545))
        self.assertEqual(queued[0][1], 50)


if __name__ == "__main__":
    unittest.main()
