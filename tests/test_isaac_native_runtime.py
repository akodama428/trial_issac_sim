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
        runtime._set_articulation_positions = lambda positions: robot_positions.append(tuple(positions))  # type: ignore[method-assign]
        physics_resets: list[str] = []
        runtime._reset_physics_scene_state = lambda: physics_resets.append("reset")  # type: ignore[method-assign]
        runtime._pump_updates = lambda frame_count: None  # type: ignore[method-assign]
        runtime._set_camera_view = lambda camera_view: None  # type: ignore[method-assign]
        runtime._found_camera_point = (1.0, 1.0, 1.0)
        runtime._found_world_point = (2.0, 2.0, 2.0)
        runtime._stop_requested = True
        runtime._scenario_active = True
        runtime._animation = object()  # type: ignore[assignment]
        runtime._grasp_check_frames_remaining = 5
        runtime._settle_monitor_active = True
        runtime._settle_stable_frames = 3

        runtime._reset_scene()

        self.assertEqual(runtime._timeline.pause_calls, 1)
        self.assertEqual(robot_positions[-1], runtime._plan.home_dof_positions)
        self.assertEqual(physics_resets, ["reset"])
        self.assertIsNone(runtime._found_camera_point)
        self.assertIsNone(runtime._found_world_point)
        self.assertFalse(runtime._stop_requested)
        self.assertFalse(runtime._scenario_active)
        self.assertEqual(runtime._grasp_check_frames_remaining, 0)
        self.assertFalse(runtime._settle_monitor_active)
        self.assertEqual(runtime._settle_stable_frames, 0)

    def test_reset_physics_scene_state_restores_tomato_before_reattaching_stem_joint(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._stage = object()  # type: ignore[assignment]
        removed_joints: list[str] = []
        runtime._remove_physics_joint = lambda joint_path: removed_joints.append(joint_path)  # type: ignore[method-assign]
        tomato_positions: list[tuple[float, float, float]] = []
        runtime._set_tomato_world_position = lambda position_m: tomato_positions.append(position_m)  # type: ignore[method-assign]
        pump_frames: list[int] = []
        runtime._pump_updates = lambda frame_count: pump_frames.append(frame_count)  # type: ignore[method-assign]
        ensured: list[str] = []
        runtime._ensure_fruit_stem_joint = lambda: ensured.append("stem")  # type: ignore[method-assign]
        kinematic_modes: list[bool] = []
        runtime._set_tomato_kinematic_enabled = lambda enabled: kinematic_modes.append(enabled)  # type: ignore[method-assign]

        class _FakeBody:
            def __init__(self) -> None:
                self.linear: list[tuple[float, float, float]] = []
                self.angular: list[tuple[float, float, float]] = []

            def set_linear_velocity(self, value: np.ndarray) -> None:
                self.linear.append(tuple(float(v) for v in value))

            def set_angular_velocity(self, value: np.ndarray) -> None:
                self.angular.append(tuple(float(v) for v in value))

        fake_body = _FakeBody()
        runtime._tomato_body = fake_body  # type: ignore[assignment]

        runtime._reset_physics_scene_state()

        self.assertEqual(removed_joints, [runtime._fruit_hand_joint_path, runtime._fruit_stem_joint_path])
        self.assertEqual(tomato_positions, [runtime._plan.tomato_initial_world_m])
        self.assertEqual(fake_body.linear, [(0.0, 0.0, 0.0)])
        self.assertEqual(fake_body.angular, [(0.0, 0.0, 0.0)])
        self.assertEqual(kinematic_modes, [True])
        self.assertEqual(pump_frames, [2, 1])
        self.assertEqual(ensured, ["stem"])

    def test_start_scenario_resumes_timeline(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._timeline = _FakeTimeline()
        preparations: list[str] = []
        runtime._prepare_scene_for_start = lambda: preparations.append("prepared")  # type: ignore[method-assign]
        runtime._pump_updates = lambda frame_count: None  # type: ignore[method-assign]
        runtime._set_phase = lambda phase, message: None  # type: ignore[method-assign]
        camera_switches: list[str] = []
        runtime._set_camera_view = lambda camera_view: camera_switches.append(camera_view.value)  # type: ignore[method-assign]
        queued: list[int] = []
        runtime._queue_scan_pose = lambda index: queued.append(index)  # type: ignore[method-assign]

        runtime._start_scenario()

        self.assertEqual(preparations, ["prepared"])
        self.assertEqual(runtime._timeline.play_calls, 1)
        self.assertEqual(queued, [0])
        self.assertEqual(camera_switches, [])

    def test_prepare_scene_for_start_returns_robot_home_without_resetting_tomato(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._timeline = _FakeTimeline()
        robot_positions: list[tuple[float, ...]] = []
        runtime._set_articulation_positions = lambda positions: robot_positions.append(tuple(positions))  # type: ignore[method-assign]
        physics_resets: list[str] = []
        runtime._reset_physics_scene_state = lambda: physics_resets.append("reset")  # type: ignore[method-assign]
        removed_joints: list[str] = []
        runtime._remove_physics_joint = lambda joint_path: removed_joints.append(joint_path)  # type: ignore[method-assign]
        runtime._pump_updates = lambda frame_count: None  # type: ignore[method-assign]
        runtime._fruit_hand_joint_active = True
        runtime._fruit_hand_joint_path = "/World/Joints/FruitHandJoint"
        runtime._found_camera_point = (1.0, 1.0, 1.0)
        runtime._found_world_point = (2.0, 2.0, 2.0)
        runtime._stop_requested = True
        runtime._scenario_active = True
        runtime._animation = object()  # type: ignore[assignment]

        runtime._prepare_scene_for_start()

        self.assertEqual(runtime._timeline.pause_calls, 1)
        self.assertEqual(robot_positions[-1], runtime._plan.home_dof_positions)
        self.assertEqual(physics_resets, [])
        self.assertEqual(removed_joints, ["/World/Joints/FruitHandJoint"])
        self.assertFalse(runtime._fruit_hand_joint_active)
        self.assertIsNone(runtime._found_camera_point)
        self.assertIsNone(runtime._found_world_point)

    def test_hand_target_position_uses_grasp_center_offset(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._top_down_grasp_center_offset_world = np.array([0.0, 0.0, 0.1034], dtype=float)

        target_position = runtime._compute_hand_target_position((0.50, 0.00, 0.42))

        self.assertEqual(target_position, (0.5, 0.0, 0.3166))

    def test_scan_pose_does_not_force_target_found_when_tomato_is_not_visible(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        moved_positions: list[tuple[float, float, float]] = []
        runtime._set_tomato_world_position = lambda position_m: moved_positions.append(position_m)  # type: ignore[method-assign]
        runtime._read_tomato_positions = lambda: ((0.80, 0.80, 2.00), (0.50, 0.00, 0.42))  # type: ignore[method-assign]
        queued: list[int] = []
        runtime._queue_scan_pose = lambda index: queued.append(index)  # type: ignore[method-assign]

        runtime._complete_scan_pose(3)

        self.assertEqual(moved_positions, [])
        self.assertEqual(queued, [4])
        self.assertIsNone(runtime._found_camera_point)
        self.assertIsNone(runtime._found_world_point)

    def test_begin_grasp_assessment_does_not_teleport_tomato(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._found_world_point = (0.50, 0.00, 0.42)
        moved_positions: list[tuple[float, float, float]] = []
        runtime._set_tomato_world_position = lambda position_m: moved_positions.append(position_m)  # type: ignore[method-assign]

        runtime._begin_grasp_assessment()

        self.assertEqual(moved_positions, [])
        self.assertEqual(runtime._grasp_contact_stable_frames, 0)
        self.assertEqual(runtime._grasp_check_frames_remaining, runtime._plan.grasp_hold_frame_count * 4)

    def test_runtime_maintains_stem_attachment_before_grasp(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        maintained: list[str] = []
        runtime._maintain_stem_attachment = lambda: maintained.append("maintained")  # type: ignore[method-assign]
        runtime._fruit_stem_joint_active = True
        runtime._fruit_hand_joint_active = False

        runtime._update_runtime()

        self.assertEqual(maintained, ["maintained"])

    def test_ensure_fruit_stem_joint_enables_kinematic_hold(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        kinematic_modes: list[bool] = []
        runtime._set_tomato_kinematic_enabled = lambda enabled: kinematic_modes.append(enabled)  # type: ignore[method-assign]
        maintained: list[str] = []
        runtime._maintain_stem_attachment = lambda: maintained.append("maintained")  # type: ignore[method-assign]

        runtime._ensure_fruit_stem_joint()

        self.assertTrue(runtime._fruit_stem_joint_active)
        self.assertEqual(kinematic_modes, [True])
        self.assertEqual(maintained, ["maintained"])

    def test_finalize_grasp_disables_kinematic_hold_after_hand_joint_creation(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._create_hand_grasp_joint = lambda: True  # type: ignore[method-assign]
        kinematic_modes: list[bool] = []
        runtime._set_tomato_kinematic_enabled = lambda enabled: kinematic_modes.append(enabled)  # type: ignore[method-assign]
        queued: list[str] = []
        runtime._queue_pull_motion = lambda: queued.append("pull")  # type: ignore[method-assign]
        runtime._fruit_stem_joint_active = True

        runtime._finalize_grasp()

        self.assertEqual(kinematic_modes, [False])
        self.assertFalse(runtime._fruit_stem_joint_active)
        self.assertEqual(queued, ["pull"])

    def test_physical_grasp_requires_dual_finger_contact(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._get_articulation_positions = lambda: np.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.02), dtype=float)  # type: ignore[method-assign]
        runtime._read_finger_contact_forces = lambda: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))  # type: ignore[method-assign]

        self.assertFalse(runtime._has_physical_grasp())

        runtime._get_articulation_positions = lambda: np.array((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), dtype=float)  # type: ignore[method-assign]
        runtime._read_finger_contact_forces = lambda: ((0.0, 0.0, 2.2), (0.0, 0.0, 2.3))  # type: ignore[method-assign]

        self.assertTrue(runtime._has_physical_grasp())

    def test_release_tomato_removes_grasp_joint_and_queues_retreat(self) -> None:
        runtime = IsaacNativeRuntime(headless=True)
        runtime._top_down_hand_orientation = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        removed_joints: list[str] = []
        runtime._remove_physics_joint = lambda joint_path: removed_joints.append(joint_path)  # type: ignore[method-assign]
        runtime._compute_hand_target_position = lambda position_m: position_m  # type: ignore[method-assign]
        runtime._solve_ik_joint_positions = lambda position_m, target_orientation=None: np.array(position_m, dtype=float)  # type: ignore[method-assign]
        queued: list[tuple[tuple[float, ...], int, object | None]] = []
        runtime._queue_joint_animation = lambda *, target_positions, frames, on_complete: queued.append((tuple(target_positions), frames, on_complete))  # type: ignore[method-assign]
        runtime._set_phase = lambda phase, message: None  # type: ignore[method-assign]
        runtime._fruit_hand_joint_path = "/World/Joints/FruitHandJoint"
        runtime._fruit_hand_joint_active = True

        runtime._release_tomato_and_retreat()

        self.assertEqual(removed_joints, ["/World/Joints/FruitHandJoint"])
        self.assertFalse(runtime._fruit_hand_joint_active)
        self.assertEqual(queued[0][0], (0.35, -0.45, 0.545))
        self.assertEqual(queued[0][1], 50)


if __name__ == "__main__":
    unittest.main()
