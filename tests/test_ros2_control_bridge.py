from __future__ import annotations

import unittest
from typing import get_type_hints

import numpy as np

from tomato_harvest_sim.api.contracts import JointTrajectory, JointTrajectoryPoint, Pose3D
from tomato_harvest_sim.api.hardware_control import HardwareCommandSample, HardwareStateSample
from tomato_harvest_sim.api.trajectory_execution import TrajectoryExecutionRequest, TrajectoryExecutionState
from tomato_harvest_sim.robot.ros2_control import JointTrajectoryControllerBridge


class _Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class _Hardware:
    def __init__(self, clock: _Clock) -> None:
        self._clock = clock
        self.positions = np.zeros(9, dtype=float)
        self.velocities = np.zeros(9, dtype=float)
        self.commands: list[HardwareCommandSample] = []

    def initialize_if_needed(self) -> bool:
        return True

    def read_state(self) -> HardwareStateSample | None:
        return HardwareStateSample(
            joint_names=(
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
                "finger_left",
                "finger_right",
            ),
            positions_rad=tuple(float(value) for value in self.positions),
            velocities_rad_s=tuple(float(value) for value in self.velocities),
            timestamp_sec=self._clock(),
        )

    def write_command(self, command: HardwareCommandSample) -> None:
        self.commands.append(command)
        if command.positions_rad is not None:
            self.positions = np.asarray(command.positions_rad, dtype=float).copy()
        if command.velocities_rad_s is not None:
            self.velocities = np.asarray(command.velocities_rad_s, dtype=float).copy()


class _LaggingHardware(_Hardware):
    def __init__(self, clock: _Clock, *, tracking_gain: float) -> None:
        super().__init__(clock)
        self._tracking_gain = tracking_gain

    def write_command(self, command: HardwareCommandSample) -> None:
        self.commands.append(command)
        if command.positions_rad is not None:
            target_positions = np.asarray(command.positions_rad, dtype=float)
            self.positions = self.positions + (target_positions - self.positions) * self._tracking_gain
        if command.velocities_rad_s is not None:
            self.velocities = np.asarray(command.velocities_rad_s, dtype=float).copy()


class _PoseAwareHardware(_Hardware):
    def __init__(self, clock: _Clock) -> None:
        super().__init__(clock)
        self.end_effector_pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)

    def read_state(self) -> HardwareStateSample | None:
        return HardwareStateSample(
            joint_names=(
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
                "finger_left",
                "finger_right",
            ),
            positions_rad=tuple(float(value) for value in self.positions),
            velocities_rad_s=tuple(float(value) for value in self.velocities),
            timestamp_sec=self._clock(),
            end_effector_pose=self.end_effector_pose,
        )


class JointTrajectoryControllerBridgeTest(unittest.TestCase):
    def test_terminal_hold_command_type_hints_are_resolvable(self) -> None:
        hints = get_type_hints(JointTrajectoryControllerBridge._write_terminal_hold_command)

        self.assertIn("hardware_state", hints)

    def test_bridge_interpolates_position_and_velocity_commands(self) -> None:
        clock = _Clock()
        hardware = _Hardware(clock)
        bridge = JointTrajectoryControllerBridge(
            hardware=hardware,
            monotonic_time_sec=clock,
            path_tolerance_rad=2.5,
        )
        trajectory = JointTrajectory(
            joint_names=(
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ),
            points=(JointTrajectoryPoint((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 0.1),),
        )

        accepted = bridge.send_goal(
            TrajectoryExecutionRequest(
                controller_name="joint_trajectory_controller",
                command_name="move_to_pregrasp",
                planner_name="moveit2_service_bridge",
                trajectory=trajectory,
                gripper_closed=False,
            )
        )

        self.assertTrue(accepted)

        for timestamp_sec in (0.0, 0.10, 0.40, 0.80, 1.10, 1.20):
            clock.now = timestamp_sec
            bridge.step()

        self.assertGreaterEqual(len(hardware.commands), 3)
        self.assertTrue(any(command.velocities_rad_s is not None for command in hardware.commands))
        self.assertEqual(bridge.current_result().state, TrajectoryExecutionState.SUCCEEDED)

    def test_bridge_clamps_arm_velocity_to_joint_limits(self) -> None:
        clock = _Clock()
        hardware = _Hardware(clock)
        bridge = JointTrajectoryControllerBridge(
            hardware=hardware,
            monotonic_time_sec=clock,
            path_tolerance_rad=10.0,
        )
        trajectory = JointTrajectory(
            joint_names=(
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ),
            points=(JointTrajectoryPoint((10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.05),),
        )

        self.assertTrue(
            bridge.send_goal(
                TrajectoryExecutionRequest(
                    controller_name="joint_trajectory_controller",
                    command_name="move_to_pregrasp",
                    planner_name="moveit2_service_bridge",
                    trajectory=trajectory,
                    gripper_closed=False,
                )
            )
        )

        clock.now = 0.01
        bridge.step()

        first_velocity = np.asarray(hardware.commands[0].velocities_rad_s, dtype=float)
        self.assertLessEqual(abs(float(first_velocity[0])), 2.175 + 1e-6)

    def test_bridge_forwards_interpolated_reference_without_feedback_correction(self) -> None:
        clock = _Clock()
        hardware = _Hardware(clock)
        hardware.positions[:7] = np.asarray((0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), dtype=float)
        bridge = JointTrajectoryControllerBridge(
            hardware=hardware,
            monotonic_time_sec=clock,
            path_tolerance_rad=10.0,
            proportional_gain=0.0,
            derivative_gain=0.0,
        )
        trajectory = JointTrajectory(
            joint_names=(
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ),
            points=(JointTrajectoryPoint((0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.1),),
        )

        self.assertTrue(
            bridge.send_goal(
                TrajectoryExecutionRequest(
                    controller_name="joint_trajectory_controller",
                    command_name="move_to_pregrasp",
                    planner_name="moveit2_service_bridge",
                    trajectory=trajectory,
                    gripper_closed=False,
                )
            )
        )

        clock.now = 0.0
        bridge.step()
        clock.now = 0.05
        bridge.step()

        second_command = hardware.commands[1]
        self.assertIsNotNone(second_command.positions_rad)
        self.assertIsNotNone(second_command.velocities_rad_s)
        self.assertAlmostEqual(float(second_command.positions_rad[0]), 0.125)
        self.assertAlmostEqual(float(second_command.velocities_rad_s[0]), 1.5)

    def test_bridge_applies_pd_correction_to_velocity_when_lagging(self) -> None:
        """位置誤差がある場合、velocity_cmd = v_ff + Kp*e_pos + Kd*e_vel となる"""
        clock = _Clock()
        hardware = _Hardware(clock)
        # 実位置 0.05 rad、参照は t=0.05 で 0.125 rad になる予定 → e_pos = 0.075
        hardware.positions[:7] = np.asarray((0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), dtype=float)
        Kp = 1.0
        Kd = 0.0  # 速度誤差の影響を除くため Kd=0 で確認
        bridge = JointTrajectoryControllerBridge(
            hardware=hardware,
            monotonic_time_sec=clock,
            path_tolerance_rad=10.0,
            proportional_gain=Kp,
            derivative_gain=Kd,
        )
        trajectory = JointTrajectory(
            joint_names=("panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                         "panda_joint5", "panda_joint6", "panda_joint7"),
            points=(JointTrajectoryPoint((0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.1),),
        )
        self.assertTrue(bridge.send_goal(TrajectoryExecutionRequest(
            controller_name="joint_trajectory_controller",
            command_name="move_to_pregrasp",
            planner_name="moveit2_service_bridge",
            trajectory=trajectory,
            gripper_closed=False,
        )))

        clock.now = 0.0
        bridge.step()
        # t=0.0 コマンドで positions=[0.05], velocities=[1.5] がセットされる
        # → 次 step では e_pos=0.075, e_vel=0 になる

        clock.now = 0.05
        bridge.step()

        second_command = hardware.commands[1]
        # 参照位置は変わらない
        self.assertAlmostEqual(float(second_command.positions_rad[0]), 0.125)
        # 速度には Kp*e_pos が加算される (v_ff=1.5, e_pos=0.075 → 1.575)
        self.assertGreater(float(second_command.velocities_rad_s[0]), 1.5)
        expected_vel = 1.5 + Kp * 0.075  # e_pos = 0.125 - 0.05 = 0.075
        self.assertAlmostEqual(float(second_command.velocities_rad_s[0]), expected_vel, places=3)

    def test_bridge_does_not_abort_while_joint_error_keeps_improving(self) -> None:
        clock = _Clock()
        hardware = _LaggingHardware(clock, tracking_gain=0.35)
        bridge = JointTrajectoryControllerBridge(
            hardware=hardware,
            monotonic_time_sec=clock,
            goal_time_tolerance_sec=0.05,
            path_tolerance_rad=10.0,
        )
        trajectory = JointTrajectory(
            joint_names=(
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ),
            points=(JointTrajectoryPoint((0.6, -0.4, 0.3, -1.9, 0.25, 1.8, 0.9), 0.1),),
        )

        self.assertTrue(
            bridge.send_goal(
                TrajectoryExecutionRequest(
                    controller_name="joint_trajectory_controller",
                    command_name="move_to_grasp",
                    planner_name="moveit2_service_bridge",
                    trajectory=trajectory,
                    gripper_closed=False,
                )
            )
        )

        for timestamp_sec in (0.0, 0.05, 0.10, 0.16, 0.24, 0.32, 0.40, 0.48, 0.56, 0.64, 0.80, 1.00, 1.20, 1.40, 1.60, 1.80, 2.00, 2.20, 2.40, 2.60):
            clock.now = timestamp_sec
            bridge.step()

        self.assertIsNotNone(bridge.current_result())
        self.assertNotEqual(bridge.current_result().state, TrajectoryExecutionState.ABORTED)
        self.assertEqual(bridge.current_result().state, TrajectoryExecutionState.SUCCEEDED)

    def test_bridge_advances_to_next_segment_when_actual_target_is_already_reached(self) -> None:
        clock = _Clock()
        hardware = _Hardware(clock)
        hardware.positions[:7] = np.asarray((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), dtype=float)
        bridge = JointTrajectoryControllerBridge(
            hardware=hardware,
            monotonic_time_sec=clock,
            path_tolerance_rad=10.0,
        )
        trajectory = JointTrajectory(
            joint_names=(
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ),
            points=(
                JointTrajectoryPoint((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 0.5),
                JointTrajectoryPoint((0.3, -0.1, 0.2, -1.8, 0.3, 1.7, 1.0), 1.0),
            ),
        )

        self.assertTrue(
            bridge.send_goal(
                TrajectoryExecutionRequest(
                    controller_name="joint_trajectory_controller",
                    command_name="move_to_pregrasp",
                    planner_name="moveit2_service_bridge",
                    trajectory=trajectory,
                    gripper_closed=False,
                )
            )
        )

        clock.now = 0.0
        bridge.step()

        self.assertEqual(bridge.active_segment_index, 1)

    def test_bridge_waits_for_end_effector_target_before_reporting_success(self) -> None:
        clock = _Clock()
        hardware = _PoseAwareHardware(clock)
        hardware.positions[:7] = np.asarray((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), dtype=float)
        bridge = JointTrajectoryControllerBridge(
            hardware=hardware,
            monotonic_time_sec=clock,
            path_tolerance_rad=10.0,
            goal_time_tolerance_sec=0.5,
        )
        trajectory = JointTrajectory(
            joint_names=(
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ),
            points=(JointTrajectoryPoint((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 0.1),),
        )

        self.assertTrue(
            bridge.send_goal(
                TrajectoryExecutionRequest(
                    controller_name="joint_trajectory_controller",
                    command_name="move_to_pregrasp",
                    planner_name="moveit2_service_bridge",
                    trajectory=trajectory,
                    target_pose=Pose3D(0.5, 0.0, 0.63, 180.0, 0.0, 0.0),
                    position_tolerance_m=0.03,
                    gripper_closed=False,
                )
            )
        )

        clock.now = 0.0
        hardware.end_effector_pose = Pose3D(0.49, -0.023, 0.655, 180.0, 0.0, 0.0)
        bridge.step()

        self.assertIsNone(bridge.current_result())
        self.assertEqual(hardware.commands[-1].context, "move_to_pregrasp:ee_settle")
        self.assertAlmostEqual(float(hardware.commands[-1].positions_rad[0]), 0.2)

        clock.now = 0.1
        hardware.end_effector_pose = Pose3D(0.50, 0.0, 0.629, 180.0, 0.0, 0.0)
        bridge.step()

        self.assertEqual(bridge.current_result().state, TrajectoryExecutionState.SUCCEEDED)


    def test_bridge_applies_terminal_damping_after_goal_succeeds(self) -> None:
        """軌道完了後も step() のたびに position=target, velocity=-Kd*v_actual を送り続ける"""
        clock = _Clock()
        hardware = _Hardware(clock)
        hardware.positions[:7] = np.asarray((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), dtype=float)
        Kd = 0.1
        bridge = JointTrajectoryControllerBridge(
            hardware=hardware,
            monotonic_time_sec=clock,
            path_tolerance_rad=10.0,
            proportional_gain=1.0,
            derivative_gain=Kd,
        )
        trajectory = JointTrajectory(
            joint_names=("panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                         "panda_joint5", "panda_joint6", "panda_joint7"),
            points=(JointTrajectoryPoint((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 0.1),),
        )
        bridge.send_goal(TrajectoryExecutionRequest(
            controller_name="joint_trajectory_controller",
            command_name="move_to_pregrasp",
            planner_name="moveit2_service_bridge",
            trajectory=trajectory,
            gripper_closed=False,
        ))
        clock.now = 0.0
        bridge.step()
        # 目標位置に到着して SUCCEEDED
        self.assertEqual(bridge.current_result().state, TrajectoryExecutionState.SUCCEEDED)

        # SUCCEEDED 後、残留速度をシミュレート
        hardware.velocities[:7] = np.full(7, 0.05)

        pre_count = len(hardware.commands)
        clock.now = 0.2
        bridge.step()  # active_request=None だが terminal damping が動く

        self.assertGreater(len(hardware.commands), pre_count, "SUCCEEDED 後も step() でコマンドが送られるべき")
        last_cmd = hardware.commands[-1]
        self.assertIsNotNone(last_cmd.velocities_rad_s)
        # velocity=-Kd*v_actual: 速度が負（減衰方向）になっているはず
        self.assertAlmostEqual(float(last_cmd.velocities_rad_s[0]), -Kd * 0.05, places=5)
        # position は常に trajectory target
        self.assertAlmostEqual(float(last_cmd.positions_rad[0]), 0.2)

    def test_terminal_damping_clears_when_new_send_goal_arrives(self) -> None:
        """send_goal() が来たら terminal damping キャッシュがリセットされる"""
        clock = _Clock()
        hardware = _Hardware(clock)
        hardware.positions[:7] = np.zeros(7, dtype=float)
        bridge = JointTrajectoryControllerBridge(
            hardware=hardware,
            monotonic_time_sec=clock,
            path_tolerance_rad=10.0,
        )
        trajectory = JointTrajectory(
            joint_names=("panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                         "panda_joint5", "panda_joint6", "panda_joint7"),
            points=(JointTrajectoryPoint((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.1),),
        )
        bridge.send_goal(TrajectoryExecutionRequest(
            controller_name="joint_trajectory_controller",
            command_name="move1",
            planner_name="moveit2_service_bridge",
            trajectory=trajectory,
            gripper_closed=False,
        ))
        clock.now = 0.0
        bridge.step()
        self.assertEqual(bridge.current_result().state, TrajectoryExecutionState.SUCCEEDED)

        # 新しいゴールを送る
        bridge.send_goal(TrajectoryExecutionRequest(
            controller_name="joint_trajectory_controller",
            command_name="move2",
            planner_name="moveit2_service_bridge",
            trajectory=JointTrajectory(
                joint_names=("panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                             "panda_joint5", "panda_joint6", "panda_joint7"),
                points=(JointTrajectoryPoint((0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.1),),
            ),
            gripper_closed=False,
        ))
        self.assertIsNone(bridge.current_result(), "新ゴール後は result がリセットされる")
        self.assertIsNotNone(bridge.active_request(), "新ゴールが有効化されている")


if __name__ == "__main__":
    unittest.main()
