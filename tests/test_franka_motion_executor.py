from __future__ import annotations

import io
import unittest
from contextlib import redirect_stdout

import numpy as np

from tomato_harvest_sim.api.contracts import JointTrajectory, JointTrajectoryPoint, Pose3D, ScenePhase, SceneSnapshot, TomatoStatus
from tomato_harvest_sim.api.hardware_control import HardwareControlPort, HardwareStateSample
from tomato_harvest_sim.robot.trajectory_tracking import FrankaTrajectoryExecutionManager, is_pose_reached, pose_distance_m
from tomato_harvest_sim.robot.trajectory_tracking.execution import _hand_pose_from_grasp_center_pose
from tomato_harvest_sim.simulator.isaac_franka_driver import IsaacFrankaDriver
from tomato_harvest_sim.simulator.isaac_ros2_control_system import IsaacRos2ControlSystem


class IsaacFrankaMotionExecutor(FrankaTrajectoryExecutionManager):
    def __init__(
        self,
        *,
        robot_prim_path: str,
        position_tolerance_m: float = 0.03,
        max_joint_step_rad: float = 0.05,
        max_gripper_step_rad: float = 0.01,
        joint_tolerance_rad: float = 0.03,
    ) -> None:
        driver = IsaacFrankaDriver(robot_prim_path=robot_prim_path)
        super().__init__(
            driver=driver,
            hardware_control_port=IsaacRos2ControlSystem(driver=driver),
            position_tolerance_m=position_tolerance_m,
            max_joint_step_rad=max_joint_step_rad,
            max_gripper_step_rad=max_gripper_step_rad,
            joint_tolerance_rad=joint_tolerance_rad,
        )

    @property
    def _trajectory_debug_enabled(self) -> bool:
        return getattr(self, "_compat_trajectory_debug_enabled", False)

    @_trajectory_debug_enabled.setter
    def _trajectory_debug_enabled(self, value: bool) -> None:
        self._compat_trajectory_debug_enabled = bool(value)
        self._driver._trajectory_debug_enabled = bool(value)

    @property
    def _articulation(self) -> object | None:
        return getattr(self._driver, "_articulation", None)

    @_articulation.setter
    def _articulation(self, value: object | None) -> None:
        self._driver._articulation = value

    @property
    def _articulation_kinematics_solver(self) -> object | None:
        return getattr(self._driver, "_articulation_kinematics_solver", None)

    @_articulation_kinematics_solver.setter
    def _articulation_kinematics_solver(self, value: object | None) -> None:
        self._driver._articulation_kinematics_solver = value

    @property
    def _kinematics_solver(self) -> object | None:
        return getattr(self._driver, "_kinematics_solver", None)

    @_kinematics_solver.setter
    def _kinematics_solver(self, value: object | None) -> None:
        self._driver._kinematics_solver = value

    @property
    def _home_joint_positions(self) -> np.ndarray | None:
        return getattr(self._driver, "_home_joint_positions", None)

    @_home_joint_positions.setter
    def _home_joint_positions(self, value: np.ndarray | None) -> None:
        self._driver._home_joint_positions = value

    @property
    def _initialized(self) -> bool:
        return bool(getattr(self._driver, "_initialized", False))

    @_initialized.setter
    def _initialized(self, value: bool) -> None:
        self._driver._initialized = bool(value)


class FrankaMotionExecutorTest(unittest.TestCase):
    def test_preview_end_effector_path_for_joint_trajectory_is_cached(self) -> None:
        class _PreviewDriver:
            def __init__(self) -> None:
                self.preview_calls = 0

            def initialize_if_needed(self) -> bool:
                return True

            def current_joint_positions(self) -> np.ndarray | None:
                return np.zeros(9, dtype=float)

            def current_joint_velocities(self) -> np.ndarray | None:
                return np.zeros(9, dtype=float)

            def current_end_effector_pose(self) -> Pose3D | None:
                return Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)

            def current_joint_state_snapshot(self) -> object | None:
                return None

            def home_joint_positions(self) -> np.ndarray | None:
                return np.zeros(9, dtype=float)

            def expand_joint_targets(self, joint_positions: np.ndarray) -> np.ndarray:
                return np.asarray(joint_positions, dtype=float)

            def solve_joint_targets_for_pose(self, target_pose: Pose3D, *, position_tolerance_m: float) -> np.ndarray | None:
                del target_pose, position_tolerance_m
                return None

            def set_joint_positions_with_debug(self, positions: np.ndarray, *, context: str) -> None:
                del positions, context

            def set_joint_velocity_targets_with_debug(
                self,
                *,
                positions: np.ndarray,
                velocities: np.ndarray,
                context: str,
            ) -> None:
                del positions, velocities, context

            def preview_end_effector_path_for_joint_trajectory(self, trajectory: JointTrajectory) -> tuple[Pose3D, ...]:
                del trajectory
                self.preview_calls += 1
                return (
                    Pose3D(0.10, 0.00, 0.50, 180.0, 0.0, 0.0),
                    Pose3D(0.20, 0.00, 0.55, 180.0, 0.0, 0.0),
                )

        trajectory = JointTrajectory(
            joint_names=("panda_joint1",),
            points=(
                JointTrajectoryPoint((0.0,), 0.0),
                JointTrajectoryPoint((0.1,), 1.0),
            ),
        )
        driver = _PreviewDriver()
        
        class _PreviewHardware(HardwareControlPort):
            def initialize_if_needed(self) -> bool:
                return True

            def read_state(self) -> HardwareStateSample | None:
                return HardwareStateSample(
                    joint_names=("panda_joint1",),
                    positions_rad=(0.0,),
                    velocities_rad_s=(0.0,),
                    timestamp_sec=0.0,
                    end_effector_pose=Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0),
                    joint_state_snapshot=None,
                )

            def write_command(self, command) -> None:
                del command

        executor = FrankaTrajectoryExecutionManager(driver=driver, hardware_control_port=_PreviewHardware())

        first_preview = executor.preview_end_effector_path_for_joint_trajectory(trajectory)
        second_preview = executor.preview_end_effector_path_for_joint_trajectory(trajectory)

        self.assertEqual(first_preview, second_preview)
        self.assertEqual(driver.preview_calls, 1)

    def test_executor_prefers_apply_action_when_articulation_supports_it(self) -> None:
        class _ActionCapableFakeArticulation:
            def __init__(self) -> None:
                self.positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.04], dtype=float)
                self.apply_action_calls = 0
                self.set_joint_positions_calls = 0
                self.last_action: object | None = None

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

            def set_joint_positions(self, positions: np.ndarray) -> None:
                self.set_joint_positions_calls += 1
                self.positions = np.asarray(positions, dtype=float).copy()

            def apply_action(self, action: object) -> None:
                self.apply_action_calls += 1
                self.last_action = action
                self.positions = np.asarray(getattr(action, "joint_positions"), dtype=float).copy()

        class _TrajectoryExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka", max_joint_step_rad=1.0)
                self._initialized = True
                self._articulation = _ActionCapableFakeArticulation()

            def _initialize_if_needed(self) -> bool:
                return True

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=True,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=JointTrajectory(
                joint_names=(
                    "panda_joint1",
                    "panda_joint2",
                    "panda_joint3",
                    "panda_joint4",
                    "panda_joint5",
                    "panda_joint6",
                    "panda_joint7",
                ),
                points=(JointTrajectoryPoint((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 1.0),),
            ),
        )

        executor = _TrajectoryExecutor()
        executor.sync_with_snapshot(snapshot)
        executor.step()

        self.assertGreaterEqual(executor._articulation.apply_action_calls, 1)
        self.assertEqual(executor._articulation.set_joint_positions_calls, 0)
        self.assertIsNotNone(executor._articulation.last_action)
        self.assertIsNotNone(getattr(executor._articulation.last_action, "joint_velocities", None))

    def test_executor_merges_gripper_and_arm_command_during_joint_trajectory(self) -> None:
        class _ActionCapableFakeArticulation:
            def __init__(self) -> None:
                self.positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.04], dtype=float)
                self.apply_action_calls = 0
                self.last_joint_positions: np.ndarray | None = None
                self.last_joint_velocities: np.ndarray | None = None

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

            def apply_action(self, action: object) -> None:
                self.apply_action_calls += 1
                self.last_joint_positions = np.asarray(getattr(action, "joint_positions"), dtype=float).copy()
                self.last_joint_velocities = np.asarray(getattr(action, "joint_velocities"), dtype=float).copy()
                self.positions = self.last_joint_positions.copy()

        class _TrajectoryExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka", max_joint_step_rad=1.0, max_gripper_step_rad=0.01)
                self._initialized = True
                self._articulation = _ActionCapableFakeArticulation()

            def _initialize_if_needed(self) -> bool:
                return True

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=True,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=JointTrajectory(
                joint_names=(
                    "panda_joint1",
                    "panda_joint2",
                    "panda_joint3",
                    "panda_joint4",
                    "panda_joint5",
                    "panda_joint6",
                    "panda_joint7",
                ),
                points=(JointTrajectoryPoint((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 1.0),),
            ),
        )

        executor = _TrajectoryExecutor()
        executor.sync_with_snapshot(snapshot)
        executor.step()

        self.assertGreaterEqual(executor._articulation.apply_action_calls, 1)
        self.assertIsNotNone(executor._articulation.last_joint_positions)
        self.assertIsNotNone(executor._articulation.last_joint_velocities)
        # PD補正が乗るため厳密一致ではなく、正方向の合理的な速度が出ていることを確認する
        self.assertGreater(float(executor._articulation.last_joint_velocities[0]), 0.1)
        self.assertAlmostEqual(float(executor._articulation.last_joint_positions[7]), 0.0, places=6)
        self.assertAlmostEqual(float(executor._articulation.last_joint_positions[8]), 0.0, places=6)

    def test_executor_advances_trajectory_when_only_fingers_differ(self) -> None:
        class _ActionCapableFakeArticulation:
            def __init__(self) -> None:
                self.positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.04], dtype=float)
                self.apply_action_calls = 0

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

            def apply_action(self, action: object) -> None:
                self.apply_action_calls += 1
                self.positions = np.asarray(getattr(action, "joint_positions"), dtype=float).copy()

        class _TrajectoryExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka", max_joint_step_rad=1.0, max_gripper_step_rad=0.01)
                self._initialized = True
                self._articulation = _ActionCapableFakeArticulation()

            def _initialize_if_needed(self) -> bool:
                return True

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        point_one = (0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9)
        point_two = (0.3, -0.1, 0.2, -1.8, 0.3, 1.7, 1.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=True,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=JointTrajectory(
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
                    JointTrajectoryPoint(point_one, 0.5),
                    JointTrajectoryPoint(point_two, 1.0),
                ),
            ),
        )

        executor = _TrajectoryExecutor()
        executor.sync_with_snapshot(snapshot)
        executor._articulation.positions[:7] = np.array(point_one, dtype=float)
        executor._articulation.positions[7:9] = np.array([0.03, 0.03], dtype=float)

        executor.step()

        self.assertEqual(executor._active_trajectory_point_index, 1)

    def test_executor_keeps_closing_gripper_after_joint_trajectory_reaches_final_point(self) -> None:
        class _ActionCapableFakeArticulation:
            def __init__(self) -> None:
                self.positions = np.array([0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9, 0.04, 0.04], dtype=float)
                self.apply_action_calls = 0
                self.last_joint_positions: np.ndarray | None = None

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

            def apply_action(self, action: object) -> None:
                self.apply_action_calls += 1
                self.last_joint_positions = np.asarray(getattr(action, "joint_positions"), dtype=float).copy()
                self.positions = self.last_joint_positions.copy()

        class _TrajectoryExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka", max_joint_step_rad=1.0, max_gripper_step_rad=0.01)
                self._initialized = True
                self._articulation = _ActionCapableFakeArticulation()

            def _initialize_if_needed(self) -> bool:
                return True

            def _get_end_effector_pose(self) -> Pose3D | None:
                return Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0)

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=True,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=JointTrajectory(
                joint_names=(
                    "panda_joint1",
                    "panda_joint2",
                    "panda_joint3",
                    "panda_joint4",
                    "panda_joint5",
                    "panda_joint6",
                    "panda_joint7",
                ),
                points=(JointTrajectoryPoint((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 1.0),),
            ),
        )

        executor = _TrajectoryExecutor()
        executor.sync_with_snapshot(snapshot)
        first_log = executor.step()
        second_log = executor.step()

        self.assertIn("accepted joint trajectory", first_log)
        self.assertIn("Franka trajectory completed", second_log)
        self.assertGreaterEqual(executor._articulation.apply_action_calls, 1)
        self.assertIsNotNone(executor._articulation.last_joint_positions)
        self.assertAlmostEqual(float(executor._articulation.last_joint_positions[7]), 0.0, places=6)
        self.assertAlmostEqual(float(executor._articulation.last_joint_positions[8]), 0.0, places=6)

    def test_executor_advances_trajectory_when_arm_error_is_within_default_tolerance(self) -> None:
        class _ActionCapableFakeArticulation:
            def __init__(self) -> None:
                self.positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.04], dtype=float)
                self.apply_action_calls = 0

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

            def apply_action(self, action: object) -> None:
                self.apply_action_calls += 1
                self.positions = np.asarray(getattr(action, "joint_positions"), dtype=float).copy()

        class _TrajectoryExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka")
                self._initialized = True
                self._articulation = _ActionCapableFakeArticulation()

            def _initialize_if_needed(self) -> bool:
                return True

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        point_one = (0.20, -0.20, 0.10, -1.90, 0.20, 1.80, 0.90)
        point_two = (0.30, -0.10, 0.20, -1.80, 0.30, 1.70, 1.00)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=JointTrajectory(
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
                    JointTrajectoryPoint(point_one, 0.5),
                    JointTrajectoryPoint(point_two, 1.0),
                ),
            ),
        )

        executor = _TrajectoryExecutor()
        executor.sync_with_snapshot(snapshot)
        executor._articulation.positions[:7] = np.array((0.221, -0.20, 0.10, -1.90, 0.20, 1.80, 0.90), dtype=float)

        executor.step()

        self.assertEqual(executor._active_trajectory_point_index, 1)

    def test_joint_trajectory_segments_use_time_from_start_and_synthetic_start(self) -> None:
        class _FakeArticulation:
            def __init__(self) -> None:
                self.positions = np.array([0.5, -0.1, 0.2, -1.7, 0.1, 1.6, 0.8, 0.04, 0.04], dtype=float)

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

        class _SegmentExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka")
                self._initialized = True
                self._articulation = _FakeArticulation()

            def _initialize_if_needed(self) -> bool:
                return True

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=JointTrajectory(
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
            ),
        )

        executor = _SegmentExecutor()
        executor.sync_with_snapshot(snapshot)

        self.assertEqual(len(executor._joint_trajectory_segments), 2)
        np.testing.assert_allclose(
            executor._joint_trajectory_segments[0].start_positions[:7],
            np.array([0.5, -0.1, 0.2, -1.7, 0.1, 1.6, 0.8], dtype=float),
        )
        self.assertAlmostEqual(executor._joint_trajectory_segments[0].duration_sec, 0.5, places=6)
        self.assertAlmostEqual(executor._joint_trajectory_segments[1].duration_sec, 0.5, places=6)

    def test_joint_trajectory_segments_extend_first_duration_for_synthetic_start(self) -> None:
        class _FakeArticulation:
            def __init__(self) -> None:
                self.positions = np.array([1.7, -0.1, 0.2, -1.7, 0.1, 1.6, 0.8, 0.04, 0.04], dtype=float)

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

        class _SegmentExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka")
                self._initialized = True
                self._articulation = _FakeArticulation()

            def _initialize_if_needed(self) -> bool:
                return True

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=JointTrajectory(
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
            ),
        )

        executor = _SegmentExecutor()
        executor.sync_with_snapshot(snapshot)

        self.assertEqual(len(executor._joint_trajectory_segments), 2)
        self.assertGreater(executor._joint_trajectory_segments[0].duration_sec, 0.5)

    def test_executor_falls_back_to_position_commands_when_velocity_action_is_unavailable(self) -> None:
        class _PositionOnlyArticulation:
            def __init__(self) -> None:
                self.positions = np.zeros(9, dtype=float)
                self.set_joint_positions_calls = 0

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

            def set_joint_positions(self, positions: np.ndarray) -> None:
                self.set_joint_positions_calls += 1
                self.positions = np.asarray(positions, dtype=float).copy()

        class _FallbackExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka")
                self._initialized = True
                self._articulation = _PositionOnlyArticulation()

            def _initialize_if_needed(self) -> bool:
                return True

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=JointTrajectory(
                joint_names=IsaacFrankaMotionExecutor.ARM_JOINT_NAMES,
                points=(JointTrajectoryPoint((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 1.0),),
            ),
        )

        executor = _FallbackExecutor()
        executor.sync_with_snapshot(snapshot)
        executor.step()

        self.assertGreaterEqual(executor._articulation.set_joint_positions_calls, 1)

    def test_joint_velocity_command_is_clamped_by_joint_limits(self) -> None:
        class _ActionCapableFakeArticulation:
            def __init__(self) -> None:
                self.positions = np.zeros(9, dtype=float)
                self.last_action: object | None = None

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

            def apply_action(self, action: object) -> None:
                self.last_action = action
                self.positions = np.asarray(getattr(action, "joint_positions"), dtype=float).copy()

        class _ClampedExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka")
                self._initialized = True
                self._articulation = _ActionCapableFakeArticulation()

            def _initialize_if_needed(self) -> bool:
                return True

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=JointTrajectory(
                joint_names=IsaacFrankaMotionExecutor.ARM_JOINT_NAMES,
                points=(JointTrajectoryPoint((10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.1),),
            ),
        )

        executor = _ClampedExecutor()
        executor.sync_with_snapshot(snapshot)
        executor.step()

        self.assertIsNotNone(executor._articulation.last_action)
        command_velocities = np.asarray(getattr(executor._articulation.last_action, "joint_velocities"), dtype=float)
        self.assertLessEqual(abs(float(command_velocities[0])), 2.175 + 1e-6)

    def test_executor_requests_replan_when_joint_trajectory_violates_path_tolerance(self) -> None:
        class _StickyArticulation:
            def __init__(self) -> None:
                self.positions = np.zeros(9, dtype=float)
                self.apply_action_calls = 0

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

            def apply_action(self, action: object) -> None:
                self.apply_action_calls += 1

            def set_joint_positions(self, positions: np.ndarray) -> None:
                self.positions = np.asarray(positions, dtype=float).copy()

        class _FallbackExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka")
                self._initialized = True
                self._articulation = _StickyArticulation()
                self._now = 0.0
                self._action_client._port._monotonic_time_sec = lambda: self._now

            def _initialize_if_needed(self) -> bool:
                return True

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        waypoint = Pose3D(0.35, 0.00, 0.57, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=waypoint,
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=JointTrajectory(
                joint_names=IsaacFrankaMotionExecutor.ARM_JOINT_NAMES,
                points=(
                    JointTrajectoryPoint((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0),
                    JointTrajectoryPoint((1.0, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 0.2),
                ),
            ),
        )

        executor = _FallbackExecutor()
        executor.sync_with_snapshot(snapshot)
        executor._now = 0.0
        first_log = executor.step()
        executor._now = 0.3
        second_log = executor.step()
        executor._now = 0.6
        third_log = executor.step()

        self.assertIn("joint trajectory", first_log)
        self.assertIsNone(second_log)
        self.assertEqual(
            third_log,
            "[Simulator] MoveIt2 joint trajectory aborted; waiting for replanned motion command. reason=path_tolerance_violation",
        )
        self.assertIn("path_tolerance_violation", executor.consume_replan_request() or "")
        self.assertEqual(executor._joint_trajectory_segments, ())
        self.assertEqual(executor._joint_waypoint_targets, ())

    def test_hand_pose_is_shifted_back_from_grasp_center(self) -> None:
        grasp_center_pose = Pose3D(0.42, 0.0, 0.54, 180.0, 0.0, 0.0)

        hand_pose = _hand_pose_from_grasp_center_pose(
            grasp_center_pose,
            grasp_center_offset_from_hand_m=(0.0, 0.0, 0.0584),
        )

        self.assertAlmostEqual(hand_pose.x, 0.42, places=6)
        self.assertAlmostEqual(hand_pose.y, 0.0, places=6)
        self.assertAlmostEqual(hand_pose.z, 0.5984, places=6)

    def test_pose_distance_uses_xyz_distance(self) -> None:
        distance_m = pose_distance_m(
            Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            Pose3D(0.33, 0.04, 0.57, 180.0, 0.0, 0.0),
        )

        self.assertAlmostEqual(distance_m, 0.05, places=6)

    def test_is_pose_reached_respects_tolerance(self) -> None:
        current_pose = Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0)
        target_pose = Pose3D(0.32, 0.01, 0.57, 180.0, 0.0, 0.0)

        self.assertTrue(is_pose_reached(current_pose, target_pose, position_tolerance_m=0.03))
        self.assertFalse(is_pose_reached(current_pose, target_pose, position_tolerance_m=0.01))

    def test_executor_reuses_joint_waypoint_targets_when_only_active_index_changes(self) -> None:
        class _CountingExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka")
                self.solve_calls = 0

            def _solve_joint_targets_for_waypoints(self, waypoints: tuple[Pose3D, ...]) -> tuple[np.ndarray, ...]:
                self.solve_calls += 1
                return tuple(np.full(9, float(index), dtype=float) for index, _ in enumerate(waypoints))

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        waypoint_a = Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0)
        waypoint_b = Pose3D(0.42, 0.00, 0.54, 180.0, 0.0, 0.0)
        snapshot_one = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=waypoint_b,
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(waypoint_a, waypoint_b),
            active_waypoint_index=0,
        )
        snapshot_two = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=waypoint_b,
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(waypoint_a, waypoint_b),
            active_waypoint_index=1,
        )

        executor = _CountingExecutor()
        executor.sync_with_snapshot(snapshot_one)
        executor.sync_with_snapshot(snapshot_two)

        self.assertEqual(executor.solve_calls, 1)

    def test_executor_does_not_rewind_local_waypoint_progress_when_snapshot_lags(self) -> None:
        class _StickyWaypointExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka")
                self.solve_calls = 0

            def _solve_joint_targets_for_waypoints(self, waypoints: tuple[Pose3D, ...]) -> tuple[np.ndarray, ...]:
                self.solve_calls += 1
                return tuple(np.full(9, float(index), dtype=float) for index, _ in enumerate(waypoints))

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        waypoint_a = Pose3D(0.60, 0.00, 0.60, 180.0, 0.0, 0.0)
        waypoint_b = Pose3D(0.54, 0.00, 0.62, 180.0, 0.0, 0.0)
        lagging_snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.HELD,
            gripper_closed=True,
            robot_home=False,
            cycle_id=2,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=waypoint_a,
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(waypoint_a, waypoint_b),
            active_waypoint_index=0,
        )

        executor = _StickyWaypointExecutor()
        executor.sync_with_snapshot(lagging_snapshot)
        executor._active_waypoint_index = 1
        executor.sync_with_snapshot(lagging_snapshot)

        self.assertEqual(executor.solve_calls, 1)
        self.assertEqual(executor._active_waypoint_index, 1)

    def test_waypoint_ik_uses_hand_pose_shifted_from_grasp_center(self) -> None:
        class _WaypointRecordingExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka")
                self.recorded_targets: list[Pose3D] = []

            def _solve_joint_targets_for_pose(self, target_pose: Pose3D) -> np.ndarray | None:
                self.recorded_targets.append(target_pose)
                return np.zeros(9, dtype=float)

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        grasp_center_waypoint = Pose3D(0.58, 0.0, 0.585, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=grasp_center_waypoint,
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(grasp_center_waypoint,),
            active_waypoint_index=0,
        )

        executor = _WaypointRecordingExecutor()
        executor.sync_with_snapshot(snapshot)

        self.assertEqual(len(executor.recorded_targets), 1)
        self.assertAlmostEqual(executor.recorded_targets[0].x, 0.58, places=6)
        self.assertAlmostEqual(executor.recorded_targets[0].y, 0.0, places=6)
        self.assertAlmostEqual(executor.recorded_targets[0].z, 0.6434, places=6)

    def test_executor_prefers_joint_trajectory_when_present(self) -> None:
        class _FakeArticulation:
            def __init__(self) -> None:
                self.positions = np.zeros(9, dtype=float)

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

            def set_joint_positions(self, positions: np.ndarray) -> None:
                self.positions = np.asarray(positions, dtype=float).copy()

        class _TrajectoryExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka", max_joint_step_rad=1.0)
                self._initialized = True
                self._articulation = _FakeArticulation()
                self.ik_calls = 0

            def _initialize_if_needed(self) -> bool:
                return True

            def _solve_joint_targets_for_waypoints(self, waypoints: tuple[Pose3D, ...]) -> tuple[np.ndarray, ...]:
                raise AssertionError("waypoint IK should not be used when a joint trajectory is present")

            def _apply_inverse_kinematics(self, target_pose: Pose3D) -> None:
                self.ik_calls += 1

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=JointTrajectory(
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
                    JointTrajectoryPoint((0.1, -0.3, 0.05, -2.0, 0.1, 1.75, 0.85), 0.5),
                    JointTrajectoryPoint((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 1.0),
                ),
            ),
        )

        executor = _TrajectoryExecutor()
        executor.sync_with_snapshot(snapshot)
        log = executor.step()

        self.assertIn("joint trajectory", log)
        self.assertEqual(executor.ik_calls, 0)
        self.assertNotEqual(executor._joint_trajectory_targets, ())

    def test_debug_trajectory_log_is_emitted_when_enabled(self) -> None:
        class _FakeArticulation:
            def __init__(self) -> None:
                self.positions = np.zeros(9, dtype=float)

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

            def set_joint_positions(self, positions: np.ndarray) -> None:
                self.positions = np.asarray(positions, dtype=float).copy()

        class _DebugTrajectoryExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka", max_joint_step_rad=1.0)
                self._initialized = True
                self._articulation = _FakeArticulation()
                self._trajectory_debug_enabled = True

            def _initialize_if_needed(self) -> bool:
                return True

            def _get_end_effector_pose(self) -> Pose3D | None:
                return Pose3D(0.44, 0.0, 0.53, 180.0, 0.0, 0.0)

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=JointTrajectory(
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
                    JointTrajectoryPoint((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 1.0),
                ),
            ),
        )

        executor = _DebugTrajectoryExecutor()
        output = io.StringIO()
        with redirect_stdout(output):
            executor.sync_with_snapshot(snapshot)
            log = executor.step()
            executor.log_post_update_debug_snapshot()

        self.assertIn("joint trajectory", log)
        text = output.getvalue()
        self.assertIn("[Simulator][TrajectoryDebug][set_joint_velocity]", text)
        self.assertIn("[Simulator][TrajectoryDebug][post_update]", text)
        self.assertIn("target_xyz=(0.3000, 0.0000, 0.5700)", text)

    def test_ready_snapshot_without_target_forces_home_motion_after_reset_cycle(self) -> None:
        class _FakeArticulation:
            def __init__(self) -> None:
                self.positions = np.array([0.4, -0.3, 0.2, -1.2, 0.3, 1.5, 0.7, 0.04, 0.04], dtype=float)

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

            def set_joint_positions(self, positions: np.ndarray) -> None:
                self.positions = np.asarray(positions, dtype=float).copy()

        class _HomeExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka", max_joint_step_rad=1.0)
                self._initialized = True
                self._articulation = _FakeArticulation()
                self._home_joint_positions = np.zeros(9, dtype=float)

            def _initialize_if_needed(self) -> bool:
                return True

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.READY,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=True,
            cycle_id=3,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=None,
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
        )

        executor = _HomeExecutor()
        executor.sync_with_snapshot(snapshot)
        log = executor.step()

        self.assertEqual(log, "[Simulator] Returning Franka to the home joint pose.")
        self.assertLess(abs(executor._articulation.positions[0]), 1e-6)

    def test_executor_prefers_joint_trajectory_over_waypoint_ik_when_both_exist(self) -> None:
        class _FakeArticulation:
            def __init__(self) -> None:
                self.positions = np.zeros(9, dtype=float)

            def get_joint_positions(self) -> np.ndarray:
                return self.positions.copy()

            def set_joint_positions(self, positions: np.ndarray) -> None:
                self.positions = np.asarray(positions, dtype=float).copy()

        class _WaypointFirstExecutor(IsaacFrankaMotionExecutor):
            def __init__(self) -> None:
                super().__init__(robot_prim_path="/World/Franka", max_joint_step_rad=1.0)
                self._initialized = True
                self._articulation = _FakeArticulation()
                self.solve_calls = 0

            def _initialize_if_needed(self) -> bool:
                return True

            def _solve_joint_targets_for_waypoints(self, waypoints: tuple[Pose3D, ...]) -> tuple[np.ndarray, ...]:
                self.solve_calls += 1
                return (np.array([0.3, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9, 0.04, 0.04], dtype=float),)

        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        waypoint = Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0)
        snapshot = SceneSnapshot(
            phase=ScenePhase.RUNNING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=False,
            cycle_id=1,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=waypoint,
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(waypoint,),
            active_waypoint_index=0,
            motion_joint_trajectory=JointTrajectory(
                joint_names=(
                    "panda_joint1",
                    "panda_joint2",
                    "panda_joint3",
                    "panda_joint4",
                    "panda_joint5",
                    "panda_joint6",
                    "panda_joint7",
                ),
                points=(JointTrajectoryPoint((0.1, -0.3, 0.05, -2.0, 0.1, 1.75, 0.85), 0.5),),
            ),
        )

        executor = _WaypointFirstExecutor()
        executor.sync_with_snapshot(snapshot)
        log = executor.step()

        self.assertEqual(executor.solve_calls, 0)
        self.assertIn("joint trajectory", log)
        self.assertNotEqual(executor._joint_trajectory_targets, ())


if __name__ == "__main__":
    unittest.main()
