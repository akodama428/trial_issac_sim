from __future__ import annotations

import unittest

import numpy as np

from tomato_harvest_sim.api.contracts import (
    AbortPolicy,
    ExecutionPhaseSpec,
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
    PhaseExecutionIntent,
    PhaseId,
    PhaseMotionPlan,
    Pose3D,
    PoseSemantics,
    ScenePhase,
    SceneSnapshot,
    SuccessJudge,
    SuccessPolicy,
    TomatoStatus,
)
from tomato_harvest_sim.api.hardware_control import HardwareControlPort, HardwareStateSample
from tomato_harvest_sim.api.trajectory_execution import (
    TrajectoryExecutionFeedback,
    TrajectoryExecutionPort,
    TrajectoryExecutionRequest,
    TrajectoryExecutionResult,
    TrajectoryExecutionState,
)


class TrajectoryTrackingRefactorTest(unittest.TestCase):
    def _build_snapshot(
        self,
        *,
        cycle_id: int = 1,
        phase: ScenePhase = ScenePhase.RUNNING,
        target_tool_pose: Pose3D | None = None,
        motion_waypoints: tuple[Pose3D, ...] = (),
        motion_joint_trajectory: JointTrajectory | None = None,
        gripper_closed: bool = False,
        active_waypoint_index: int | None = None,
        execution_phase_spec: ExecutionPhaseSpec | None = None,
    ) -> SceneSnapshot:
        pose = Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
        return SceneSnapshot(
            phase=phase,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=gripper_closed,
            robot_home=False,
            cycle_id=cycle_id,
            robot_model="Franka Panda",
            robot_base_pose=pose,
            fixed_camera_pose=pose,
            hand_camera_pose=pose,
            branch_pose=pose,
            stem_pose=pose,
            tomato_pose=pose,
            tray_pose=pose,
            robot_tool_pose=pose,
            target_tool_pose=target_tool_pose,
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=motion_waypoints,
            active_waypoint_index=active_waypoint_index,
            motion_joint_trajectory=motion_joint_trajectory,
            execution_phase_spec=execution_phase_spec,
        )

    def test_state_store_marks_home_pending_on_ready_cycle_change(self) -> None:
        from tomato_harvest_sim.robot.trajectory_tracking.state_store import TrajectoryTrackingStateStore

        store = TrajectoryTrackingStateStore()

        running_snapshot = self._build_snapshot(
            cycle_id=1,
            target_tool_pose=Pose3D(0.3, 0.0, 0.57, 180.0, 0.0, 0.0),
        )
        ready_snapshot = self._build_snapshot(
            cycle_id=2,
            phase=ScenePhase.READY,
            target_tool_pose=None,
        )

        store.normalize_snapshot(running_snapshot)
        store.normalize_snapshot(ready_snapshot)

        self.assertTrue(store.state.home_command_pending)
        self.assertIsNone(store.state.target_pose)

    def test_coordinator_submits_joint_trajectory_to_execution_port(self) -> None:
        from tomato_harvest_sim.robot.trajectory_tracking.coordinator import TrajectoryTrackingCoordinator

        class _Driver:
            def __init__(self) -> None:
                self.positions = np.zeros(9, dtype=float)

            def initialize_if_needed(self) -> bool:
                return True

            def get_observation(self):
                from tomato_harvest_sim.robot.api.trajectory_tracking import ObservationData

                return ObservationData(
                    joint_positions=self.positions.copy(),
                    joint_velocities=np.zeros(9, dtype=float),
                    end_effector_pose=Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0),
                    joint_state_snapshot=self.current_joint_state_snapshot(),
                )

            def current_joint_positions(self) -> np.ndarray | None:
                return self.positions.copy()

            def current_joint_velocities(self) -> np.ndarray | None:
                return np.zeros(9, dtype=float)

            def current_end_effector_pose(self) -> Pose3D | None:
                return Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)

            def current_joint_state_snapshot(self):
                return JointStateSnapshot(
                    joint_names=TrajectoryTrackingCoordinator.ARM_JOINT_NAMES,
                    positions_rad=tuple(float(value) for value in self.positions[:7]),
                )

            def home_joint_positions(self) -> np.ndarray | None:
                return np.zeros(9, dtype=float)

            def expand_joint_targets(self, joint_positions: np.ndarray) -> np.ndarray:
                expanded = self.positions.copy()
                expanded[: joint_positions.shape[0]] = joint_positions
                return expanded

            def solve_joint_targets_for_pose(self, target_pose: Pose3D, *, position_tolerance_m: float) -> np.ndarray | None:
                del target_pose, position_tolerance_m
                return np.zeros(9, dtype=float)

            def set_joint_positions_with_debug(self, positions: np.ndarray, *, context: str) -> None:
                del context
                self.positions = np.asarray(positions, dtype=float).copy()

            def set_joint_velocity_targets_with_debug(
                self,
                *,
                positions: np.ndarray,
                velocities: np.ndarray,
                context: str,
            ) -> None:
                del velocities, context
                self.positions = np.asarray(positions, dtype=float).copy()

        class _Hardware(HardwareControlPort):
            def __init__(self, driver: _Driver) -> None:
                self.driver = driver

            def initialize_if_needed(self) -> bool:
                return True

            def read_state(self) -> HardwareStateSample | None:
                return HardwareStateSample(
                    joint_names=TrajectoryTrackingCoordinator.ARM_JOINT_NAMES + ("finger_left", "finger_right"),
                    positions_rad=tuple(float(value) for value in self.driver.positions),
                    velocities_rad_s=tuple(0.0 for _ in range(9)),
                    timestamp_sec=0.0,
                    end_effector_pose=Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0),
                    joint_state_snapshot=self.driver.current_joint_state_snapshot(),
                )

            def write_command(self, command) -> None:
                if command.positions_rad is not None:
                    self.driver.positions = np.asarray(command.positions_rad, dtype=float).copy()

        class _ExecutionPort(TrajectoryExecutionPort):
            def __init__(self) -> None:
                self.request: TrajectoryExecutionRequest | None = None
                self.feedback: TrajectoryExecutionFeedback | None = None
                self.result: TrajectoryExecutionResult | None = None
                self.step_count = 0

            def send_goal(self, request: TrajectoryExecutionRequest) -> bool:
                self.request = request
                self.feedback = TrajectoryExecutionFeedback(
                    controller_name=request.controller_name,
                    state=TrajectoryExecutionState.ACCEPTED,
                    desired_positions_rad=tuple(0.0 for _ in range(9)),
                    actual_positions_rad=tuple(0.0 for _ in range(9)),
                    desired_velocities_rad_s=tuple(0.0 for _ in range(9)),
                    actual_velocities_rad_s=tuple(0.0 for _ in range(9)),
                    error_norm_rad=0.0,
                    timestamp_sec=0.0,
                )
                self.result = None
                return True

            def cancel_goal(self) -> None:
                self.request = None

            def step(self) -> None:
                self.step_count += 1
                if self.request is None:
                    return
                self.feedback = TrajectoryExecutionFeedback(
                    controller_name=self.request.controller_name,
                    state=TrajectoryExecutionState.ACTIVE,
                    desired_positions_rad=tuple(0.0 for _ in range(9)),
                    actual_positions_rad=tuple(0.0 for _ in range(9)),
                    desired_velocities_rad_s=tuple(0.0 for _ in range(9)),
                    actual_velocities_rad_s=tuple(0.0 for _ in range(9)),
                    error_norm_rad=0.0,
                    timestamp_sec=float(self.step_count),
                )

            def active_request(self) -> TrajectoryExecutionRequest | None:
                return self.request

            def current_feedback(self) -> TrajectoryExecutionFeedback | None:
                return self.feedback

            def current_result(self) -> TrajectoryExecutionResult | None:
                return self.result

        trajectory = JointTrajectory(
            joint_names=TrajectoryTrackingCoordinator.ARM_JOINT_NAMES,
            points=(JointTrajectoryPoint((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 1.0),),
        )
        execution_phase_spec = ExecutionPhaseSpec(
            phase_id=PhaseId.MOVING_TO_PREGRASP,
            intent=PhaseExecutionIntent(
                phase_id=PhaseId.MOVING_TO_PREGRASP,
                phase_goal_pose=Pose3D(0.3, 0.0, 0.57, 180.0, 0.0, 0.0),
                pose_semantics=PoseSemantics.TOOL_CENTER,
                success=SuccessPolicy(judge=SuccessJudge.END_EFFECTOR_POSE, position_tolerance_m=0.03),
                abort=AbortPolicy(nominal_timeout_sec=3.0, stall_timeout_sec=0.5),
            ),
            motion=PhaseMotionPlan(
                phase_goal_pose=Pose3D(0.3, 0.0, 0.57, 180.0, 0.0, 0.0),
                active_waypoints=(Pose3D(0.3, 0.0, 0.57, 180.0, 0.0, 0.0),),
                joint_trajectory=trajectory,
            ),
        )
        snapshot = self._build_snapshot(
            target_tool_pose=Pose3D(0.3, 0.0, 0.57, 180.0, 0.0, 0.0),
            motion_joint_trajectory=trajectory,
            execution_phase_spec=execution_phase_spec,
        )
        driver = _Driver()
        execution_port = _ExecutionPort()
        coordinator = TrajectoryTrackingCoordinator(
            driver=driver,
            hardware_control_port=_Hardware(driver),
            trajectory_execution_port=execution_port,
        )

        log = coordinator.run_cycle(snapshot)

        self.assertEqual(execution_port.request.trajectory, trajectory)
        self.assertEqual(execution_port.request.target_pose, execution_phase_spec.motion.phase_goal_pose)
        self.assertEqual(execution_port.request.position_tolerance_m, execution_phase_spec.intent.success.position_tolerance_m)
        self.assertEqual(execution_port.request.execution_phase_spec, execution_phase_spec)
        self.assertEqual(execution_port.step_count, 1)
        self.assertIn("accepted joint trajectory", log or "")

    def test_coordinator_does_not_fall_back_to_waypoint_step_while_controller_trajectory_is_active(self) -> None:
        from tomato_harvest_sim.robot.trajectory_tracking.coordinator import TrajectoryTrackingCoordinator

        class _Driver:
            def __init__(self) -> None:
                self.positions = np.zeros(9, dtype=float)
                self.position_commands: list[str] = []

            def initialize_if_needed(self) -> bool:
                return True

            def get_observation(self):
                from tomato_harvest_sim.robot.api.trajectory_tracking import ObservationData

                return ObservationData(
                    joint_positions=self.positions.copy(),
                    joint_velocities=np.zeros(9, dtype=float),
                    end_effector_pose=Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0),
                    joint_state_snapshot=self.current_joint_state_snapshot(),
                )

            def current_joint_positions(self) -> np.ndarray | None:
                return self.positions.copy()

            def current_joint_velocities(self) -> np.ndarray | None:
                return np.zeros(9, dtype=float)

            def current_end_effector_pose(self) -> Pose3D | None:
                return Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0)

            def current_joint_state_snapshot(self):
                return JointStateSnapshot(
                    joint_names=TrajectoryTrackingCoordinator.ARM_JOINT_NAMES,
                    positions_rad=tuple(float(value) for value in self.positions[:7]),
                )

            def home_joint_positions(self) -> np.ndarray | None:
                return np.zeros(9, dtype=float)

            def expand_joint_targets(self, joint_positions: np.ndarray) -> np.ndarray:
                expanded = self.positions.copy()
                expanded[: joint_positions.shape[0]] = joint_positions
                return expanded

            def solve_joint_targets_for_pose(self, target_pose: Pose3D, *, position_tolerance_m: float) -> np.ndarray | None:
                del target_pose, position_tolerance_m
                return np.ones(9, dtype=float)

            def set_joint_positions_with_debug(self, positions: np.ndarray, *, context: str) -> None:
                del positions
                self.position_commands.append(context)

            def set_joint_velocity_targets_with_debug(
                self,
                *,
                positions: np.ndarray,
                velocities: np.ndarray,
                context: str,
            ) -> None:
                del positions, velocities, context

        class _Hardware(HardwareControlPort):
            def __init__(self, driver: _Driver) -> None:
                self.driver = driver

            def initialize_if_needed(self) -> bool:
                return True

            def read_state(self) -> HardwareStateSample | None:
                return HardwareStateSample(
                    joint_names=TrajectoryTrackingCoordinator.ARM_JOINT_NAMES + ("finger_left", "finger_right"),
                    positions_rad=tuple(float(value) for value in self.driver.positions),
                    velocities_rad_s=tuple(0.0 for _ in range(9)),
                    timestamp_sec=0.0,
                    end_effector_pose=Pose3D(0.0, 0.0, 0.0, 180.0, 0.0, 0.0),
                    joint_state_snapshot=self.driver.current_joint_state_snapshot(),
                )

            def write_command(self, command) -> None:
                if command.positions_rad is not None:
                    self.driver.positions = np.asarray(command.positions_rad, dtype=float).copy()

        class _ExecutionPort(TrajectoryExecutionPort):
            def __init__(self) -> None:
                self.request: TrajectoryExecutionRequest | None = None
                self.feedback: TrajectoryExecutionFeedback | None = None

            def send_goal(self, request: TrajectoryExecutionRequest) -> bool:
                self.request = request
                self.feedback = TrajectoryExecutionFeedback(
                    controller_name=request.controller_name,
                    state=TrajectoryExecutionState.ACTIVE,
                    desired_positions_rad=tuple(0.0 for _ in range(9)),
                    actual_positions_rad=tuple(0.0 for _ in range(9)),
                    desired_velocities_rad_s=tuple(0.0 for _ in range(9)),
                    actual_velocities_rad_s=tuple(0.0 for _ in range(9)),
                    error_norm_rad=0.0,
                    timestamp_sec=0.0,
                )
                return True

            def cancel_goal(self) -> None:
                self.request = None

            def step(self) -> None:
                return None

            def active_request(self) -> TrajectoryExecutionRequest | None:
                return self.request

            def current_feedback(self) -> TrajectoryExecutionFeedback | None:
                return self.feedback

            def current_result(self) -> TrajectoryExecutionResult | None:
                return None

        trajectory = JointTrajectory(
            joint_names=TrajectoryTrackingCoordinator.ARM_JOINT_NAMES,
            points=(JointTrajectoryPoint((0.2, -0.2, 0.1, -1.9, 0.2, 1.8, 0.9), 1.0),),
        )
        snapshot = self._build_snapshot(
            target_tool_pose=Pose3D(0.3, 0.0, 0.57, 180.0, 0.0, 0.0),
            motion_waypoints=(Pose3D(0.4, 0.0, 0.6, 180.0, 0.0, 0.0),),
            motion_joint_trajectory=trajectory,
        )
        driver = _Driver()
        execution_port = _ExecutionPort()
        coordinator = TrajectoryTrackingCoordinator(
            driver=driver,
            hardware_control_port=_Hardware(driver),
            trajectory_execution_port=execution_port,
        )

        coordinator.run_cycle(snapshot)

        self.assertEqual(driver.position_commands, [])


if __name__ == "__main__":
    unittest.main()
