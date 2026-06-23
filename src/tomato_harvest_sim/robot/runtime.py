from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.api.bridge import BridgeProtocol
from tomato_harvest_sim.api.contracts import (
    ControlCommand,
    HarvestTaskPhase,
    MotionCommand,
    Pose3D,
    PreGraspPlan,
    RobotRuntimeState,
    ScenePhase,
    SceneSnapshot,
    TargetEstimate,
    TomatoStatus,
)
from tomato_harvest_sim.simulator.scene_config import load_scene_layout_config
from tomato_harvest_sim.robot.motion import MoveItStyleMotionPublisher
from tomato_harvest_sim.robot.perception import TomatoTargetEstimator
from tomato_harvest_sim.robot.planner_backend import build_planner


@dataclass
class RobotState:
    runtime_state: RobotRuntimeState
    task_phase: HarvestTaskPhase
    last_seen_phase: ScenePhase
    last_scene_snapshot: SceneSnapshot | None
    last_target_estimate: TargetEstimate | None
    last_pregrasp_plan: PreGraspPlan | None
    last_motion_command: MotionCommand | None
    planner_backend_name: str
    grasp_evaluation_wait_steps: int
    grasp_converged_steps: int
    grasp_settle_wait_steps: int


class RobotRuntime:
    POSITION_TOLERANCE_M = 0.03
    GRASP_CLOSE_TOLERANCE_M = 0.005
    GRASP_CLOSE_STABLE_STEPS = 2
    GRASP_SETTLE_STEPS = 5
    GRASP_EVALUATION_TIMEOUT_STEPS = 60
    GRASP_EVALUATION_LOG_INTERVAL_STEPS = 10

    def __init__(self, *, grasp_lateral_offset_m: float = 0.0) -> None:
        self._estimator = TomatoTargetEstimator()
        self._planner, planner_info = build_planner(grasp_lateral_offset_m=grasp_lateral_offset_m)
        self._motion_publisher = MoveItStyleMotionPublisher()
        self.state = RobotState(
            runtime_state=RobotRuntimeState.BOOTING,
            task_phase=HarvestTaskPhase.IDLE,
            last_seen_phase=ScenePhase.BOOTING,
            last_scene_snapshot=None,
            last_target_estimate=None,
            last_pregrasp_plan=None,
            last_motion_command=None,
            planner_backend_name=planner_info.name,
            grasp_evaluation_wait_steps=0,
            grasp_converged_steps=0,
            grasp_settle_wait_steps=0,
        )

    def boot(self) -> None:
        self.state.runtime_state = RobotRuntimeState.READY
        self.state.task_phase = HarvestTaskPhase.IDLE
        self.state.last_seen_phase = ScenePhase.READY
        self.state.grasp_evaluation_wait_steps = 0
        self.state.grasp_converged_steps = 0
        self.state.grasp_settle_wait_steps = 0

    def observe_scene(self, snapshot: SceneSnapshot) -> None:
        self.state.last_scene_snapshot = snapshot
        self.state.last_seen_phase = snapshot.phase

    def apply_control(self, command: ControlCommand) -> None:
        if command is ControlCommand.START:
            self.state.runtime_state = RobotRuntimeState.RUNNING
            self.state.task_phase = HarvestTaskPhase.DETECTING
            self.state.grasp_evaluation_wait_steps = 0
            self.state.grasp_converged_steps = 0
            self.state.grasp_settle_wait_steps = 0
            return

        if command is ControlCommand.STOP:
            self.state.runtime_state = RobotRuntimeState.STOPPED
            self.state.task_phase = HarvestTaskPhase.STOPPED
            self.state.grasp_evaluation_wait_steps = 0
            self.state.grasp_converged_steps = 0
            self.state.grasp_settle_wait_steps = 0
            return

        if command is ControlCommand.RESET:
            self.state.runtime_state = RobotRuntimeState.READY
            self.state.task_phase = HarvestTaskPhase.IDLE
            self.state.last_target_estimate = None
            self.state.last_pregrasp_plan = None
            self.state.last_motion_command = None
            self.state.grasp_evaluation_wait_steps = 0
            self.state.grasp_converged_steps = 0
            self.state.grasp_settle_wait_steps = 0
            return

        raise ValueError(f"Unsupported control command: {command}")

    def step(self, bridge: BridgeProtocol) -> tuple[str, ...]:
        if self.state.runtime_state is not RobotRuntimeState.RUNNING:
            return ()

        if self.state.task_phase is HarvestTaskPhase.DETECTING:
            estimate = self._estimator.estimate(
                bridge.read_camera_frame("fixed_camera"),
                bridge.read_tf_tree(),
            )
            self.state.last_target_estimate = estimate
            self.state.task_phase = HarvestTaskPhase.TARGET_FOUND
            bridge.publish_target_estimate(estimate)
            return (
                "[State] detecting -> target_found",
                "Target is Found!",
                (
                    "Target camera xyz: "
                    f"({estimate.target_camera_pose.x:.4f}, {estimate.target_camera_pose.y:.4f}, {estimate.target_camera_pose.z:.4f})"
                ),
                (
                    "Target world xyz: "
                    f"({estimate.target_world_pose.x:.4f}, {estimate.target_world_pose.y:.4f}, {estimate.target_world_pose.z:.4f})"
                ),
            )

        if self.state.task_phase is HarvestTaskPhase.TARGET_FOUND:
            if self.state.last_target_estimate is None:
                return ()
            plan = self._planner.plan(
                self.state.last_target_estimate,
                bridge.read_joint_state(),
                bridge.read_tf_tree(),
                self._require_scene_snapshot(),
            )
            self.state.last_pregrasp_plan = plan
            self.state.planner_backend_name = plan.planner_name
            self.state.task_phase = HarvestTaskPhase.PLANNING
            return (
                "[State] target_found -> planning",
                f"[Planning] Planner backend={self.state.planner_backend_name}",
                "[Planning] MoveIt2-ready pre-grasp plan was created.",
                (
                    "Pre-grasp world xyz: "
                    f"({plan.pregrasp_pose.x:.4f}, {plan.pregrasp_pose.y:.4f}, {plan.pregrasp_pose.z:.4f})"
                ),
                (
                    "Grasp world xyz: "
                    f"({plan.grasp_pose.x:.4f}, {plan.grasp_pose.y:.4f}, {plan.grasp_pose.z:.4f})"
                ),
                (
                    "Pull world xyz: "
                    f"({plan.pull_pose.x:.4f}, {plan.pull_pose.y:.4f}, {plan.pull_pose.z:.4f})"
                ),
            )

        if self.state.task_phase is HarvestTaskPhase.PLANNING:
            if self.state.last_pregrasp_plan is None:
                return ()
            command = self._motion_publisher.build_pregrasp_command(self.state.last_pregrasp_plan)
            self.state.last_motion_command = command
            self.state.task_phase = HarvestTaskPhase.MOVING_TO_PREGRASP
            bridge.publish_motion_command(command)
            return (
                "[State] planning -> moving_to_pregrasp",
                "[Approach] Published pre-grasp command to simulator side.",
            )

        if self.state.task_phase is HarvestTaskPhase.MOVING_TO_PREGRASP:
            snapshot = self._require_scene_snapshot()
            target_pose = self.state.last_pregrasp_plan.pregrasp_pose
            if not self._target_pose_reached(snapshot, target_pose):
                error_m = self._pose_error_m(snapshot.robot_tool_pose, target_pose)
                return (
                    "[Approach] Waiting for pre-grasp convergence.",
                    (
                        "  tool_xyz="
                        f"({snapshot.robot_tool_pose.x:.4f}, {snapshot.robot_tool_pose.y:.4f}, {snapshot.robot_tool_pose.z:.4f}) "
                        "target_xyz="
                        f"({target_pose.x:.4f}, {target_pose.y:.4f}, {target_pose.z:.4f}) "
                        f"error={error_m:.4f} m"
                    ),
                )
            self.state.task_phase = HarvestTaskPhase.PREGRASP_REACHED
            return (
                "[State] moving_to_pregrasp -> pregrasp_reached",
                "[Complete] pre-grasp reached.",
            )

        if self.state.task_phase is HarvestTaskPhase.PREGRASP_REACHED:
            if self.state.last_pregrasp_plan is None:
                return ()
            command = self._motion_publisher.build_grasp_command(self.state.last_pregrasp_plan)
            self.state.last_motion_command = command
            self.state.task_phase = HarvestTaskPhase.MOVING_TO_GRASP
            self.state.grasp_converged_steps = 0
            bridge.publish_motion_command(command)
            return (
                "[State] pregrasp_reached -> moving_to_grasp",
                "[Approach] Published grasp command to simulator side.",
                (
                    "Grasp command target xyz: "
                    f"({command.target_pose.x:.4f}, {command.target_pose.y:.4f}, {command.target_pose.z:.4f})"
                ),
            )

        if self.state.task_phase is HarvestTaskPhase.MOVING_TO_GRASP:
            snapshot = self._require_scene_snapshot()
            target_pose = self.state.last_pregrasp_plan.grasp_pose
            error_m = self._pose_error_m(snapshot.robot_tool_pose, target_pose)
            if error_m > self.GRASP_CLOSE_TOLERANCE_M:
                self.state.grasp_converged_steps = 0
                return (
                    "[Approach] Waiting for grasp convergence.",
                    (
                        "  tool_xyz="
                        f"({snapshot.robot_tool_pose.x:.4f}, {snapshot.robot_tool_pose.y:.4f}, {snapshot.robot_tool_pose.z:.4f}) "
                        "target_xyz="
                        f"({target_pose.x:.4f}, {target_pose.y:.4f}, {target_pose.z:.4f}) "
                        f"error={error_m:.4f} m"
                    ),
                )
            self.state.grasp_converged_steps += 1
            if self.state.grasp_converged_steps < self.GRASP_CLOSE_STABLE_STEPS:
                return (
                    "[Approach] Waiting for grasp convergence.",
                    (
                        "  tool_xyz="
                        f"({snapshot.robot_tool_pose.x:.4f}, {snapshot.robot_tool_pose.y:.4f}, {snapshot.robot_tool_pose.z:.4f}) "
                        "target_xyz="
                        f"({target_pose.x:.4f}, {target_pose.y:.4f}, {target_pose.z:.4f}) "
                        f"error={error_m:.4f} m stable_steps={self.state.grasp_converged_steps}/{self.GRASP_CLOSE_STABLE_STEPS}"
                    ),
                )
            self.state.task_phase = HarvestTaskPhase.AT_GRASP
            self.state.grasp_converged_steps = 0
            self.state.grasp_settle_wait_steps = 0
            return (
                "[State] moving_to_grasp -> at_grasp",
                "[Complete] grasp pose reached.",
            )

        if self.state.task_phase is HarvestTaskPhase.AT_GRASP:
            if self.state.grasp_settle_wait_steps < self.GRASP_SETTLE_STEPS:
                self.state.grasp_settle_wait_steps += 1
                if self.state.grasp_settle_wait_steps == 1:
                    return (
                        "[Grasp] Settling at grasp pose before closing the gripper.",
                    )
                return ()
            if self.state.last_pregrasp_plan is None:
                return ()
            command = self._motion_publisher.build_close_gripper_command(self.state.last_pregrasp_plan)
            self.state.last_motion_command = command
            self.state.task_phase = HarvestTaskPhase.GRASP_EVALUATION
            self.state.grasp_evaluation_wait_steps = 0
            self.state.grasp_converged_steps = 0
            self.state.grasp_settle_wait_steps = 0
            bridge.publish_motion_command(command)
            return (
                "[State] at_grasp -> grasp_evaluation",
                "[Grasp] Closing the gripper for grasp evaluation.",
            )

        if self.state.task_phase is HarvestTaskPhase.GRASP_EVALUATION:
            snapshot = self.state.last_scene_snapshot
            if snapshot is None:
                return ()
            if snapshot.tomato_status is TomatoStatus.HELD:
                self.state.grasp_evaluation_wait_steps = 0
                if self.state.last_pregrasp_plan is None:
                    return ()
                command = self._motion_publisher.build_pull_command(self.state.last_pregrasp_plan)
                self.state.last_motion_command = command
                self.state.task_phase = HarvestTaskPhase.DETACHING
                bridge.publish_motion_command(command)
                return (
                    "[State] grasp_evaluation -> detaching",
                    "[Grasp] Stable grasp established.",
                    "[Detach] Published pull command to detach the tomato.",
                )
            if snapshot.tomato_status is TomatoStatus.FALLEN:
                self.state.grasp_evaluation_wait_steps = 0
                self.state.task_phase = HarvestTaskPhase.FAILED
                reason = snapshot.grasp_result_reason or "grasp_failed"
                return (
                    "[State] grasp_evaluation -> failed",
                    f"[Failed] The tomato was not stably grasped by both fingers. reason={reason}",
                )
            self.state.grasp_evaluation_wait_steps += 1
            if self.state.grasp_evaluation_wait_steps >= self.GRASP_EVALUATION_TIMEOUT_STEPS:
                self.state.grasp_evaluation_wait_steps = 0
                self.state.task_phase = HarvestTaskPhase.FAILED
                reason = snapshot.grasp_result_reason or "grasp_evaluation_timeout"
                return (
                    "[State] grasp_evaluation -> failed",
                    "[Failed] Grasp evaluation timed out waiting for the physics grasp result. "
                    f"reason={reason}",
                )
            if self.state.grasp_evaluation_wait_steps % self.GRASP_EVALUATION_LOG_INTERVAL_STEPS == 0:
                return (
                    "[Grasp] Waiting for the physics grasp result.",
                    (
                        f"  tomato_status={snapshot.tomato_status.value} "
                        f"gripper_closed={snapshot.gripper_closed} "
                        f"wait_steps={self.state.grasp_evaluation_wait_steps}"
                    ),
                )
            return ()

        if self.state.task_phase is HarvestTaskPhase.DETACHING:
            snapshot = self.state.last_scene_snapshot
            if snapshot is None:
                return ()
            if snapshot.tomato_status is TomatoStatus.DETACHED:
                self.state.task_phase = HarvestTaskPhase.DETACHED
                return (
                    "[State] detaching -> detached",
                    "[Detach] Tomato detached from stem.",
                )
            if snapshot.tomato_status is TomatoStatus.FALLEN:
                self.state.task_phase = HarvestTaskPhase.FAILED
                reason = snapshot.grasp_result_reason or "detach_failed"
                return (
                    "[State] detaching -> failed",
                    f"[Failed] Detach failed and the tomato fell. reason={reason}",
                )
            return ()

        if self.state.task_phase is HarvestTaskPhase.DETACHED:
            if self.state.last_pregrasp_plan is None:
                return ()
            command = self._motion_publisher.build_place_command(self.state.last_pregrasp_plan)
            self.state.last_motion_command = command
            self.state.task_phase = HarvestTaskPhase.MOVING_TO_PLACE
            bridge.publish_motion_command(command)
            return (
                "[State] detached -> moving_to_place",
                "[Place] Published place command to simulator side.",
            )

        if self.state.task_phase is HarvestTaskPhase.MOVING_TO_PLACE:
            if not self._target_pose_reached(self._require_scene_snapshot(), self.state.last_pregrasp_plan.place_pose):
                return ()
            if self.state.last_pregrasp_plan is None:
                return ()
            command = self._motion_publisher.build_open_gripper_command(self.state.last_pregrasp_plan)
            self.state.last_motion_command = command
            self.state.task_phase = HarvestTaskPhase.PLACED
            bridge.publish_motion_command(command)
            return (
                "[State] moving_to_place -> placed",
                "[Place] Opening the gripper above the tray.",
            )

        if self.state.task_phase is HarvestTaskPhase.PLACED:
            snapshot = self.state.last_scene_snapshot
            if snapshot is None:
                return ()
            if snapshot.tomato_status is TomatoStatus.PLACED:
                if self.state.last_pregrasp_plan is None:
                    return ()
                command = self._motion_publisher.build_home_command(self.state.last_pregrasp_plan)
                self.state.last_motion_command = command
                self.state.task_phase = HarvestTaskPhase.RETURNING_HOME
                bridge.publish_motion_command(command)
                return (
                    "[State] placed -> returning_home",
                    "[Complete] Tomato placed in the tray.",
                    "[Home] Returning the robot to the home pose.",
                )
            if snapshot.tomato_status is TomatoStatus.FALLEN:
                self.state.task_phase = HarvestTaskPhase.FAILED
                reason = snapshot.grasp_result_reason or "place_failed"
                return (
                    "[State] placed -> failed",
                    f"[Failed] Tomato placement failed. reason={reason}",
                )
            return ()

        if self.state.task_phase is HarvestTaskPhase.RETURNING_HOME:
            snapshot = self._require_scene_snapshot()
            if snapshot.robot_home or self._target_pose_reached(snapshot, self._home_tool_pose()):
                self.state.task_phase = HarvestTaskPhase.COMPLETE
                return (
                    "[State] returning_home -> complete",
                    "[Complete] Harvest scenario completed and the robot returned home.",
                )
            return ()

        return ()

    def _require_scene_snapshot(self) -> SceneSnapshot:
        if self.state.last_scene_snapshot is None:
            raise RuntimeError("Scene snapshot is not available.")
        return self.state.last_scene_snapshot

    def _target_pose_reached(self, snapshot: SceneSnapshot, target_pose: Pose3D) -> bool:
        return self._pose_error_m(snapshot.robot_tool_pose, target_pose) <= self.POSITION_TOLERANCE_M

    @staticmethod
    def _pose_error_m(current_pose: Pose3D, target_pose: Pose3D) -> float:
        dx = current_pose.x - target_pose.x
        dy = current_pose.y - target_pose.y
        dz = current_pose.z - target_pose.z
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    @staticmethod
    def _home_tool_pose() -> Pose3D:
        return load_scene_layout_config().home_tool_pose
