from __future__ import annotations

from dataclasses import dataclass, replace

from tomato_harvest_sim.api.bridge import BridgeProtocol
from tomato_harvest_sim.api.contracts import (
    ControlCommand,
    ExecutionPhaseSpec,
    HarvestMotionPlan,
    HarvestTaskPhase,
    MotionCommand,
    PhaseId,
    PhaseMotionPlan,
    Pose3D,
    RobotRuntimeState,
    ScenePhase,
    SceneSnapshot,
    SuccessJudge,
    TargetEstimate,
    TomatoStatus,
)
from tomato_harvest_sim.robot.api.perception import TargetEstimator
from tomato_harvest_sim.robot.api.planner import MotionPlanner
from tomato_harvest_sim.robot.behavior_planner import BehaviorPlanner
from tomato_harvest_sim.robot.motion import MoveItStyleMotionPublisher, phase_motion_from_harvest_plan
from tomato_harvest_sim.robot.perception import TomatoTargetEstimator
from tomato_harvest_sim.robot.planner import build_planner
from tomato_harvest_sim.simulator.scene_config import load_scene_layout_config


@dataclass
class RobotState:
    runtime_state: RobotRuntimeState
    task_phase: HarvestTaskPhase
    last_seen_phase: ScenePhase
    last_scene_snapshot: SceneSnapshot | None
    last_target_estimate: TargetEstimate | None
    last_harvest_motion_plan: HarvestMotionPlan | None
    last_motion_command: MotionCommand | None
    last_execution_phase_spec: ExecutionPhaseSpec | None
    planner_backend_name: str
    grasp_evaluation_wait_steps: int
    grasp_settle_wait_steps: int
    phase_success_stable_steps: int


class HarvestRuntime:
    GRASP_SETTLE_STEPS = 5
    GRASP_EVALUATION_TIMEOUT_STEPS = 60
    GRASP_EVALUATION_LOG_INTERVAL_STEPS = 10

    def __init__(self, *, grasp_lateral_offset_m: float = 0.0) -> None:
        self._estimator: TargetEstimator = TomatoTargetEstimator()
        self._planner: MotionPlanner
        self._planner, planner_info = build_planner(grasp_lateral_offset_m=grasp_lateral_offset_m)
        self._behavior_planner = BehaviorPlanner()
        self._motion_publisher = MoveItStyleMotionPublisher()
        self.state = RobotState(
            runtime_state=RobotRuntimeState.BOOTING,
            task_phase=HarvestTaskPhase.IDLE,
            last_seen_phase=ScenePhase.BOOTING,
            last_scene_snapshot=None,
            last_target_estimate=None,
            last_harvest_motion_plan=None,
            last_motion_command=None,
            last_execution_phase_spec=None,
            planner_backend_name=planner_info.name,
            grasp_evaluation_wait_steps=0,
            grasp_settle_wait_steps=0,
            phase_success_stable_steps=0,
        )

    def boot(self) -> None:
        self.state.runtime_state = RobotRuntimeState.READY
        self.state.task_phase = HarvestTaskPhase.IDLE
        self.state.last_seen_phase = ScenePhase.READY
        self._reset_phase_counters()

    def observe_scene(self, snapshot: SceneSnapshot) -> None:
        self.state.last_scene_snapshot = snapshot
        self.state.last_seen_phase = snapshot.phase

    def apply_control(self, command: ControlCommand) -> None:
        if command is ControlCommand.START:
            self.state.runtime_state = RobotRuntimeState.RUNNING
            self.state.task_phase = HarvestTaskPhase.DETECTING
            self._reset_phase_counters()
            return

        if command is ControlCommand.STOP:
            self.state.runtime_state = RobotRuntimeState.STOPPED
            self.state.task_phase = HarvestTaskPhase.STOPPED
            self._reset_phase_counters()
            return

        if command is ControlCommand.RESET:
            self.state.runtime_state = RobotRuntimeState.READY
            self.state.task_phase = HarvestTaskPhase.IDLE
            self.state.last_target_estimate = None
            self.state.last_harvest_motion_plan = None
            self.state.last_motion_command = None
            self.state.last_execution_phase_spec = None
            self._reset_phase_counters()
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
            self.state.last_harvest_motion_plan = plan
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
            command = self._publish_phase_command(
                bridge,
                next_task_phase=HarvestTaskPhase.MOVING_TO_PREGRASP,
                plan=self.state.last_harvest_motion_plan,
            )
            if command is None:
                return ()
            return (
                "[State] planning -> moving_to_pregrasp",
                "[Approach] Published pre-grasp command to simulator side.",
            )

        if self.state.task_phase is HarvestTaskPhase.MOVING_TO_PREGRASP:
            completed, waiting_logs = self._evaluate_motion_phase(self._require_scene_snapshot())
            if not completed:
                return waiting_logs
            self.state.task_phase = HarvestTaskPhase.PREGRASP_REACHED
            return (
                "[State] moving_to_pregrasp -> pregrasp_reached",
                "[Complete] pre-grasp reached.",
            )

        if self.state.task_phase is HarvestTaskPhase.PREGRASP_REACHED:
            command = self._publish_phase_command(
                bridge,
                next_task_phase=HarvestTaskPhase.MOVING_TO_GRASP,
                plan=self.state.last_harvest_motion_plan,
            )
            if command is None:
                return ()
            return (
                "[State] pregrasp_reached -> moving_to_grasp",
                "[Approach] Published grasp command to simulator side.",
                (
                    "Grasp command target xyz: "
                    f"({command.target_pose.x:.4f}, {command.target_pose.y:.4f}, {command.target_pose.z:.4f})"
                ),
            )

        if self.state.task_phase is HarvestTaskPhase.MOVING_TO_GRASP:
            completed, waiting_logs = self._evaluate_motion_phase(self._require_scene_snapshot())
            if not completed:
                return waiting_logs
            self.state.task_phase = HarvestTaskPhase.AT_GRASP
            self.state.grasp_settle_wait_steps = 0
            self.state.phase_success_stable_steps = 0
            return (
                "[State] moving_to_grasp -> at_grasp",
                "[Complete] grasp pose reached.",
            )

        if self.state.task_phase is HarvestTaskPhase.AT_GRASP:
            if self.state.grasp_settle_wait_steps < self.GRASP_SETTLE_STEPS:
                self.state.grasp_settle_wait_steps += 1
                if self.state.grasp_settle_wait_steps == 1:
                    return ("[Grasp] Settling at grasp pose before closing the gripper.",)
                return ()
            if self.state.last_harvest_motion_plan is None:
                return ()
            command = self._motion_publisher.build_close_gripper_command(self.state.last_harvest_motion_plan)
            self.state.last_motion_command = command
            self.state.last_execution_phase_spec = None
            self.state.task_phase = HarvestTaskPhase.GRASP_EVALUATION
            self.state.grasp_evaluation_wait_steps = 0
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
                command = self._publish_phase_command(
                    bridge,
                    next_task_phase=HarvestTaskPhase.DETACHING,
                    plan=self.state.last_harvest_motion_plan,
                )
                if command is None:
                    return ()
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
            if snapshot.tomato_status is TomatoStatus.FALLEN:
                self.state.task_phase = HarvestTaskPhase.FAILED
                reason = snapshot.grasp_result_reason or "detach_failed"
                return (
                    "[State] detaching -> failed",
                    f"[Failed] Detach failed and the tomato fell. reason={reason}",
                )
            completed, waiting_logs = self._evaluate_motion_phase(snapshot)
            if not completed:
                return waiting_logs
            self.state.task_phase = HarvestTaskPhase.DETACHED
            return (
                "[State] detaching -> detached",
                "[Detach] Tomato detached from stem.",
            )

        if self.state.task_phase is HarvestTaskPhase.DETACHED:
            command = self._publish_phase_command(
                bridge,
                next_task_phase=HarvestTaskPhase.MOVING_TO_PLACE,
                plan=self.state.last_harvest_motion_plan,
            )
            if command is None:
                return ()
            return (
                "[State] detached -> moving_to_place",
                "[Place] Published place command to simulator side.",
            )

        if self.state.task_phase is HarvestTaskPhase.MOVING_TO_PLACE:
            completed, waiting_logs = self._evaluate_motion_phase(self._require_scene_snapshot())
            if not completed:
                return waiting_logs
            if self.state.last_harvest_motion_plan is None:
                return ()
            command = self._motion_publisher.build_open_gripper_command(self.state.last_harvest_motion_plan)
            self.state.last_motion_command = command
            self.state.last_execution_phase_spec = None
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
                command = self._publish_phase_command(
                    bridge,
                    next_task_phase=HarvestTaskPhase.RETURNING_HOME,
                    plan=self.state.last_harvest_motion_plan,
                )
                if command is None:
                    return ()
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
            if snapshot.robot_home:
                self.state.task_phase = HarvestTaskPhase.COMPLETE
                return (
                    "[State] returning_home -> complete",
                    "[Complete] Harvest scenario completed and the robot returned home.",
                )
            completed, waiting_logs = self._evaluate_motion_phase(snapshot)
            if not completed:
                return waiting_logs
            self.state.task_phase = HarvestTaskPhase.COMPLETE
            return (
                "[State] returning_home -> complete",
                "[Complete] Harvest scenario completed and the robot returned home.",
            )

        return ()

    def replan_active_motion(self, bridge: BridgeProtocol, *, reason: str) -> tuple[str, ...]:
        if self.state.runtime_state is not RobotRuntimeState.RUNNING:
            return ()

        phase = self.state.task_phase
        if phase not in {
            HarvestTaskPhase.MOVING_TO_PREGRASP,
            HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.DETACHING,
            HarvestTaskPhase.MOVING_TO_PLACE,
        }:
            return ()

        if self.state.last_target_estimate is None:
            return ()

        plan = self._planner.plan(
            self.state.last_target_estimate,
            bridge.read_joint_state(),
            bridge.read_tf_tree(),
            self._require_scene_snapshot(),
        )
        command = self._build_motion_command_for_task_phase(plan=plan, task_phase=phase)
        if command is None:
            return ()

        self.state.last_harvest_motion_plan = plan
        self.state.last_motion_command = command
        self.state.last_execution_phase_spec = command.execution_phase_spec
        self.state.phase_success_stable_steps = 0
        self.state.planner_backend_name = plan.planner_name
        bridge.publish_motion_command(command)
        return (
            f"[Replan] Active motion replan requested. phase={phase.value} reason={reason}",
            (
                f"[Replan] Published {command.command_name} using planner={plan.planner_name} "
                f"trajectory={'yes' if command.joint_trajectory is not None else 'no'}"
            ),
        )

    def _publish_phase_command(
        self,
        bridge: BridgeProtocol,
        *,
        next_task_phase: HarvestTaskPhase,
        plan: HarvestMotionPlan | None,
    ) -> MotionCommand | None:
        command = self._build_motion_command_for_task_phase(plan=plan, task_phase=next_task_phase)
        if command is None:
            return None
        self.state.last_motion_command = command
        self.state.last_execution_phase_spec = command.execution_phase_spec
        self.state.phase_success_stable_steps = 0
        self.state.task_phase = next_task_phase
        bridge.publish_motion_command(command)
        return command

    def _build_motion_command_for_task_phase(
        self,
        *,
        plan: HarvestMotionPlan | None,
        task_phase: HarvestTaskPhase,
    ) -> MotionCommand | None:
        spec = self._build_execution_phase_spec(plan=plan, task_phase=task_phase)
        if spec is None:
            return None
        planner_name = self.state.planner_backend_name if plan is None else plan.planner_name
        return self._motion_publisher.build_phase_command(planner_name=planner_name, spec=spec)

    def _build_execution_phase_spec(
        self,
        *,
        plan: HarvestMotionPlan | None,
        task_phase: HarvestTaskPhase,
    ) -> ExecutionPhaseSpec | None:
        intent = self._behavior_planner.intent_for_task_phase(task_phase)
        if intent is None:
            return None
        if intent.phase_id is PhaseId.RETURNING_HOME:
            home_pose = self._home_tool_pose()
            motion = PhaseMotionPlan(
                phase_goal_pose=home_pose,
                active_waypoints=(home_pose,),
                joint_trajectory=None,
            )
        else:
            if plan is None:
                return None
            motion = phase_motion_from_harvest_plan(plan, intent.phase_id)
        resolved_intent = replace(intent, phase_goal_pose=motion.phase_goal_pose)
        return ExecutionPhaseSpec(
            phase_id=resolved_intent.phase_id,
            intent=resolved_intent,
            motion=motion,
        )

    def _evaluate_motion_phase(self, snapshot: SceneSnapshot) -> tuple[bool, tuple[str, ...]]:
        spec = self.state.last_execution_phase_spec
        if spec is None:
            return False, ()

        success = spec.intent.success
        if success.judge is SuccessJudge.TOMATO_STATE:
            required_status = success.required_tomato_status
            if required_status is not None and snapshot.tomato_status is required_status:
                self.state.phase_success_stable_steps = 0
                return True, ()
            return False, (
                f"[Approach] Waiting for {spec.phase_id.value} completion.",
                f"  tomato_status={snapshot.tomato_status.value}",
            )

        target_pose = spec.motion.phase_goal_pose
        tolerance_m = success.position_tolerance_m
        if target_pose is None or tolerance_m is None:
            self.state.phase_success_stable_steps = 0
            return True, ()

        error_m = self._pose_error_m(snapshot.robot_tool_pose, target_pose)
        if error_m > tolerance_m:
            self.state.phase_success_stable_steps = 0
            return False, self._waiting_log_for_phase(
                phase_id=spec.phase_id,
                current_pose=snapshot.robot_tool_pose,
                target_pose=target_pose,
                error_m=error_m,
                stable_steps=0,
                required_stable_steps=success.stable_steps,
            )

        self.state.phase_success_stable_steps += 1
        if self.state.phase_success_stable_steps < success.stable_steps:
            return False, self._waiting_log_for_phase(
                phase_id=spec.phase_id,
                current_pose=snapshot.robot_tool_pose,
                target_pose=target_pose,
                error_m=error_m,
                stable_steps=self.state.phase_success_stable_steps,
                required_stable_steps=success.stable_steps,
            )
        self.state.phase_success_stable_steps = 0
        return True, ()

    def _waiting_log_for_phase(
        self,
        *,
        phase_id: PhaseId,
        current_pose: Pose3D,
        target_pose: Pose3D,
        error_m: float,
        stable_steps: int,
        required_stable_steps: int,
    ) -> tuple[str, ...]:
        if phase_id is PhaseId.MOVING_TO_PREGRASP:
            label = "pre-grasp convergence"
        elif phase_id is PhaseId.MOVING_TO_GRASP:
            label = "grasp convergence"
        else:
            label = f"{phase_id.value} convergence"
        suffix = ""
        if required_stable_steps > 1:
            suffix = f" stable_steps={stable_steps}/{required_stable_steps}"
        return (
            f"[Approach] Waiting for {label}.",
            (
                "  tool_xyz="
                f"({current_pose.x:.4f}, {current_pose.y:.4f}, {current_pose.z:.4f}) "
                "target_xyz="
                f"({target_pose.x:.4f}, {target_pose.y:.4f}, {target_pose.z:.4f}) "
                f"error={error_m:.4f} m{suffix}"
            ),
        )

    def _reset_phase_counters(self) -> None:
        self.state.grasp_evaluation_wait_steps = 0
        self.state.grasp_settle_wait_steps = 0
        self.state.phase_success_stable_steps = 0

    def _require_scene_snapshot(self) -> SceneSnapshot:
        if self.state.last_scene_snapshot is None:
            raise RuntimeError("Scene snapshot is not available.")
        return self.state.last_scene_snapshot

    @staticmethod
    def _pose_error_m(current_pose: Pose3D, target_pose: Pose3D) -> float:
        dx = current_pose.x - target_pose.x
        dy = current_pose.y - target_pose.y
        dz = current_pose.z - target_pose.z
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    @staticmethod
    def _home_tool_pose() -> Pose3D:
        return load_scene_layout_config().home_tool_pose


RobotRuntime = HarvestRuntime
