from __future__ import annotations

from tomato_harvest_sim.msg.bridge import BridgeProtocol
from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    PhaseExecutionIntent,
    PhaseId,
    PhaseMotionPlan,
    Pose3D,
    SceneSnapshot,
    SuccessJudge,
    TargetEstimate,
    TomatoStatus,
)
from tomato_harvest_sim.robot.behavior_planner.intent_builder import PhaseExecutionIntentBuilder
from tomato_harvest_sim.robot.behavior_planner.phase_motion import (
    MoveItStyleMotionPublisher,
    phase_motion_from_harvest_plan,
)
from tomato_harvest_sim.simulator.scene_config import load_scene_layout_config


class BehaviorPlanner:
    GRASP_SETTLE_STEPS = 5
    GRASP_EVALUATION_TIMEOUT_STEPS = 60
    GRASP_EVALUATION_LOG_INTERVAL_STEPS = 10

    _TASK_PHASE_TO_PHASE_ID: dict[HarvestTaskPhase, PhaseId] = {
        HarvestTaskPhase.MOVING_TO_PREGRASP: PhaseId.MOVING_TO_PREGRASP,
        HarvestTaskPhase.MOVING_TO_GRASP: PhaseId.MOVING_TO_GRASP,
        HarvestTaskPhase.DETACHING: PhaseId.PULL_TO_DETACH,
        HarvestTaskPhase.MOVING_TO_PLACE: PhaseId.MOVING_TO_PLACE,
        HarvestTaskPhase.RETURNING_HOME: PhaseId.RETURNING_HOME,
    }

    def __init__(
        self,
        *,
        state=None,
        intent_builder: PhaseExecutionIntentBuilder | None = None,
        motion_publisher: MoveItStyleMotionPublisher | None = None,
    ) -> None:
        self._shared_state = state
        self._intent_builder = intent_builder or PhaseExecutionIntentBuilder()
        self._motion_publisher = motion_publisher or MoveItStyleMotionPublisher()

    def intent_for_task_phase(self, task_phase: HarvestTaskPhase) -> PhaseExecutionIntent | None:
        phase_id = self._TASK_PHASE_TO_PHASE_ID.get(task_phase)
        if phase_id is None:
            return None
        return self._intent_builder.build(phase_id)

    def step(
        self,
        snapshot: SceneSnapshot,
        bridge: BridgeProtocol,
        *,
        estimate: TargetEstimate | None = None,
    ) -> tuple[str, ...]:
        state = self._shared_state

        if state.task_phase is HarvestTaskPhase.DETECTING:
            if estimate is None:
                return ()
            state.last_target_estimate = estimate
            state.task_phase = HarvestTaskPhase.TARGET_FOUND
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

        if state.task_phase is HarvestTaskPhase.PLANNING:
            command = self._publish_phase_command(
                bridge,
                next_task_phase=HarvestTaskPhase.MOVING_TO_PREGRASP,
                plan=state.last_harvest_motion_plan,
            )
            if command is None:
                return ()
            return (
                "[State] planning -> moving_to_pregrasp",
                "[Approach] Published pre-grasp command to simulator side.",
            )

        if state.task_phase is HarvestTaskPhase.MOVING_TO_PREGRASP:
            completed, waiting_logs = self._evaluate_motion_phase(snapshot)
            if not completed:
                return waiting_logs
            state.task_phase = HarvestTaskPhase.PREGRASP_REACHED
            return (
                "[State] moving_to_pregrasp -> pregrasp_reached",
                "[Complete] pre-grasp reached.",
            )

        if state.task_phase is HarvestTaskPhase.PREGRASP_REACHED:
            command = self._publish_phase_command(
                bridge,
                next_task_phase=HarvestTaskPhase.MOVING_TO_GRASP,
                plan=state.last_harvest_motion_plan,
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

        if state.task_phase is HarvestTaskPhase.MOVING_TO_GRASP:
            completed, waiting_logs = self._evaluate_motion_phase(snapshot)
            if not completed:
                return waiting_logs
            state.task_phase = HarvestTaskPhase.AT_GRASP
            state.grasp_settle_wait_steps = 0
            state.phase_success_stable_steps = 0
            return (
                "[State] moving_to_grasp -> at_grasp",
                "[Complete] grasp pose reached.",
            )

        if state.task_phase is HarvestTaskPhase.AT_GRASP:
            if state.grasp_settle_wait_steps < self.GRASP_SETTLE_STEPS:
                state.grasp_settle_wait_steps += 1
                if state.grasp_settle_wait_steps == 1:
                    return ("[Grasp] Settling at grasp pose before closing the gripper.",)
                return ()
            if state.last_harvest_motion_plan is None:
                return ()
            command = self._motion_publisher.build_close_gripper_command(state.last_harvest_motion_plan)
            state.last_motion_command = command
            state.last_phase_motion_plan = None
            state.task_phase = HarvestTaskPhase.GRASP_EVALUATION
            state.grasp_evaluation_wait_steps = 0
            state.grasp_settle_wait_steps = 0
            bridge.publish_motion_command(command)
            return (
                "[State] at_grasp -> grasp_evaluation",
                "[Grasp] Closing the gripper for grasp evaluation.",
            )

        if state.task_phase is HarvestTaskPhase.GRASP_EVALUATION:
            if snapshot.tomato_status is TomatoStatus.HELD:
                state.grasp_evaluation_wait_steps = 0
                command = self._publish_phase_command(
                    bridge,
                    next_task_phase=HarvestTaskPhase.DETACHING,
                    plan=state.last_harvest_motion_plan,
                )
                if command is None:
                    return ()
                return (
                    "[State] grasp_evaluation -> detaching",
                    "[Grasp] Stable grasp established.",
                    "[Detach] Published pull command to detach the tomato.",
                )
            if snapshot.tomato_status is TomatoStatus.FALLEN:
                state.grasp_evaluation_wait_steps = 0
                state.task_phase = HarvestTaskPhase.FAILED
                reason = snapshot.grasp_result_reason or "grasp_failed"
                return (
                    "[State] grasp_evaluation -> failed",
                    f"[Failed] The tomato was not stably grasped by both fingers. reason={reason}",
                )
            state.grasp_evaluation_wait_steps += 1
            if state.grasp_evaluation_wait_steps >= self.GRASP_EVALUATION_TIMEOUT_STEPS:
                state.grasp_evaluation_wait_steps = 0
                state.task_phase = HarvestTaskPhase.FAILED
                reason = snapshot.grasp_result_reason or "grasp_evaluation_timeout"
                return (
                    "[State] grasp_evaluation -> failed",
                    "[Failed] Grasp evaluation timed out waiting for the physics grasp result. "
                    f"reason={reason}",
                )
            if state.grasp_evaluation_wait_steps % self.GRASP_EVALUATION_LOG_INTERVAL_STEPS == 0:
                return (
                    "[Grasp] Waiting for the physics grasp result.",
                    (
                        f"  tomato_status={snapshot.tomato_status.value} "
                        f"gripper_closed={snapshot.gripper_closed} "
                        f"wait_steps={state.grasp_evaluation_wait_steps}"
                    ),
                )
            return ()

        if state.task_phase is HarvestTaskPhase.DETACHING:
            if snapshot.tomato_status is TomatoStatus.FALLEN:
                state.task_phase = HarvestTaskPhase.FAILED
                reason = snapshot.grasp_result_reason or "detach_failed"
                return (
                    "[State] detaching -> failed",
                    f"[Failed] Detach failed and the tomato fell. reason={reason}",
                )
            completed, waiting_logs = self._evaluate_motion_phase(snapshot)
            if not completed:
                return waiting_logs
            state.task_phase = HarvestTaskPhase.DETACHED
            return (
                "[State] detaching -> detached",
                "[Detach] Tomato detached from stem.",
            )

        if state.task_phase is HarvestTaskPhase.DETACHED:
            command = self._publish_phase_command(
                bridge,
                next_task_phase=HarvestTaskPhase.MOVING_TO_PLACE,
                plan=state.last_harvest_motion_plan,
            )
            if command is None:
                return ()
            return (
                "[State] detached -> moving_to_place",
                "[Place] Published place command to simulator side.",
            )

        if state.task_phase is HarvestTaskPhase.MOVING_TO_PLACE:
            completed, waiting_logs = self._evaluate_motion_phase(snapshot)
            if not completed:
                return waiting_logs
            if state.last_harvest_motion_plan is None:
                return ()
            command = self._motion_publisher.build_open_gripper_command(state.last_harvest_motion_plan)
            state.last_motion_command = command
            state.last_phase_motion_plan = None
            state.task_phase = HarvestTaskPhase.PLACED
            bridge.publish_motion_command(command)
            return (
                "[State] moving_to_place -> placed",
                "[Place] Opening the gripper above the tray.",
            )

        if state.task_phase is HarvestTaskPhase.PLACED:
            if snapshot.tomato_status is TomatoStatus.PLACED:
                command = self._publish_phase_command(
                    bridge,
                    next_task_phase=HarvestTaskPhase.RETURNING_HOME,
                    plan=state.last_harvest_motion_plan,
                )
                if command is None:
                    return ()
                return (
                    "[State] placed -> returning_home",
                    "[Complete] Tomato placed in the tray.",
                    "[Home] Returning the robot to the home pose.",
                )
            if snapshot.tomato_status is TomatoStatus.FALLEN:
                state.task_phase = HarvestTaskPhase.FAILED
                reason = snapshot.grasp_result_reason or "place_failed"
                return (
                    "[State] placed -> failed",
                    f"[Failed] Tomato placement failed. reason={reason}",
                )
            return ()

        if state.task_phase is HarvestTaskPhase.RETURNING_HOME:
            if snapshot.robot_home:
                state.task_phase = HarvestTaskPhase.COMPLETE
                return (
                    "[State] returning_home -> complete",
                    "[Complete] Harvest scenario completed and the robot returned home.",
                )
            completed, waiting_logs = self._evaluate_motion_phase(snapshot)
            if not completed:
                return waiting_logs
            state.task_phase = HarvestTaskPhase.COMPLETE
            return (
                "[State] returning_home -> complete",
                "[Complete] Harvest scenario completed and the robot returned home.",
            )

        return ()

    def replan(
        self,
        snapshot: SceneSnapshot,
        bridge: BridgeProtocol,
        *,
        reason: str,
        motion_plan: HarvestMotionPlan,
    ) -> tuple[str, ...]:
        state = self._shared_state
        phase = state.task_phase
        if phase not in {
            HarvestTaskPhase.MOVING_TO_PREGRASP,
            HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.DETACHING,
            HarvestTaskPhase.MOVING_TO_PLACE,
        }:
            return ()
        phase_id = self._TASK_PHASE_TO_PHASE_ID.get(phase)
        if phase_id is None:
            return ()
        phase_motion_plan = phase_motion_from_harvest_plan(motion_plan, phase_id)
        command = self._motion_publisher.build_phase_command(
            planner_name=motion_plan.planner_name,
            phase_motion_plan=phase_motion_plan,
        )
        state.last_harvest_motion_plan = motion_plan
        state.last_motion_command = command
        state.last_phase_motion_plan = phase_motion_plan
        state.phase_success_stable_steps = 0
        state.planner_backend_name = motion_plan.planner_name
        bridge.publish_motion_command(command)
        return (
            f"[Replan] Active motion replan requested. phase={phase.value} reason={reason}",
            (
                f"[Replan] Published {command.command_name} using planner={motion_plan.planner_name} "
                f"trajectory={'yes' if (command.phase_motion_plan is not None and command.phase_motion_plan.joint_trajectory is not None) else 'no'}"
            ),
        )

    def _publish_phase_command(self, bridge, *, next_task_phase, plan):
        state = self._shared_state
        phase_id = self._TASK_PHASE_TO_PHASE_ID.get(next_task_phase)
        if phase_id is None:
            return None
        if phase_id is PhaseId.RETURNING_HOME:
            home_pose = load_scene_layout_config().home_tool_pose
            phase_motion_plan = PhaseMotionPlan(
                phase_id=phase_id,
                phase_goal_pose=home_pose,
                active_waypoints=(home_pose,),
                joint_trajectory=None,
            )
        else:
            if plan is None:
                return None
            phase_motion_plan = phase_motion_from_harvest_plan(plan, phase_id)
        planner_name = plan.planner_name if plan is not None else state.planner_backend_name
        command = self._motion_publisher.build_phase_command(
            planner_name=planner_name,
            phase_motion_plan=phase_motion_plan,
        )
        state.last_motion_command = command
        state.last_phase_motion_plan = phase_motion_plan
        state.phase_success_stable_steps = 0
        state.task_phase = next_task_phase
        bridge.publish_motion_command(command)
        return command

    def _evaluate_motion_phase(self, snapshot) -> tuple[bool, tuple[str, ...]]:
        state = self._shared_state
        phase_motion_plan = state.last_phase_motion_plan
        if phase_motion_plan is None:
            return False, ()

        intent = self._intent_builder.build(phase_motion_plan.phase_id)
        success = intent.success

        if success.judge is SuccessJudge.TOMATO_STATE:
            required_status = success.required_tomato_status
            if required_status is not None and snapshot.tomato_status is required_status:
                state.phase_success_stable_steps = 0
                return True, ()
            return False, (
                f"[Approach] Waiting for {phase_motion_plan.phase_id.value} completion.",
                f"  tomato_status={snapshot.tomato_status.value}",
            )

        target_pose = phase_motion_plan.phase_goal_pose
        tolerance_m = success.position_tolerance_m
        if target_pose is None or tolerance_m is None:
            state.phase_success_stable_steps = 0
            return True, ()

        error_m = self._pose_error_m(snapshot.robot_tool_pose, target_pose)
        if error_m > tolerance_m:
            state.phase_success_stable_steps = 0
            return False, self._waiting_log_for_phase(
                phase_id=phase_motion_plan.phase_id,
                current_pose=snapshot.robot_tool_pose,
                target_pose=target_pose,
                error_m=error_m,
                stable_steps=0,
                required_stable_steps=success.stable_steps,
            )

        state.phase_success_stable_steps += 1
        if state.phase_success_stable_steps < success.stable_steps:
            return False, self._waiting_log_for_phase(
                phase_id=phase_motion_plan.phase_id,
                current_pose=snapshot.robot_tool_pose,
                target_pose=target_pose,
                error_m=error_m,
                stable_steps=state.phase_success_stable_steps,
                required_stable_steps=success.stable_steps,
            )
        state.phase_success_stable_steps = 0
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

    @staticmethod
    def _pose_error_m(current_pose: Pose3D, target_pose: Pose3D) -> float:
        dx = current_pose.x - target_pose.x
        dy = current_pose.y - target_pose.y
        dz = current_pose.z - target_pose.z
        return (dx * dx + dy * dy + dz * dz) ** 0.5
