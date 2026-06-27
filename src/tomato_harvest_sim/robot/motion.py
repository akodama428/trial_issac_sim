from __future__ import annotations

from tomato_harvest_sim.api.contracts import (
    ExecutionPhaseSpec,
    HarvestMotionPlan,
    MotionCommand,
    PhaseId,
    PhaseMotionPlan,
)


def phase_motion_from_harvest_plan(plan: HarvestMotionPlan, phase_id: PhaseId) -> PhaseMotionPlan:
    if phase_id is PhaseId.MOVING_TO_PREGRASP:
        return PhaseMotionPlan(
            phase_goal_pose=plan.pregrasp_pose,
            active_waypoints=plan.pregrasp_waypoints or (plan.pregrasp_pose,),
            joint_trajectory=plan.pregrasp_joint_trajectory,
        )
    if phase_id is PhaseId.MOVING_TO_GRASP:
        return PhaseMotionPlan(
            phase_goal_pose=plan.grasp_pose,
            active_waypoints=plan.grasp_waypoints or (plan.grasp_pose,),
            joint_trajectory=plan.grasp_joint_trajectory,
        )
    if phase_id is PhaseId.PULL_TO_DETACH:
        return PhaseMotionPlan(
            phase_goal_pose=plan.pull_pose,
            active_waypoints=plan.pull_waypoints or (plan.pull_pose,),
            joint_trajectory=plan.pull_joint_trajectory,
        )
    if phase_id is PhaseId.MOVING_TO_PLACE:
        return PhaseMotionPlan(
            phase_goal_pose=plan.place_pose,
            active_waypoints=plan.place_waypoints or (plan.place_pose,),
            joint_trajectory=plan.place_joint_trajectory,
        )
    if phase_id is PhaseId.RETURNING_HOME:
        return PhaseMotionPlan(
            phase_goal_pose=None,
            active_waypoints=(),
            joint_trajectory=None,
        )
    raise ValueError(f"Unsupported phase id: {phase_id}")


def command_name_for_phase(phase_id: PhaseId) -> str:
    if phase_id is PhaseId.MOVING_TO_PREGRASP:
        return "move_to_pregrasp"
    if phase_id is PhaseId.MOVING_TO_GRASP:
        return "move_to_grasp"
    if phase_id is PhaseId.PULL_TO_DETACH:
        return "pull_to_detach"
    if phase_id is PhaseId.MOVING_TO_PLACE:
        return "move_to_place"
    if phase_id is PhaseId.RETURNING_HOME:
        return "move_home"
    raise ValueError(f"Unsupported phase id: {phase_id}")


class MoveItStyleMotionPublisher:
    def build_phase_command(self, *, planner_name: str, spec: ExecutionPhaseSpec) -> MotionCommand:
        motion = spec.motion
        if spec.phase_id is PhaseId.RETURNING_HOME:
            return MotionCommand(
                command_name=command_name_for_phase(spec.phase_id),
                planner_name=planner_name,
                execution_phase_spec=spec,
            )
        return MotionCommand(
            command_name=command_name_for_phase(spec.phase_id),
            planner_name=planner_name,
            target_pose=motion.phase_goal_pose,
            waypoint_poses=motion.active_waypoints,
            joint_trajectory=motion.joint_trajectory,
            execution_phase_spec=spec,
        )

    def build_pregrasp_command(self, plan: HarvestMotionPlan) -> MotionCommand:
        return MotionCommand(
            command_name="move_to_pregrasp",
            planner_name=plan.planner_name,
            target_pose=plan.pregrasp_pose,
            waypoint_poses=plan.pregrasp_waypoints or (plan.pregrasp_pose,),
            joint_trajectory=plan.pregrasp_joint_trajectory,
        )

    def build_grasp_command(self, plan: HarvestMotionPlan) -> MotionCommand:
        return MotionCommand(
            command_name="move_to_grasp",
            planner_name=plan.planner_name,
            target_pose=plan.grasp_pose,
            waypoint_poses=plan.grasp_waypoints or (plan.grasp_pose,),
            joint_trajectory=plan.grasp_joint_trajectory,
        )

    def build_close_gripper_command(self, plan: HarvestMotionPlan) -> MotionCommand:
        return MotionCommand(
            command_name="close_gripper",
            planner_name=plan.planner_name,
            gripper_closed=True,
        )

    def build_pull_command(self, plan: HarvestMotionPlan) -> MotionCommand:
        return MotionCommand(
            command_name="pull_to_detach",
            planner_name=plan.planner_name,
            target_pose=plan.pull_pose,
            waypoint_poses=plan.pull_waypoints or (plan.pull_pose,),
            joint_trajectory=plan.pull_joint_trajectory,
        )

    def build_place_command(self, plan: HarvestMotionPlan) -> MotionCommand:
        return MotionCommand(
            command_name="move_to_place",
            planner_name=plan.planner_name,
            target_pose=plan.place_pose,
            waypoint_poses=plan.place_waypoints or (plan.place_pose,),
            joint_trajectory=plan.place_joint_trajectory,
        )

    def build_open_gripper_command(self, plan: HarvestMotionPlan) -> MotionCommand:
        return MotionCommand(
            command_name="open_gripper",
            planner_name=plan.planner_name,
            gripper_closed=False,
        )

    def build_home_command(self, plan: HarvestMotionPlan) -> MotionCommand:
        return MotionCommand(
            command_name="move_home",
            planner_name=plan.planner_name,
        )
