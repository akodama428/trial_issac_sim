from __future__ import annotations

from tomato_harvest_sim.api.contracts import HarvestMotionPlan, MotionCommand


class MoveItStyleMotionPublisher:
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
