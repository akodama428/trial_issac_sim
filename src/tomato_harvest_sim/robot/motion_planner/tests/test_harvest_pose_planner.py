from tomato_harvest_sim.robot.motion_planner.harvest_pose_planner import (
    HarvestPoseWaypointPlanner,
)

from tomato_harvest_sim.robot.motion_planner.tests.test_moveit_planner_backend import (
    _scene_snapshot,
    _target_estimate,
)


def test_default_grasp_height_compensates_measured_finger_center_offset() -> None:
    plan = HarvestPoseWaypointPlanner().plan(_target_estimate(), _scene_snapshot())

    assert plan.grasp_pose.z == 0.588
    assert plan.place_pose.z == 0.60


def test_pose_waypoint_planner_does_not_generate_moveit_results() -> None:
    plan = HarvestPoseWaypointPlanner().plan(_target_estimate(), _scene_snapshot())

    assert plan.pregrasp_joint_trajectory is None
    assert plan.grasp_joint_trajectory is None
    assert plan.pull_joint_trajectory is None
    assert plan.place_joint_trajectory is None
    assert plan.planning_scene_object_ids == ()
