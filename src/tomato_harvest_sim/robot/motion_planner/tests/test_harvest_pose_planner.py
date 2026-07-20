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


def test_place_waypoints_limit_vertical_segment_to_5_mm() -> None:
    """Tray直上の往復経路は、同じ鉛直線上の細分化waypointを共有する。"""
    plan = HarvestPoseWaypointPlanner().plan(_target_estimate(), _scene_snapshot())

    assert plan.place_waypoints[0].z == 0.70
    assert plan.place_waypoints[-1].z == 0.60
    assert len(plan.place_waypoints) == 21
    assert all(
        round(upper.z - lower.z, 6) == 0.005
        for upper, lower in zip(
            plan.place_waypoints,
            plan.place_waypoints[1:],
        )
    )
    assert all(
        (pose.x, pose.y, pose.roll, pose.pitch, pose.yaw)
        == (
            plan.place_pose.x,
            plan.place_pose.y,
            plan.place_pose.roll,
            plan.place_pose.pitch,
            plan.place_pose.yaw,
        )
        for pose in plan.place_waypoints
    )


def test_pose_waypoint_planner_does_not_generate_moveit_results() -> None:
    plan = HarvestPoseWaypointPlanner().plan(_target_estimate(), _scene_snapshot())

    assert plan.pregrasp_joint_trajectory is None
    assert plan.grasp_joint_trajectory is None
    assert plan.pull_joint_trajectory is None
    assert plan.place_joint_trajectory is None
    assert plan.planning_scene_object_ids == ()
