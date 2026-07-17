from tomato_harvest_sim.robot.motion_planner.pregrasp_planner import MoveItStylePreGraspPlanner

from tomato_harvest_sim.robot.motion_planner.tests.test_moveit_planner_backend import (
    _joint_state,
    _scene_snapshot,
    _target_estimate,
    _tf_tree,
)


def test_default_grasp_height_compensates_measured_finger_center_offset() -> None:
    plan = MoveItStylePreGraspPlanner().plan(
        _target_estimate(), _joint_state(), _tf_tree(), _scene_snapshot()
    )

    assert plan.grasp_pose.z == 0.588
