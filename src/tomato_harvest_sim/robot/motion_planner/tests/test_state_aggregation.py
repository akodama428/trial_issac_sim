from __future__ import annotations

import unittest
from dataclasses import replace

from tomato_harvest_sim.msg.contracts import (
    HarvestTaskPhase, JointStateSnapshot, Pose3D, ScenePhase, SceneSnapshot,
    TomatoStatus,
)
from tomato_harvest_sim.robot.motion_planner.state_aggregation import PlannerStateAggregator


class PlannerStateAggregatorTest(unittest.TestCase):
    def test_dynamic_robot_pose_does_not_report_scene_change(self) -> None:
        pose = Pose3D(0, 0, 0, 0, 0, 0)
        scene = SceneSnapshot(
            phase=ScenePhase.RUNNING, active_camera="fixed", tomato_attached=False,
            tomato_status=TomatoStatus.ATTACHED, gripper_closed=False,
            robot_home=False, cycle_id=1, robot_model="panda",
            robot_base_pose=pose, fixed_camera_pose=pose, hand_camera_pose=pose,
            branch_pose=pose, stem_pose=pose, tomato_pose=pose, tray_pose=pose,
            robot_tool_pose=pose, target_tool_pose=None, grasp_result_reason=None,
        )
        aggregator = PlannerStateAggregator()
        aggregator.update_scene_snapshot(scene)
        aggregator.update_scene_snapshot(replace(
            scene, robot_tool_pose=replace(pose, x=0.25), gripper_closed=True
        ))
        self.assertEqual(aggregator.snapshot().scene_generation, 0)

    def test_collision_object_change_increments_scene_generation(self) -> None:
        pose = Pose3D(0, 0, 0, 0, 0, 0)
        scene = SceneSnapshot(
            phase=ScenePhase.RUNNING, active_camera="fixed", tomato_attached=False,
            tomato_status=TomatoStatus.ATTACHED, gripper_closed=False,
            robot_home=False, cycle_id=1, robot_model="panda",
            robot_base_pose=pose, fixed_camera_pose=pose, hand_camera_pose=pose,
            branch_pose=pose, stem_pose=pose, tomato_pose=pose, tray_pose=pose,
            robot_tool_pose=pose, target_tool_pose=None, grasp_result_reason=None,
        )
        aggregator = PlannerStateAggregator()
        aggregator.update_scene_snapshot(scene)
        aggregator.update_scene_snapshot(replace(
            scene, tray_pose=replace(pose, x=0.25)
        ))
        self.assertEqual(aggregator.snapshot().scene_generation, 1)

    def test_latest_values_are_exposed_as_one_snapshot(self) -> None:
        aggregator = PlannerStateAggregator()
        joints = JointStateSnapshot(("joint1",), (0.25,))
        aggregator.update_phase(HarvestTaskPhase.MOVING_TO_GRASP)
        aggregator.update_joint_state(joints)
        aggregator.observe_tracking_error(0.12)

        state = aggregator.snapshot()
        self.assertEqual(state.phase, HarvestTaskPhase.MOVING_TO_GRASP)
        self.assertEqual(state.joint_state, joints)
        self.assertEqual(state.tracking_error_rad, 0.12)

    def test_abort_is_recorded_as_a_monotonic_event(self) -> None:
        aggregator = PlannerStateAggregator()
        aggregator.observe_abort()
        aggregator.observe_abort()
        self.assertEqual(aggregator.snapshot().abort_generation, 2)

    def test_stall_is_recorded_once_per_rising_edge(self) -> None:
        aggregator = PlannerStateAggregator()

        aggregator.observe_stall(True)
        aggregator.observe_stall(True)
        self.assertEqual(aggregator.snapshot().stall_generation, 1)
        aggregator.observe_stall(False)
        aggregator.observe_stall(True)
        self.assertEqual(aggregator.snapshot().stall_generation, 2)

    def test_tracking_error_holds_the_peak_of_observed_values(self) -> None:
        aggregator = PlannerStateAggregator()

        aggregator.observe_tracking_error(0.15)
        aggregator.observe_tracking_error(0.03)

        self.assertEqual(aggregator.snapshot().tracking_error_rad, 0.15)

    def test_pending_tracking_error_is_not_carried_to_next_phase(self) -> None:
        aggregator = PlannerStateAggregator()
        aggregator.update_phase(HarvestTaskPhase.MOVING_TO_PREGRASP)
        aggregator.observe_tracking_error(0.08)

        aggregator.update_phase(HarvestTaskPhase.MOVING_TO_GRASP)

        self.assertIsNone(aggregator.snapshot().tracking_error_rad)
