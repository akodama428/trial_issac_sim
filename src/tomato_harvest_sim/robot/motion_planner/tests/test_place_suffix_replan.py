from __future__ import annotations

import unittest
from dataclasses import replace

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan, JointStateSnapshot, JointTrajectory, JointTrajectoryPoint,
    Pose3D, ScenePhase, SceneSnapshot, TfTreeSnapshot, TomatoStatus,
)
from tomato_harvest_sim.robot.msg.planner import MoveIt2PlanningResult
from tomato_harvest_sim.robot.motion_planner.moveit_service_bridge import (
    MoveIt2ServiceBridgePlanner,
)
from tomato_harvest_sim.robot.motion_planner.place_suffix_replan import (
    PlaceSuffixReplanGate, evaluate_place_suffix_update,
)


def _plan(*, endpoint: float, revision: int = 1) -> HarvestMotionPlan:
    pose = Pose3D(0, 0, 0, 0, 0, 0)
    trajectory = JointTrajectory(
        joint_names=("joint1", "joint2"),
        points=(JointTrajectoryPoint((0.0, 0.0), 0.0),
                JointTrajectoryPoint((endpoint, endpoint), 1.0)),
    )
    return HarvestMotionPlan(
        planner_name="test", target_pose=pose, pregrasp_pose=pose,
        grasp_pose=pose, pull_pose=pose, place_pose=pose,
        place_joint_trajectory=trajectory, plan_revision=revision,
    )


class PlaceSuffixUpdateTest(unittest.TestCase):
    def test_small_endpoint_difference_keeps_current_plan(self) -> None:
        decision = evaluate_place_suffix_update(
            current_plan=_plan(endpoint=1.0),
            candidate_plan=_plan(endpoint=1.005, revision=2),
            minimum_endpoint_delta_rad=0.02,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_small_trajectory_delta")

    def test_significant_endpoint_difference_adopts_suffix(self) -> None:
        decision = evaluate_place_suffix_update(
            current_plan=_plan(endpoint=1.0),
            candidate_plan=_plan(endpoint=1.05, revision=2),
            minimum_endpoint_delta_rad=0.02,
        )
        self.assertTrue(decision.adopted)
        self.assertEqual(decision.reason, "adopted_significant_trajectory_delta")

    def test_missing_candidate_place_trajectory_is_rejected(self) -> None:
        decision = evaluate_place_suffix_update(
            current_plan=_plan(endpoint=1.0),
            candidate_plan=replace(_plan(endpoint=1.2), place_joint_trajectory=None),
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_missing_place_trajectory")


class PlaceSuffixReplanGateTest(unittest.TestCase):
    def test_second_planner_start_is_suppressed_while_in_flight(self) -> None:
        gate = PlaceSuffixReplanGate()
        self.assertTrue(gate.try_begin())
        self.assertFalse(gate.try_begin())
        gate.finish()
        self.assertTrue(gate.try_begin())


class PlaceSuffixIntegrationTest(unittest.TestCase):
    def test_pose_deviation_replans_only_place_from_current_joint_state(self) -> None:
        current_joints = JointStateSnapshot(("joint1", "joint2"), (0.25, 0.25))
        suffix = JointTrajectory(
            ("joint1", "joint2"),
            (JointTrajectoryPoint(current_joints.positions_rad, 0.0),
             JointTrajectoryPoint((1.0, 1.0), 1.0)),
        )

        class FakeBridge:
            received_joint_state: JointStateSnapshot | None = None

            def plan_place_trajectory(self, **kwargs: object) -> MoveIt2PlanningResult:
                self.received_joint_state = kwargs["joint_state"]  # type: ignore[assignment]
                return MoveIt2PlanningResult(
                    success=True, backend_name="place_suffix", reason="service_ok",
                    place_joint_trajectory=suffix,
                )

        bridge = FakeBridge()
        planner = MoveIt2ServiceBridgePlanner(bridge=bridge)  # type: ignore[arg-type]
        pose = Pose3D(0, 0, 0, 0, 0, 0)
        scene = SceneSnapshot(
            phase=ScenePhase.RUNNING, active_camera="fixed", tomato_attached=True,
            tomato_status=TomatoStatus.HELD, gripper_closed=True, robot_home=False,
            cycle_id=1, robot_model="panda", robot_base_pose=pose,
            fixed_camera_pose=pose, hand_camera_pose=pose, branch_pose=pose,
            stem_pose=pose, tomato_pose=pose, tray_pose=pose, robot_tool_pose=pose,
            target_tool_pose=None, grasp_result_reason=None,
        )
        tf_tree = TfTreeSnapshot(
            "panda_link0", "fixed_camera", "target", pose, pose, pose
        )
        prior = _plan(endpoint=1.0)

        candidate = planner.plan_place_from_joint_state(
            prior, current_joints, tf_tree, scene
        )

        self.assertIsNotNone(candidate)
        self.assertEqual(bridge.received_joint_state, current_joints)
        self.assertEqual(candidate.place_joint_trajectory, suffix)  # type: ignore[union-attr]
        self.assertEqual(
            candidate.pregrasp_joint_trajectory, prior.pregrasp_joint_trajectory  # type: ignore[union-attr]
        )
        decision = evaluate_place_suffix_update(
            current_plan=prior, candidate_plan=candidate  # type: ignore[arg-type]
        )
        self.assertTrue(decision.adopted)
