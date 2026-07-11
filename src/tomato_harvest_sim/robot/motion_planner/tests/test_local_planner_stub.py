"""local planner stub — producer複線化の最小受け皿のテスト (Issue #13)。"""
from __future__ import annotations

import unittest
from dataclasses import replace

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointTrajectory,
    JointTrajectoryPoint,
    PlanProducerKind,
    Pose3D,
)
from tomato_harvest_sim.robot.motion_planner.local_planner_stub import (
    build_local_refinement_plan,
)


def _base_plan() -> HarvestMotionPlan:
    pose = Pose3D(0, 0, 0, 0, 0, 0)
    trajectory = JointTrajectory(
        joint_names=("joint1", "joint2"),
        points=(JointTrajectoryPoint((0.0, 0.0), 0.0),
                JointTrajectoryPoint((1.0, 1.0), 1.0)),
    )
    return HarvestMotionPlan(
        planner_name="moveit2_service_bridge",
        target_pose=pose, pregrasp_pose=pose, grasp_pose=pose,
        pull_pose=pose, place_pose=pose,
        pregrasp_joint_trajectory=trajectory,
        place_joint_trajectory=trajectory,
        plan_revision=3,
        generated_at_sec=100.0,
        planned_from_phase=HarvestTaskPhase.TARGET_FOUND,
        producer_kind=PlanProducerKind.GLOBAL_PLANNER,
        producer_instance_id="global-instance-a",
    )


class LocalRefinementPlanTest(unittest.TestCase):
    def test_refinement_is_stamped_as_independent_local_producer(self) -> None:
        candidate = build_local_refinement_plan(
            base_plan=_base_plan(),
            phase=HarvestTaskPhase.MOVING_TO_PLACE,
            now_sec=200.0,
            instance_id="local-instance-a",
            revision=1,
        )

        self.assertIsNotNone(candidate)
        assert candidate is not None
        self.assertEqual(candidate.producer_kind, PlanProducerKind.LOCAL_PLANNER)
        self.assertEqual(candidate.producer_instance_id, "local-instance-a")
        self.assertEqual(candidate.plan_revision, 1)
        self.assertEqual(candidate.generated_at_sec, 200.0)
        self.assertEqual(
            candidate.planned_from_phase, HarvestTaskPhase.MOVING_TO_PLACE
        )
        self.assertEqual(candidate.planner_name, "local_planner_stub")

    def test_executor_contract_fields_are_preserved(self) -> None:
        """下流(motion_command / executor)が読むtrajectory契約を変えない。"""
        base = _base_plan()
        candidate = build_local_refinement_plan(
            base_plan=base,
            phase=HarvestTaskPhase.MOVING_TO_PLACE,
            now_sec=200.0,
            instance_id="local-instance-a",
            revision=1,
        )

        assert candidate is not None
        self.assertEqual(
            candidate.place_joint_trajectory, base.place_joint_trajectory
        )
        self.assertEqual(
            candidate.pregrasp_joint_trajectory, base.pregrasp_joint_trajectory
        )
        self.assertEqual(candidate.place_pose, base.place_pose)

    def test_contact_dominant_phase_is_not_refined(self) -> None:
        candidate = build_local_refinement_plan(
            base_plan=_base_plan(),
            phase=HarvestTaskPhase.DETACHING,
            now_sec=200.0,
            instance_id="local-instance-a",
            revision=1,
        )
        self.assertIsNone(candidate)

    def test_missing_phase_trajectory_is_not_refined(self) -> None:
        base = replace(_base_plan(), grasp_joint_trajectory=None)
        candidate = build_local_refinement_plan(
            base_plan=base,
            phase=HarvestTaskPhase.MOVING_TO_GRASP,
            now_sec=200.0,
            instance_id="local-instance-a",
            revision=1,
        )
        self.assertIsNone(candidate)


if __name__ == "__main__":
    unittest.main()
