"""単一global plan producerの裁定規則を検証する。"""
from __future__ import annotations

import unittest

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan, HarvestTaskPhase, PlanProducerKind, Pose3D,
)
from tomato_harvest_sim.robot.execute_manager.plan_arbitration import evaluate_plan_arbitration

_POSE = Pose3D(x=0.1, y=0.2, z=0.3, roll=0.0, pitch=0.0, yaw=0.0)


def make_plan(*, revision: int = 1, producer: PlanProducerKind = PlanProducerKind.GLOBAL_PLANNER) -> HarvestMotionPlan:
    return HarvestMotionPlan(
        planner_name="test", target_pose=_POSE, pregrasp_pose=_POSE, grasp_pose=_POSE,
        pull_pose=_POSE, place_pose=_POSE, plan_revision=revision,
        generated_at_sec=100.0, planned_from_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        producer_kind=producer, producer_instance_id="global-instance-a",
    )


class GlobalProducerArbitrationTest(unittest.TestCase):
    def test_global_plan_uses_adoption_rules(self) -> None:
        decision = evaluate_plan_arbitration(
            candidate=make_plan(), current_plan=None,
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertTrue(decision.adopted)
        self.assertEqual(decision.reason, "adopted_initial")

    def test_stale_global_plan_is_rejected(self) -> None:
        decision = evaluate_plan_arbitration(
            candidate=make_plan(revision=1), current_plan=make_plan(revision=2),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_stale_revision")

    def test_unknown_producer_is_rejected(self) -> None:
        decision = evaluate_plan_arbitration(
            candidate=make_plan(producer=PlanProducerKind.UNKNOWN), current_plan=None,
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_unknown_producer")


if __name__ == "__main__":
    unittest.main()
