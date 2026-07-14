"""plan adoption policy — stale plan を採用しない最低限の規則のテスト (Issue #9)。"""
from __future__ import annotations

import unittest

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    PlanProducerKind,
    Pose3D,
)
from tomato_harvest_sim.robot.execute_manager.plan_adoption import (
    evaluate_plan_adoption,
)

_POSE = Pose3D(x=0.1, y=0.2, z=0.3, roll=0.0, pitch=0.0, yaw=0.0)


def make_plan(
    *,
    plan_revision: int = 0,
    generated_at_sec: float | None = 100.0,
    planned_from_phase: HarvestTaskPhase | None = None,
    producer_kind: PlanProducerKind = PlanProducerKind.GLOBAL_PLANNER,
    producer_instance_id: str | None = "global-instance-a",
) -> HarvestMotionPlan:
    return HarvestMotionPlan(
        planner_name="moveit2_service_bridge",
        target_pose=_POSE,
        pregrasp_pose=_POSE,
        grasp_pose=_POSE,
        pull_pose=_POSE,
        place_pose=_POSE,
        plan_revision=plan_revision,
        generated_at_sec=generated_at_sec,
        planned_from_phase=planned_from_phase,
        producer_kind=producer_kind,
        producer_instance_id=producer_instance_id,
    )


class TestRevisionRule(unittest.TestCase):
    def test_first_plan_is_adopted(self) -> None:
        decision = evaluate_plan_adoption(
            candidate=make_plan(plan_revision=1,
                                planned_from_phase=HarvestTaskPhase.TARGET_FOUND),
            current_plan=None,
            current_phase=HarvestTaskPhase.TARGET_FOUND,
        )
        self.assertTrue(decision.adopted)
        self.assertEqual(decision.reason, "adopted_initial")

    def test_newer_revision_replaces_current_plan(self) -> None:
        decision = evaluate_plan_adoption(
            candidate=make_plan(plan_revision=2,
                                planned_from_phase=HarvestTaskPhase.MOVING_TO_GRASP),
            current_plan=make_plan(plan_revision=1),
            current_phase=HarvestTaskPhase.MOVING_TO_GRASP,
        )
        self.assertTrue(decision.adopted)
        self.assertEqual(decision.reason, "adopted_newer_revision")

    def test_older_revision_is_rejected_as_stale(self) -> None:
        decision = evaluate_plan_adoption(
            candidate=make_plan(plan_revision=1,
                                planned_from_phase=HarvestTaskPhase.TARGET_FOUND),
            current_plan=make_plan(plan_revision=2),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_stale_revision")

    def test_duplicate_revision_is_rejected_as_stale(self) -> None:
        decision = evaluate_plan_adoption(
            candidate=make_plan(plan_revision=2,
                                planned_from_phase=HarvestTaskPhase.MOVING_TO_GRASP),
            current_plan=make_plan(plan_revision=2),
            current_phase=HarvestTaskPhase.MOVING_TO_GRASP,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_stale_revision")

    def test_new_planner_instance_can_restart_revision_from_one(self) -> None:
        decision = evaluate_plan_adoption(
            candidate=make_plan(
                plan_revision=1,
                generated_at_sec=200.0,
                planned_from_phase=HarvestTaskPhase.MOVING_TO_PLACE,
                producer_instance_id="global-instance-b",
            ),
            current_plan=make_plan(
                plan_revision=20,
                generated_at_sec=100.0,
                producer_instance_id="global-instance-a",
            ),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertTrue(decision.adopted)
        self.assertEqual(decision.reason, "adopted_newer_producer_instance")

    def test_delayed_plan_from_old_planner_instance_is_rejected(self) -> None:
        decision = evaluate_plan_adoption(
            candidate=make_plan(
                plan_revision=21,
                generated_at_sec=100.0,
                planned_from_phase=HarvestTaskPhase.MOVING_TO_PLACE,
                producer_instance_id="global-instance-a",
            ),
            current_plan=make_plan(
                plan_revision=1,
                generated_at_sec=200.0,
                producer_instance_id="global-instance-b",
            ),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_stale_producer_instance")


class TestPhaseConsistencyRule(unittest.TestCase):
    def test_replan_bound_to_passed_phase_is_rejected(self) -> None:
        """moving_to_grasp 中の replan が detaching 到達後に届いたら巻き戻さない。"""
        decision = evaluate_plan_adoption(
            candidate=make_plan(plan_revision=3,
                                planned_from_phase=HarvestTaskPhase.MOVING_TO_GRASP),
            current_plan=make_plan(plan_revision=2),
            current_phase=HarvestTaskPhase.DETACHING,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_phase_mismatch")

    def test_replan_for_current_phase_is_adopted(self) -> None:
        decision = evaluate_plan_adoption(
            candidate=make_plan(plan_revision=3,
                                planned_from_phase=HarvestTaskPhase.MOVING_TO_PLACE),
            current_plan=make_plan(plan_revision=2),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertTrue(decision.adopted)

    def test_full_chain_plan_is_adoptable_in_any_phase(self) -> None:
        """target_found 起点の full-chain plan は phase-bound ではない。"""
        decision = evaluate_plan_adoption(
            candidate=make_plan(plan_revision=3,
                                planned_from_phase=HarvestTaskPhase.TARGET_FOUND),
            current_plan=make_plan(plan_revision=2),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertTrue(decision.adopted)

    def test_phase_bound_plan_is_rejected_until_consumer_phase_is_known(self) -> None:
        decision = evaluate_plan_adoption(
            candidate=make_plan(plan_revision=1,
                                planned_from_phase=HarvestTaskPhase.MOVING_TO_GRASP),
            current_plan=None,
            current_phase=None,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_current_phase_unknown")

    def test_versioned_plan_without_planned_phase_is_rejected(self) -> None:
        decision = evaluate_plan_adoption(
            candidate=make_plan(plan_revision=1, planned_from_phase=None),
            current_plan=None,
            current_phase=HarvestTaskPhase.TARGET_FOUND,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_missing_plan_metadata")


class TestProducerAndMetadataRule(unittest.TestCase):
    def test_unversioned_plan_is_rejected_as_missing_metadata(self) -> None:
        """revision 0 (未刻印) の plan は契約違反として fail-closed で棄却する。"""
        decision = evaluate_plan_adoption(
            candidate=make_plan(plan_revision=0),
            current_plan=None,
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_missing_plan_metadata")

    def test_unversioned_plan_cannot_overwrite_versioned_plan(self) -> None:
        decision = evaluate_plan_adoption(
            candidate=make_plan(plan_revision=0),
            current_plan=make_plan(plan_revision=10),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_missing_plan_metadata")

    def test_unknown_producer_kind_is_rejected(self) -> None:
        decision = evaluate_plan_adoption(
            candidate=make_plan(plan_revision=5,
                                planned_from_phase=HarvestTaskPhase.MOVING_TO_PLACE,
                                producer_kind=PlanProducerKind.UNKNOWN),
            current_plan=None,
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_unknown_producer")

    def test_global_producer_uses_contract_rules(self) -> None:
        decision = evaluate_plan_adoption(
            candidate=make_plan(
                plan_revision=1,
                planned_from_phase=HarvestTaskPhase.MOVING_TO_PLACE,
                producer_kind=PlanProducerKind.GLOBAL_PLANNER,
                producer_instance_id="global-instance-a",
            ),
            current_plan=None,
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertTrue(decision.adopted)
        self.assertEqual(decision.reason, "adopted_initial")


if __name__ == "__main__":
    unittest.main()
