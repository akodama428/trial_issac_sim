"""plan arbitration policy — 複数 plan producer の裁定規則のテスト (Issue #13)。"""
from __future__ import annotations

import unittest

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    PlanProducerKind,
    Pose3D,
)
from tomato_harvest_sim.robot.execute_manager.plan_arbitration import (
    evaluate_plan_arbitration,
)

_POSE = Pose3D(x=0.1, y=0.2, z=0.3, roll=0.0, pitch=0.0, yaw=0.0)


def make_plan(
    *,
    plan_revision: int = 1,
    generated_at_sec: float | None = 100.0,
    planned_from_phase: HarvestTaskPhase | None = HarvestTaskPhase.MOVING_TO_PLACE,
    producer_kind: PlanProducerKind = PlanProducerKind.GLOBAL_PLANNER,
    producer_instance_id: str | None = "global-instance-a",
) -> HarvestMotionPlan:
    return HarvestMotionPlan(
        planner_name="test",
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


def make_local_plan(**changes: object) -> HarvestMotionPlan:
    values: dict[str, object] = dict(
        producer_kind=PlanProducerKind.LOCAL_PLANNER,
        producer_instance_id="local-instance-a",
        generated_at_sec=200.0,
    )
    values.update(changes)
    return make_plan(**values)  # type: ignore[arg-type]


class GlobalProducerArbitrationTest(unittest.TestCase):
    def test_global_plan_keeps_existing_adoption_rules(self) -> None:
        decision = evaluate_plan_arbitration(
            candidate=make_plan(planned_from_phase=HarvestTaskPhase.TARGET_FOUND),
            current_plan=None,
            current_phase=HarvestTaskPhase.TARGET_FOUND,
        )
        self.assertTrue(decision.adopted)
        self.assertEqual(decision.reason, "adopted_initial")

    def test_stale_global_plan_is_still_rejected(self) -> None:
        decision = evaluate_plan_arbitration(
            candidate=make_plan(plan_revision=1),
            current_plan=make_plan(plan_revision=2),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_stale_revision")

    def test_unknown_producer_kind_is_rejected(self) -> None:
        decision = evaluate_plan_arbitration(
            candidate=make_plan(producer_kind=PlanProducerKind.UNKNOWN),
            current_plan=None,
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_unknown_producer")


class LocalProducerArbitrationTest(unittest.TestCase):
    def test_local_refinement_of_current_phase_is_adopted(self) -> None:
        decision = evaluate_plan_arbitration(
            candidate=make_local_plan(),
            current_plan=make_plan(plan_revision=3, generated_at_sec=100.0),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertTrue(decision.adopted)
        self.assertEqual(decision.reason, "adopted_newer_producer_instance")

    def test_local_plan_without_adopted_global_base_is_rejected(self) -> None:
        """global planの土台なしにlocal planだけで実行を始めてはならない。"""
        decision = evaluate_plan_arbitration(
            candidate=make_local_plan(),
            current_plan=None,
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_local_without_adopted_plan")

    def test_local_plan_not_bound_to_current_phase_is_rejected(self) -> None:
        """local planは実行中phaseの補正専用であり、pre-motion起点を許さない。"""
        decision = evaluate_plan_arbitration(
            candidate=make_local_plan(
                planned_from_phase=HarvestTaskPhase.TARGET_FOUND,
            ),
            current_plan=make_plan(plan_revision=3, generated_at_sec=100.0),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_local_phase_mismatch")

    def test_stale_local_plan_is_rejected_by_generation_time(self) -> None:
        decision = evaluate_plan_arbitration(
            candidate=make_local_plan(generated_at_sec=50.0),
            current_plan=make_plan(plan_revision=3, generated_at_sec=100.0),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_stale_producer_instance")

    def test_local_plan_with_missing_metadata_is_rejected_fail_closed(self) -> None:
        decision = evaluate_plan_arbitration(
            candidate=make_local_plan(plan_revision=0),
            current_plan=make_plan(plan_revision=3, generated_at_sec=100.0),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertFalse(decision.adopted)
        self.assertEqual(decision.reason, "rejected_missing_plan_metadata")

    def test_global_plan_takes_over_after_local_adoption(self) -> None:
        """local採用後も、より新しいglobal planは通常規則で採用される。"""
        decision = evaluate_plan_arbitration(
            candidate=make_plan(
                plan_revision=4,
                generated_at_sec=300.0,
                producer_instance_id="global-instance-a",
            ),
            current_plan=make_local_plan(),
            current_phase=HarvestTaskPhase.MOVING_TO_PLACE,
        )
        self.assertTrue(decision.adopted)
        self.assertEqual(decision.reason, "adopted_newer_producer_instance")


if __name__ == "__main__":
    unittest.main()
