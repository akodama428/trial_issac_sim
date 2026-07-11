from __future__ import annotations

import unittest

from tomato_harvest_sim.msg.contracts import (
    HarvestTaskPhase, JointStateSnapshot, Pose3D, TargetEstimate,
)
from tomato_harvest_sim.robot.motion_planner.replan_trigger import (
    ReplanTrigger, TriggerMemory, evaluate_replan_trigger, should_inject_place_replan,
    trigger_starts_planner,
)
from tomato_harvest_sim.robot.motion_planner.state_aggregation import PlannerState


def _ready_state(**changes: object) -> PlannerState:
    pose = Pose3D(0, 0, 0, 0, 0, 0)
    values = dict(
        phase=HarvestTaskPhase.MOVING_TO_GRASP,
        joint_state=JointStateSnapshot(("joint1",), (0.0,)),
        target_estimate=TargetEstimate("fixed", pose, pose, 1.0),
    )
    values.update(changes)
    return PlannerState(**values)


class ReplanTriggerPolicyTest(unittest.TestCase):
    def test_e2e_place_replan_injection_runs_once_in_place_phase(self) -> None:
        self.assertTrue(should_inject_place_replan(
            enabled=True, already_injected=False,
            phase=HarvestTaskPhase.MOVING_TO_PLACE,
        ))
        self.assertFalse(should_inject_place_replan(
            enabled=True, already_injected=True,
            phase=HarvestTaskPhase.MOVING_TO_PLACE,
        ))
        self.assertFalse(should_inject_place_replan(
            enabled=True, already_injected=False,
            phase=HarvestTaskPhase.MOVING_TO_GRASP,
        ))

    def test_only_abort_starts_full_chain_planner_in_step2(self) -> None:
        self.assertTrue(trigger_starts_planner(
            ReplanTrigger.ABORT, HarvestTaskPhase.MOVING_TO_GRASP
        ))
        self.assertFalse(trigger_starts_planner(
            ReplanTrigger.TIMER, HarvestTaskPhase.MOVING_TO_GRASP
        ))

    def test_place_phase_starts_suffix_planner_for_new_triggers(self) -> None:
        for trigger in (ReplanTrigger.SCENE_CHANGE, ReplanTrigger.TRACKING_ERROR):
            self.assertTrue(trigger_starts_planner(
                trigger, HarvestTaskPhase.MOVING_TO_PLACE
            ))
        self.assertFalse(trigger_starts_planner(
            ReplanTrigger.TIMER, HarvestTaskPhase.MOVING_TO_PLACE
        ))

    def test_timer_triggers_in_enabled_phase(self) -> None:
        decision = evaluate_replan_trigger(
            state=_ready_state(), memory=TriggerMemory(), now_sec=10.0
        )
        self.assertEqual(decision.trigger, ReplanTrigger.TIMER)

    def test_abort_triggers_independently(self) -> None:
        decision = evaluate_replan_trigger(
            state=_ready_state(abort_generation=1),
            memory=TriggerMemory(handled_abort_generation=0), now_sec=10.0,
        )
        self.assertEqual(decision.trigger, ReplanTrigger.ABORT)

    def test_scene_change_triggers_independently(self) -> None:
        decision = evaluate_replan_trigger(
            state=_ready_state(scene_generation=2),
            memory=TriggerMemory(handled_scene_generation=1), now_sec=10.0,
        )
        self.assertEqual(decision.trigger, ReplanTrigger.SCENE_CHANGE)

    def test_tracking_error_triggers_at_threshold(self) -> None:
        decision = evaluate_replan_trigger(
            state=_ready_state(tracking_error_rad=0.10),
            memory=TriggerMemory(), now_sec=10.0,
        )
        self.assertEqual(decision.trigger, ReplanTrigger.TRACKING_ERROR)

    def test_minimum_interval_suppresses_all_triggers(self) -> None:
        decision = evaluate_replan_trigger(
            state=_ready_state(abort_generation=1, tracking_error_rad=1.0),
            memory=TriggerMemory(last_replan_at_sec=9.5), now_sec=10.0,
        )
        self.assertFalse(decision.triggered)
        self.assertEqual(decision.reason, "suppressed_minimum_interval")

    def test_timer_is_suppressed_in_contact_phase(self) -> None:
        decision = evaluate_replan_trigger(
            state=_ready_state(phase=HarvestTaskPhase.DETACHING),
            memory=TriggerMemory(), now_sec=10.0,
        )
        self.assertFalse(decision.triggered)
        self.assertEqual(decision.reason, "suppressed_phase")

    def test_incomplete_state_is_suppressed(self) -> None:
        decision = evaluate_replan_trigger(
            state=PlannerState(phase=HarvestTaskPhase.MOVING_TO_GRASP),
            memory=TriggerMemory(), now_sec=10.0,
        )
        self.assertFalse(decision.triggered)
        self.assertEqual(decision.reason, "suppressed_incomplete_state")

    def test_subthreshold_error_does_not_trigger_outside_timer_phase(self) -> None:
        decision = evaluate_replan_trigger(
            state=_ready_state(
                phase=HarvestTaskPhase.DETACHING, tracking_error_rad=0.099
            ),
            memory=TriggerMemory(), now_sec=10.0,
        )
        self.assertFalse(decision.triggered)
        self.assertEqual(decision.reason, "suppressed_phase")
