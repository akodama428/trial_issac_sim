from __future__ import annotations

import unittest

from tomato_harvest_sim.msg.contracts import (
    HarvestTaskPhase, JointStateSnapshot, Pose3D, TargetEstimate,
)
from tomato_harvest_sim.robot.motion_planner.replan_trigger import (
    ReplanTrigger, TriggerMemory, evaluate_replan_trigger,
    parse_suffix_injection_phases, should_inject_suffix_replan,
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
    def test_e2e_suffix_replan_injection_runs_once_per_enabled_phase(self) -> None:
        enabled = frozenset({
            HarvestTaskPhase.MOVING_TO_PREGRASP, HarvestTaskPhase.MOVING_TO_PLACE,
        })
        self.assertTrue(should_inject_suffix_replan(
            enabled_phases=enabled, injected_phases=frozenset(),
            phase=HarvestTaskPhase.MOVING_TO_PREGRASP,
        ))
        self.assertFalse(should_inject_suffix_replan(
            enabled_phases=enabled,
            injected_phases=frozenset({HarvestTaskPhase.MOVING_TO_PREGRASP}),
            phase=HarvestTaskPhase.MOVING_TO_PREGRASP,
        ))
        self.assertTrue(should_inject_suffix_replan(
            enabled_phases=enabled,
            injected_phases=frozenset({HarvestTaskPhase.MOVING_TO_PREGRASP}),
            phase=HarvestTaskPhase.MOVING_TO_PLACE,
        ))
        self.assertFalse(should_inject_suffix_replan(
            enabled_phases=enabled, injected_phases=frozenset(),
            phase=HarvestTaskPhase.MOVING_TO_GRASP,
        ))

    def test_injection_phases_are_parsed_from_environment_value(self) -> None:
        self.assertEqual(
            parse_suffix_injection_phases("moving_to_pregrasp, moving_to_place"),
            frozenset({
                HarvestTaskPhase.MOVING_TO_PREGRASP,
                HarvestTaskPhase.MOVING_TO_PLACE,
            }),
        )
        self.assertEqual(parse_suffix_injection_phases(""), frozenset())
        self.assertEqual(parse_suffix_injection_phases("detaching, bogus"), frozenset())

    def test_returning_home_is_a_valid_correction_phase(self) -> None:
        """home復帰もsuffix replan対象になったため有効化phaseとして受理する (Issue #32)。"""
        self.assertEqual(
            parse_suffix_injection_phases("returning_home"),
            frozenset({HarvestTaskPhase.RETURNING_HOME}),
        )

    def test_abort_starts_full_chain_planner_in_any_phase(self) -> None:
        self.assertTrue(trigger_starts_planner(
            ReplanTrigger.ABORT, HarvestTaskPhase.MOVING_TO_GRASP
        ))
        self.assertTrue(trigger_starts_planner(
            ReplanTrigger.ABORT, HarvestTaskPhase.DETACHING
        ))

    def test_tracking_error_does_not_start_global_planner(self) -> None:
        for phase in (
            HarvestTaskPhase.MOVING_TO_PREGRASP,
            HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.MOVING_TO_PLACE,
        ):
            with self.subTest(phase=phase):
                self.assertFalse(trigger_starts_planner(
                    ReplanTrigger.TRACKING_ERROR, phase
                ))

    def test_contact_dominant_detaching_stays_observe_only(self) -> None:
        self.assertFalse(trigger_starts_planner(
            ReplanTrigger.TRACKING_ERROR, HarvestTaskPhase.DETACHING
        ))

    def test_timer_and_scene_change_stay_observe_only(self) -> None:
        for phase in (
            HarvestTaskPhase.MOVING_TO_PREGRASP,
            HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.MOVING_TO_PLACE,
        ):
            with self.subTest(phase=phase):
                self.assertFalse(trigger_starts_planner(ReplanTrigger.TIMER, phase))
                self.assertFalse(trigger_starts_planner(
                    ReplanTrigger.SCENE_CHANGE, phase
                ))

    def test_timer_does_not_trigger_in_step7(self) -> None:
        decision = evaluate_replan_trigger(
            state=_ready_state(), memory=TriggerMemory(), now_sec=10.0
        )
        self.assertFalse(decision.triggered)

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
