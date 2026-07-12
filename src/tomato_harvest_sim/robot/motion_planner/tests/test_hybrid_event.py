from __future__ import annotations

import unittest

from tomato_harvest_sim.msg.contracts import HarvestTaskPhase
from tomato_harvest_sim.robot.motion_planner.hybrid_event import (
    LocalEventMemory, PlannerRoute, admit_local_event, route_event,
)
from tomato_harvest_sim.robot.motion_planner.replan_trigger import ReplanTrigger


class HybridEventTest(unittest.TestCase):
    def test_tracking_error_is_local_and_abort_is_global(self) -> None:
        phase = HarvestTaskPhase.MOVING_TO_GRASP
        self.assertEqual(route_event(ReplanTrigger.TRACKING_ERROR, phase), PlannerRoute.LOCAL)
        self.assertEqual(route_event(ReplanTrigger.ABORT, phase), PlannerRoute.GLOBAL)

    def test_timer_scene_and_contact_tracking_are_observe_only(self) -> None:
        self.assertEqual(route_event(ReplanTrigger.TIMER, HarvestTaskPhase.MOVING_TO_PLACE), PlannerRoute.OBSERVE)
        self.assertEqual(route_event(ReplanTrigger.SCENE_CHANGE, HarvestTaskPhase.MOVING_TO_PLACE), PlannerRoute.OBSERVE)
        self.assertEqual(route_event(ReplanTrigger.TRACKING_ERROR, HarvestTaskPhase.DETACHING), PlannerRoute.OBSERVE)

    def test_local_event_admission_rejects_stale_duplicate_and_rate(self) -> None:
        phase = HarvestTaskPhase.MOVING_TO_PLACE
        self.assertTrue(admit_local_event(event_id="a", event_at_sec=9.9, now_sec=10.0, phase=phase, memory=LocalEventMemory()).accepted)
        memory = LocalEventMemory(last_accepted_at_sec=10.0, last_event_id="a")
        self.assertEqual(admit_local_event(event_id="a", event_at_sec=10.0, now_sec=10.1, phase=phase, memory=memory).reason, "duplicate_event")
        self.assertEqual(admit_local_event(event_id="b", event_at_sec=10.0, now_sec=10.1, phase=phase, memory=memory).reason, "rate_limited")
        self.assertEqual(admit_local_event(event_id="c", event_at_sec=7.0, now_sec=10.0, phase=phase, memory=LocalEventMemory()).reason, "stale_event")


if __name__ == "__main__":
    unittest.main()
