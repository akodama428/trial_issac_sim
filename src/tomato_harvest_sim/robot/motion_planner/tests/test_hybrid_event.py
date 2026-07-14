from __future__ import annotations

import unittest

from tomato_harvest_sim.msg.contracts import HarvestTaskPhase
from tomato_harvest_sim.robot.motion_planner.hybrid_event import PlannerRoute, route_event
from tomato_harvest_sim.robot.motion_planner.replan_trigger import ReplanTrigger


class HybridEventTest(unittest.TestCase):
    def test_abort_is_global_and_tracking_error_is_observed_by_servo(self) -> None:
        for phase in (
            HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.RETURNING_HOME,
            HarvestTaskPhase.DETACHING,
        ):
            self.assertEqual(route_event(ReplanTrigger.TRACKING_ERROR, phase), PlannerRoute.OBSERVE)
            self.assertEqual(route_event(ReplanTrigger.ABORT, phase), PlannerRoute.GLOBAL)

    def test_timer_and_scene_change_are_observe_only(self) -> None:
        phase = HarvestTaskPhase.MOVING_TO_PLACE
        self.assertEqual(route_event(ReplanTrigger.TIMER, phase), PlannerRoute.OBSERVE)
        self.assertEqual(route_event(ReplanTrigger.SCENE_CHANGE, phase), PlannerRoute.OBSERVE)


if __name__ == "__main__":
    unittest.main()
