from __future__ import annotations

import unittest

from tomato_harvest_sim.msg.contracts import HarvestTaskPhase, JointStateSnapshot
from tomato_harvest_sim.robot.motion_planner.state_aggregation import PlannerStateAggregator


class PlannerStateAggregatorTest(unittest.TestCase):
    def test_latest_values_are_exposed_as_one_snapshot(self) -> None:
        aggregator = PlannerStateAggregator()
        joints = JointStateSnapshot(("joint1",), (0.25,))
        aggregator.update_phase(HarvestTaskPhase.MOVING_TO_GRASP)
        aggregator.update_joint_state(joints)
        aggregator.update_tracking_error(0.12)

        state = aggregator.snapshot()
        self.assertEqual(state.phase, HarvestTaskPhase.MOVING_TO_GRASP)
        self.assertEqual(state.joint_state, joints)
        self.assertEqual(state.tracking_error_rad, 0.12)

    def test_abort_is_recorded_as_a_monotonic_event(self) -> None:
        aggregator = PlannerStateAggregator()
        aggregator.observe_abort()
        aggregator.observe_abort()
        self.assertEqual(aggregator.snapshot().abort_generation, 2)
