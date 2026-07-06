from __future__ import annotations

import unittest

from tomato_harvest_sim.msg.contracts import HarvestTaskPhase, PhaseId, SuccessJudge
from tomato_harvest_sim.robot.behavior_planner import BehaviorPlanner, PhaseExecutionIntentBuilder


class BehaviorPlannerTest(unittest.TestCase):
    def test_intent_builder_loads_phase_policy_from_yaml(self) -> None:
        builder = PhaseExecutionIntentBuilder()

        intent = builder.build(PhaseId.MOVING_TO_GRASP)

        self.assertEqual(intent.phase_id, PhaseId.MOVING_TO_GRASP)
        self.assertEqual(intent.success.judge, SuccessJudge.END_EFFECTOR_POSE)
        self.assertEqual(intent.success.position_tolerance_m, 0.010)
        self.assertEqual(intent.success.stable_steps, 2)
        self.assertEqual(intent.abort.stall_timeout_sec, 0.5)

    def test_behavior_planner_maps_runtime_phase_to_execution_phase(self) -> None:
        planner = BehaviorPlanner()

        intent = planner.intent_for_task_phase(HarvestTaskPhase.DETACHING)

        self.assertIsNotNone(intent)
        self.assertEqual(intent.phase_id, PhaseId.PULL_TO_DETACH)


if __name__ == "__main__":
    unittest.main()
