from __future__ import annotations

from tomato_harvest_sim.api.contracts import HarvestTaskPhase, PhaseExecutionIntent, PhaseId
from tomato_harvest_sim.robot.behavior_planner.intent_builder import PhaseExecutionIntentBuilder


class BehaviorPlanner:
    _TASK_PHASE_TO_EXECUTION_PHASE = {
        HarvestTaskPhase.MOVING_TO_PREGRASP: PhaseId.MOVING_TO_PREGRASP,
        HarvestTaskPhase.MOVING_TO_GRASP: PhaseId.MOVING_TO_GRASP,
        HarvestTaskPhase.DETACHING: PhaseId.PULL_TO_DETACH,
        HarvestTaskPhase.MOVING_TO_PLACE: PhaseId.MOVING_TO_PLACE,
        HarvestTaskPhase.RETURNING_HOME: PhaseId.RETURNING_HOME,
    }

    def __init__(self, *, intent_builder: PhaseExecutionIntentBuilder | None = None) -> None:
        self._intent_builder = intent_builder or PhaseExecutionIntentBuilder()

    def intent_for_task_phase(self, task_phase: HarvestTaskPhase) -> PhaseExecutionIntent | None:
        phase_id = self._TASK_PHASE_TO_EXECUTION_PHASE.get(task_phase)
        if phase_id is None:
            return None
        return self._intent_builder.build(phase_id)
