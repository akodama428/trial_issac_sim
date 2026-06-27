from __future__ import annotations

from tomato_harvest_sim.api.contracts import JointTrajectory
from tomato_harvest_sim.api.trajectory_execution import (
    TrajectoryExecutionFeedback,
    TrajectoryExecutionResult,
    TrajectoryExecutionState,
)


class ExecutionMonitor:
    def __init__(self) -> None:
        self._last_trajectory: JointTrajectory | None = None
        self._accepted_announced = False
        self._completed_announced = False
        self._aborted_announced = False

    def reset_for_trajectory(self, trajectory: JointTrajectory) -> None:
        if trajectory == self._last_trajectory:
            return
        self._last_trajectory = trajectory
        self._accepted_announced = False
        self._completed_announced = False
        self._aborted_announced = False

    def acceptance_log(self, feedback: TrajectoryExecutionFeedback | None) -> str | None:
        if feedback is None or feedback.state not in {TrajectoryExecutionState.ACCEPTED, TrajectoryExecutionState.ACTIVE}:
            return None
        if self._accepted_announced:
            return None
        self._accepted_announced = True
        return "[Simulator] accepted joint trajectory goal via FollowJointTrajectory."

    def result_update(self, result: TrajectoryExecutionResult | None) -> tuple[str | None, str | None]:
        if result is None:
            return None, None
        if result.state is TrajectoryExecutionState.SUCCEEDED:
            if self._completed_announced:
                return None, None
            self._completed_announced = True
            return "[Simulator] Franka trajectory completed.", None
        if result.state in {TrajectoryExecutionState.ABORTED, TrajectoryExecutionState.REJECTED}:
            if self._aborted_announced:
                return None, None
            self._aborted_announced = True
            reason = result.message or result.state.value
            return (
                f"[Simulator] MoveIt2 joint trajectory aborted; waiting for replanned motion command. reason={reason}",
                reason,
            )
        return None, None
