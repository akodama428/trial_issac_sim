from __future__ import annotations

from tomato_harvest_sim.api.trajectory_execution import (
    TrajectoryExecutionFeedback,
    TrajectoryExecutionPort,
    TrajectoryExecutionRequest,
    TrajectoryExecutionResult,
)


class FollowJointTrajectoryActionClient:
    def __init__(self, *, port: TrajectoryExecutionPort) -> None:
        self._port = port

    def send_goal(self, request: TrajectoryExecutionRequest) -> bool:
        return self._port.send_goal(request)

    def cancel_goal(self) -> None:
        self._port.cancel_goal()

    def step(self) -> None:
        self._port.step()

    def active_request(self) -> TrajectoryExecutionRequest | None:
        return self._port.active_request()

    def current_feedback(self) -> TrajectoryExecutionFeedback | None:
        return self._port.current_feedback()

    def current_result(self) -> TrajectoryExecutionResult | None:
        return self._port.current_result()

    def active_segment_index(self) -> int | None:
        return getattr(self._port, "active_segment_index", None)

    def current_controller_state(self) -> object | None:
        fn = getattr(self._port, "current_controller_state", None)
        return fn() if callable(fn) else None

    def update_external_command_state(self, **kwargs: object) -> None:
        fn = getattr(self._port, "update_external_command_state", None)
        if callable(fn):
            fn(**kwargs)
