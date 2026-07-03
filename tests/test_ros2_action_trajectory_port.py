"""Tests for Ros2ActionTrajectoryPort state machine logic."""
from __future__ import annotations

import time
import unittest
from dataclasses import dataclass, field
from typing import Any
from unittest.mock import MagicMock

from tomato_harvest_sim.api.contracts import JointTrajectory, JointTrajectoryPoint
from tomato_harvest_sim.api.trajectory_execution import (
    TrajectoryExecutionRequest,
    TrajectoryExecutionResult,
    TrajectoryExecutionState,
)


def _make_trajectory(n_points: int = 2) -> JointTrajectory:
    joint_names = ("panda_joint1", "panda_joint2", "panda_joint3")
    points = tuple(
        JointTrajectoryPoint(
            positions_rad=tuple(0.1 * i * (j + 1) for j in range(3)),
            time_from_start_sec=float(i + 1),
        )
        for i in range(n_points)
    )
    return JointTrajectory(joint_names=joint_names, points=points)


def _make_request(trajectory: JointTrajectory | None = None) -> TrajectoryExecutionRequest:
    return TrajectoryExecutionRequest(
        controller_name="joint_trajectory_controller",
        command_name="test_move",
        planner_name="test_planner",
        trajectory=trajectory or _make_trajectory(),
    )


@dataclass
class _FakeFuture:
    _value: Any = None
    _done: bool = False
    _callbacks: list = field(default_factory=list)

    def result(self) -> Any:
        return self._value

    def done(self) -> bool:
        return self._done

    def add_done_callback(self, cb: Any) -> None:
        self._callbacks.append(cb)

    def resolve(self, value: Any) -> None:
        self._value = value
        self._done = True
        for cb in self._callbacks:
            cb(self)


@dataclass
class _FakeGoalHandle:
    accepted: bool = True
    _result_future: _FakeFuture = field(default_factory=_FakeFuture)
    _cancel_called: bool = False

    def get_result_async(self) -> _FakeFuture:
        return self._result_future

    def cancel_goal_async(self) -> _FakeFuture:
        self._cancel_called = True
        return _FakeFuture()


@dataclass
class _FakeResultWrapper:
    error_code: int = 0
    error_string: str = ""

    def __post_init__(self) -> None:
        self.result = _FakeInnerResult(self.error_code, self.error_string)


@dataclass
class _FakeInnerResult:
    error_code: int
    error_string: str


def _make_port() -> "object":
    """Build a Ros2ActionTrajectoryPort with all ROS2 deps mocked out."""
    from tomato_harvest_sim.robot.trajectory_tracking.ros2_action_trajectory_port import (
        Ros2ActionTrajectoryPort,
    )

    mock_rclpy = MagicMock()
    mock_rclpy.ok.return_value = True
    mock_rclpy.spin_once.return_value = None

    mock_action_client = MagicMock()

    port = Ros2ActionTrajectoryPort.__new__(Ros2ActionTrajectoryPort)
    port._rclpy = mock_rclpy
    port._action_name = "/joint_trajectory_controller/follow_joint_trajectory"
    port._controller_name = "joint_trajectory_controller"
    port._spin_timeout_sec = 0.001
    port._server_wait_timeout_sec = 1.0
    port._FollowJointTrajectory = MagicMock()
    port._FollowJointTrajectory.Goal.return_value = MagicMock()
    port._node = MagicMock()
    port._action_client = mock_action_client
    port._active_request = None
    port._feedback = None
    port._result = None
    port._goal_handle = None
    port._send_goal_future = None
    port._result_future = None
    port._server_ready = True
    port._initialized_here = False
    port._executor = MagicMock()

    # Stub _build_goal to avoid importing builtin_interfaces (not available outside ROS).
    port._build_goal = lambda request: MagicMock()
    return port


class TestRos2ActionTrajectoryPortInterface(unittest.TestCase):

    def setUp(self) -> None:
        self._port = _make_port()
        self._action_client = self._port._action_client

    def test_initial_state_is_idle(self) -> None:
        self.assertIsNone(self._port.active_request())
        self.assertIsNone(self._port.current_feedback())
        self.assertIsNone(self._port.current_result())

    def test_send_goal_sets_active_request(self) -> None:
        send_goal_future = _FakeFuture()
        self._action_client.send_goal_async.return_value = send_goal_future

        request = _make_request()
        accepted = self._port.send_goal(request)

        self.assertTrue(accepted)
        self.assertIs(self._port.active_request(), request)
        self.assertIsNotNone(self._port.current_feedback())
        self.assertEqual(
            self._port.current_feedback().state, TrajectoryExecutionState.ACCEPTED
        )

    def test_goal_response_rejection_sets_result(self) -> None:
        send_goal_future = _FakeFuture()
        self._action_client.send_goal_async.return_value = send_goal_future

        request = _make_request()
        self._port.send_goal(request)

        rejected_handle = _FakeGoalHandle(accepted=False)
        send_goal_future.resolve(rejected_handle)

        self.assertIsNone(self._port.active_request())
        result = self._port.current_result()
        self.assertIsNotNone(result)
        self.assertEqual(result.state, TrajectoryExecutionState.REJECTED)

    def test_goal_success_sets_result_succeeded(self) -> None:
        send_goal_future = _FakeFuture()
        self._action_client.send_goal_async.return_value = send_goal_future

        request = _make_request()
        self._port.send_goal(request)

        goal_handle = _FakeGoalHandle(accepted=True)
        send_goal_future.resolve(goal_handle)

        result_wrapper = _FakeResultWrapper(error_code=0)
        goal_handle._result_future.resolve(result_wrapper)

        result = self._port.current_result()
        self.assertIsNotNone(result)
        self.assertEqual(result.state, TrajectoryExecutionState.SUCCEEDED)
        self.assertEqual(result.message, "goal_reached")
        self.assertIsNone(self._port.active_request())

    def test_goal_failure_sets_result_aborted(self) -> None:
        send_goal_future = _FakeFuture()
        self._action_client.send_goal_async.return_value = send_goal_future

        request = _make_request()
        self._port.send_goal(request)

        goal_handle = _FakeGoalHandle(accepted=True)
        send_goal_future.resolve(goal_handle)

        result_wrapper = _FakeResultWrapper(error_code=-1, error_string="path_tolerance_violated")
        goal_handle._result_future.resolve(result_wrapper)

        result = self._port.current_result()
        self.assertIsNotNone(result)
        self.assertEqual(result.state, TrajectoryExecutionState.ABORTED)
        self.assertEqual(result.message, "path_tolerance_violated")

    def test_feedback_callback_updates_state_to_active(self) -> None:
        send_goal_future = _FakeFuture()
        self._action_client.send_goal_async.return_value = send_goal_future

        request = _make_request()
        self._port.send_goal(request)

        goal_handle = _FakeGoalHandle(accepted=True)
        send_goal_future.resolve(goal_handle)

        feedback_msg = MagicMock()
        feedback_msg.feedback.desired.positions = [0.1, 0.2, 0.3]
        feedback_msg.feedback.desired.velocities = [0.01, 0.02, 0.03]
        feedback_msg.feedback.actual.positions = [0.09, 0.19, 0.29]
        feedback_msg.feedback.actual.velocities = [0.0, 0.0, 0.0]
        feedback_msg.feedback.error.positions = [0.01, 0.01, 0.01]
        feedback_msg.feedback.error.velocities = []

        self._port._on_feedback(feedback_msg)

        fb = self._port.current_feedback()
        self.assertIsNotNone(fb)
        self.assertEqual(fb.state, TrajectoryExecutionState.ACTIVE)
        self.assertAlmostEqual(fb.error_norm_rad, 0.01, places=5)

    def test_cancel_goal_clears_active_request(self) -> None:
        send_goal_future = _FakeFuture()
        self._action_client.send_goal_async.return_value = send_goal_future

        request = _make_request()
        self._port.send_goal(request)

        goal_handle = _FakeGoalHandle(accepted=True)
        send_goal_future.resolve(goal_handle)
        self._port._goal_handle = goal_handle

        self._port.cancel_goal()

        self.assertIsNone(self._port.active_request())
        result = self._port.current_result()
        self.assertIsNotNone(result)
        self.assertEqual(result.state, TrajectoryExecutionState.CANCELED)
        self.assertTrue(goal_handle._cancel_called)

    def test_server_unavailable_returns_rejected(self) -> None:
        self._port._server_ready = False
        self._action_client.wait_for_server.return_value = False

        request = _make_request()
        accepted = self._port.send_goal(request)

        self.assertFalse(accepted)
        result = self._port.current_result()
        self.assertIsNotNone(result)
        self.assertEqual(result.state, TrajectoryExecutionState.REJECTED)

    def test_send_goal_clears_previous_result(self) -> None:
        self._port._result = TrajectoryExecutionResult(
            controller_name="joint_trajectory_controller",
            state=TrajectoryExecutionState.ABORTED,
            message="old_result",
            timestamp_sec=time.monotonic(),
        )

        send_goal_future = _FakeFuture()
        self._action_client.send_goal_async.return_value = send_goal_future

        self._port.send_goal(_make_request())

        self.assertIsNone(self._port.current_result())

    def test_feedback_ignored_after_goal_complete(self) -> None:
        # active_request is already None (goal completed before feedback arrives).
        self._port._active_request = None

        feedback_msg = MagicMock()
        feedback_msg.feedback.desired.positions = [0.1]
        feedback_msg.feedback.desired.velocities = []
        feedback_msg.feedback.actual.positions = [0.1]
        feedback_msg.feedback.actual.velocities = []
        feedback_msg.feedback.error.positions = [0.0]
        feedback_msg.feedback.error.velocities = []

        self._port._on_feedback(feedback_msg)
        self.assertIsNone(self._port.current_feedback())


class TestRos2ActionTrajectoryPortProtocolCompliance(unittest.TestCase):
    """Verify that the port satisfies the TrajectoryExecutionPort Protocol."""

    def test_has_all_protocol_methods(self) -> None:
        from tomato_harvest_sim.robot.trajectory_tracking.ros2_action_trajectory_port import (
            Ros2ActionTrajectoryPort,
        )
        from tomato_harvest_sim.api.trajectory_execution import TrajectoryExecutionPort
        import inspect

        for method_name in ("send_goal", "cancel_goal", "step", "active_request",
                            "current_feedback", "current_result"):
            self.assertTrue(
                hasattr(Ros2ActionTrajectoryPort, method_name),
                f"Missing method: {method_name}",
            )

        port = _make_port()
        for method_name in ("send_goal", "cancel_goal", "step", "active_request",
                            "current_feedback", "current_result"):
            self.assertTrue(callable(getattr(port, method_name)))


if __name__ == "__main__":
    unittest.main()
