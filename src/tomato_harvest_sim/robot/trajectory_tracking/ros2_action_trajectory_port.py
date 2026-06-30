"""
TrajectoryExecutionPort implementation that sends FollowJointTrajectory
goals to the C++ JointTrajectoryController via ROS2 action.

This replaces the in-process JointTrajectoryControllerBridge when running
with the external franka_ros2_control package.
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

import numpy as np

from tomato_harvest_sim.api.trajectory_execution import (
    TrajectoryExecutionFeedback,
    TrajectoryExecutionPort,
    TrajectoryExecutionRequest,
    TrajectoryExecutionResult,
    TrajectoryExecutionState,
)

if TYPE_CHECKING:
    pass

_FOLLOW_JT_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"



class Ros2ActionTrajectoryPort:
    """
    Implements TrajectoryExecutionPort over a real ROS2 FollowJointTrajectory action.

    step() must be called once per control tick to pump the ROS2 executor and
    update feedback / result state.
    """

    def __init__(
        self,
        *,
        action_name: str = _FOLLOW_JT_ACTION,
        controller_name: str = "joint_trajectory_controller",
        spin_timeout_sec: float = 0.001,
        server_wait_timeout_sec: float = 10.0,
    ) -> None:
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from rclpy.node import Node

        self._rclpy = rclpy
        self._action_name = action_name
        self._controller_name = controller_name
        self._spin_timeout_sec = spin_timeout_sec
        self._FollowJointTrajectory = FollowJointTrajectory

        self._initialized_here = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._initialized_here = True

        self._node: Node = rclpy.create_node("ros2_action_trajectory_port")
        self._action_client: ActionClient = ActionClient(
            self._node, FollowJointTrajectory, action_name
        )

        self._active_request: TrajectoryExecutionRequest | None = None
        self._feedback: TrajectoryExecutionFeedback | None = None
        self._result: TrajectoryExecutionResult | None = None

        self._goal_handle: object = None
        self._send_goal_future: object = None
        self._result_future: object = None
        self._server_ready = False
        self._server_wait_timeout_sec = server_wait_timeout_sec

    def send_goal(self, request: TrajectoryExecutionRequest) -> bool:
        self._result = None
        self._feedback = None
        self._goal_handle = None
        self._send_goal_future = None
        self._result_future = None

        if not self._ensure_server_ready():
            self._result = TrajectoryExecutionResult(
                controller_name=self._controller_name,
                state=TrajectoryExecutionState.REJECTED,
                message="action_server_unavailable",
                timestamp_sec=time.monotonic(),
            )
            return False

        goal = self._build_goal(request)
        self._send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,
        )
        self._send_goal_future.add_done_callback(self._on_goal_response)
        self._active_request = request
        self._feedback = TrajectoryExecutionFeedback(
            controller_name=self._controller_name,
            state=TrajectoryExecutionState.ACCEPTED,
            desired_positions_rad=(),
            actual_positions_rad=(),
            desired_velocities_rad_s=(),
            actual_velocities_rad_s=(),
            error_norm_rad=0.0,
            timestamp_sec=time.monotonic(),
        )
        return True

    def cancel_goal(self) -> None:
        if self._goal_handle is not None:
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _: None)
        self._result = TrajectoryExecutionResult(
            controller_name=self._controller_name,
            state=TrajectoryExecutionState.CANCELED,
            message="goal_canceled",
            timestamp_sec=time.monotonic(),
        )
        self._active_request = None
        self._goal_handle = None
        self._send_goal_future = None
        self._result_future = None

    def step(self) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=self._spin_timeout_sec)

    def active_request(self) -> TrajectoryExecutionRequest | None:
        return self._active_request

    def current_feedback(self) -> TrajectoryExecutionFeedback | None:
        return self._feedback

    def current_result(self) -> TrajectoryExecutionResult | None:
        return self._result

    def close(self) -> None:
        self._node.destroy_node()
        if self._initialized_here and self._rclpy.ok():
            self._rclpy.shutdown()

    def _ensure_server_ready(self) -> bool:
        if self._server_ready:
            return True
        self._rclpy.spin_once(self._node, timeout_sec=0.01)
        ready = self._action_client.wait_for_server(
            timeout_sec=self._server_wait_timeout_sec
        )
        if ready:
            self._server_ready = True
        return ready

    def _build_goal(self, request: TrajectoryExecutionRequest) -> object:
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        goal = self._FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(request.trajectory.joint_names)

        for point in request.trajectory.points:
            ros_point = JointTrajectoryPoint()
            ros_point.positions = [float(v) for v in point.positions_rad]
            ros_point.velocities = [float(v) for v in (point.velocities_rad_s or [])]
            total_sec = point.time_from_start_sec
            dur = Duration()
            dur.sec = int(total_sec)
            dur.nanosec = int((total_sec - dur.sec) * 1_000_000_000)
            ros_point.time_from_start = dur
            traj.points.append(ros_point)

        goal.trajectory = traj
        return goal

    def _on_goal_response(self, future: object) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._result = TrajectoryExecutionResult(
                controller_name=self._controller_name,
                state=TrajectoryExecutionState.REJECTED,
                message="goal_rejected_by_server",
                timestamp_sec=time.monotonic(),
            )
            self._active_request = None
            return
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg: object) -> None:
        if self._active_request is None:
            return
        fb = getattr(feedback_msg, "feedback", feedback_msg)
        desired = getattr(fb, "desired", None)
        actual = getattr(fb, "actual", None)
        error = getattr(fb, "error", None)

        def _positions(pt: object) -> tuple[float, ...]:
            if pt is None:
                return ()
            return tuple(float(v) for v in getattr(pt, "positions", ()))

        def _velocities(pt: object) -> tuple[float, ...]:
            if pt is None:
                return ()
            return tuple(float(v) for v in getattr(pt, "velocities", ()))

        desired_pos = _positions(desired)
        actual_pos = _positions(actual)
        error_pos = _positions(error)
        error_norm = float(np.max(np.abs(np.asarray(error_pos, dtype=float)))) if error_pos else 0.0

        self._feedback = TrajectoryExecutionFeedback(
            controller_name=self._controller_name,
            state=TrajectoryExecutionState.ACTIVE,
            desired_positions_rad=desired_pos,
            actual_positions_rad=actual_pos,
            desired_velocities_rad_s=_velocities(desired),
            actual_velocities_rad_s=_velocities(actual),
            error_norm_rad=error_norm,
            timestamp_sec=time.monotonic(),
        )

    def _on_result(self, future: object) -> None:
        if self._active_request is None:
            return
        result_wrapper = future.result()
        error_code = 0
        error_string = ""
        if result_wrapper is not None:
            result = getattr(result_wrapper, "result", None)
            if result is not None:
                error_code = int(getattr(result, "error_code", 0))
                error_string = str(getattr(result, "error_string", ""))

        if error_code == 0:
            state = TrajectoryExecutionState.SUCCEEDED
            message = "goal_reached"
        else:
            state = TrajectoryExecutionState.ABORTED
            message = error_string or f"error_code_{error_code}"

        self._result = TrajectoryExecutionResult(
            controller_name=self._controller_name,
            state=state,
            message=message,
            timestamp_sec=time.monotonic(),
        )
        self._active_request = None
        self._goal_handle = None
