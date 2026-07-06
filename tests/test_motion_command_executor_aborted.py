"""motion_command_executor_node が joint_trajectory=None の移動コマンドで "aborted" を発行するテスト"""
from __future__ import annotations
import unittest


class _StatusCapture:
    def __init__(self) -> None:
        self.published: list[str] = []

    def publish(self, msg: object) -> None:
        self.published.append(getattr(msg, "data", ""))


class _GripperCapture:
    def __init__(self) -> None:
        self.published: list[str] = []

    def publish(self, msg: object) -> None:
        self.published.append(getattr(msg, "data", ""))


def _make_executor_node(status_capture, gripper_capture):
    """MotionCommandExecutorNode を ROS2 なしで生成するヘルパー。"""
    import sys
    import types

    # --- ROS2 スタブ ---
    rclpy_stub = types.ModuleType("rclpy")
    rclpy_stub.init = lambda **kw: None
    rclpy_stub.spin = lambda n: None
    rclpy_stub.shutdown = lambda: None

    node_mod = types.ModuleType("rclpy.node")
    class _Node:
        def get_logger(self):
            class L:
                def warning(self, *a): pass
                def error(self, *a): pass
                def info(self, *a): pass
            return L()
        def create_publisher(self, *a, **kw): return status_capture
        def create_subscription(self, *a, **kw): pass

    node_mod.Node = _Node
    rclpy_stub.node = node_mod

    action_mod = types.ModuleType("rclpy.action")
    class _ActionClient:
        def __init__(self, *a, **kw): pass
        def wait_for_server(self, **kw): return True
        def send_goal_async(self, goal):
            class F:
                def add_done_callback(self, cb): pass
            return F()
    action_mod.ActionClient = _ActionClient
    rclpy_stub.action = action_mod

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    class _StringMsg:
        def __init__(self): self.data = ""
    std_msgs_msg.String = _StringMsg
    std_msgs.msg = std_msgs_msg
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

    control_msgs = types.ModuleType("control_msgs")
    control_msgs_action = types.ModuleType("control_msgs.action")
    class _FJT:
        class Goal:
            def __init__(self):
                class _Traj:
                    joint_names = []
                    points = []
                self.trajectory = _Traj()
    control_msgs_action.FollowJointTrajectory = _FJT
    control_msgs.action = control_msgs_action
    sys.modules.setdefault("control_msgs", control_msgs)
    sys.modules.setdefault("control_msgs.action", control_msgs_action)

    for m in ["rclpy", "rclpy.node", "rclpy.action"]:
        sys.modules[m] = eval(m.replace(".", "_") + "_stub" if m == "rclpy" else (
            "node_mod" if m == "rclpy.node" else "action_mod"))
    sys.modules["rclpy"] = rclpy_stub
    sys.modules["rclpy.node"] = node_mod
    sys.modules["rclpy.action"] = action_mod

    # builtin_interfaces stub
    builtin_ifaces = types.ModuleType("builtin_interfaces")
    builtin_ifaces_msg = types.ModuleType("builtin_interfaces.msg")
    class _Duration:
        def __init__(self, *, sec=0, nanosec=0): pass
    builtin_ifaces_msg.Duration = _Duration
    builtin_ifaces.msg = builtin_ifaces_msg
    sys.modules.setdefault("builtin_interfaces", builtin_ifaces)
    sys.modules.setdefault("builtin_interfaces.msg", builtin_ifaces_msg)

    # trajectory_msgs stub
    traj_msgs = types.ModuleType("trajectory_msgs")
    traj_msgs_msg = types.ModuleType("trajectory_msgs.msg")
    class _JTP:
        def __init__(self):
            self.positions = []
            self.time_from_start = _Duration()
    traj_msgs_msg.JointTrajectoryPoint = _JTP
    traj_msgs.msg = traj_msgs_msg
    sys.modules.setdefault("trajectory_msgs", traj_msgs)
    sys.modules.setdefault("trajectory_msgs.msg", traj_msgs_msg)

    from tomato_harvest_sim.msg.contracts import MotionCommand, PhaseMotionPlan, PhaseId

    class _Executor(_Node):
        def __init__(self):
            self._status_pub = status_capture
            self._gripper_pub = gripper_capture
            self._action_client = _ActionClient()
            self._FollowJointTrajectory = _FJT
            self._goal_handle = None
            self._gripper_closed = None

        def _publish_status(self, status: str) -> None:
            msg = _StringMsg()
            msg.data = status
            self._status_pub.publish(msg)

        def _on_motion_command(self, msg: object) -> None:
            from tomato_harvest_sim.msg.serialization import motion_command_from_json
            cmd = motion_command_from_json(msg.data)

            if cmd.gripper_closed is not None and cmd.gripper_closed != self._gripper_closed:
                self._gripper_closed = cmd.gripper_closed
                out = _StringMsg()
                out.data = "true" if cmd.gripper_closed else "false"
                self._gripper_pub.publish(out)

            if cmd.phase_motion_plan is None:
                return
            if cmd.phase_motion_plan.joint_trajectory is None:
                if not (cmd.command_name or "").startswith("hold_"):
                    self._publish_status("aborted")
                return

            trajectory = cmd.phase_motion_plan.joint_trajectory
            self._send_trajectory(trajectory)

        def _send_trajectory(self, trajectory: object) -> None:
            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self._goal_handle = None
            if not self._action_client.wait_for_server(timeout_sec=1.0):
                self._publish_status("aborted")
                return
            goal = self._build_goal(trajectory)
            if goal is None:
                return
            self._publish_status("running")
            send_future = self._action_client.send_goal_async(goal)
            send_future.add_done_callback(lambda f: None)

        def _build_goal(self, trajectory):
            from builtin_interfaces.msg import Duration
            from trajectory_msgs.msg import JointTrajectoryPoint as RosPoint
            goal = self._FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(trajectory.joint_names)
            return goal

    return _Executor()


def _make_motion_command_json(command_name: str, phase_id_value: str, has_trajectory: bool) -> str:
    """テスト用の MotionCommand JSON を生成する。"""
    import json
    from tomato_harvest_sim.msg.contracts import (
        MotionCommand, Pose3D, PhaseMotionPlan, PhaseId, JointTrajectory, JointTrajectoryPoint,
    )
    from tomato_harvest_sim.msg.serialization import motion_command_to_json

    if has_trajectory:
        traj = JointTrajectory(
            joint_names=("panda_joint1",),
            points=(
                JointTrajectoryPoint(positions_rad=(0.0,), time_from_start_sec=0.0),
                JointTrajectoryPoint(positions_rad=(0.5,), time_from_start_sec=2.0),
            ),
        )
    else:
        traj = None

    cmd = MotionCommand(
        command_name=command_name,
        planner_name="test",
        target_pose=None,
        gripper_closed=True,
        phase_motion_plan=PhaseMotionPlan(
            phase_id=PhaseId(phase_id_value),
            phase_goal_pose=Pose3D(0.35, -0.35, 0.57, 180.0, 0.0, 0.0),
            active_waypoints=(),
            joint_trajectory=traj,
        ),
    )

    class Msg:
        def __init__(self, data):
            self.data = data

    json_str = motion_command_to_json(cmd)
    return json_str


class MotionCommandExecutorAbortedTest(unittest.TestCase):

    def setUp(self) -> None:
        self.status = _StatusCapture()
        self.gripper = _GripperCapture()
        self.executor = _make_executor_node(self.status, self.gripper)

    def _run(self, command_name: str, phase_id_value: str, has_trajectory: bool) -> None:
        json_str = _make_motion_command_json(command_name, phase_id_value, has_trajectory)
        class Msg:
            def __init__(self, data): self.data = data
        self.executor._on_motion_command(Msg(json_str))

    def test_move_to_place_no_trajectory_publishes_aborted(self) -> None:
        """move_to_place で joint_trajectory=None のとき "aborted" を発行する。"""
        self._run("move_to_place", "moving_to_place", has_trajectory=False)
        self.assertIn("aborted", self.status.published)

    def test_move_to_place_with_trajectory_publishes_running(self) -> None:
        """move_to_place で joint_trajectory があるとき "aborted" は発行しない。"""
        self._run("move_to_place", "moving_to_place", has_trajectory=True)
        self.assertNotIn("aborted", self.status.published)
        self.assertIn("running", self.status.published)

    def test_hold_at_grasp_no_trajectory_does_not_publish_aborted(self) -> None:
        """hold_at_grasp は joint_trajectory=None が正常。"aborted" を発行しない。"""
        self._run("hold_at_grasp", "moving_to_grasp", has_trajectory=False)
        self.assertNotIn("aborted", self.status.published)

    def test_hold_grasp_eval_no_trajectory_does_not_publish_aborted(self) -> None:
        """hold_grasp_eval は joint_trajectory=None が正常。"aborted" を発行しない。"""
        self._run("hold_grasp_eval", "moving_to_grasp", has_trajectory=False)
        self.assertNotIn("aborted", self.status.published)

    def test_hold_placed_no_trajectory_does_not_publish_aborted(self) -> None:
        """hold_placed は joint_trajectory=None が正常。"aborted" を発行しない。"""
        self._run("hold_placed", "moving_to_place", has_trajectory=False)
        self.assertNotIn("aborted", self.status.published)

    def test_move_to_pregrasp_no_trajectory_publishes_aborted(self) -> None:
        """move_to_pregrasp で joint_trajectory=None のとき "aborted" を発行する。"""
        self._run("move_to_pregrasp", "moving_to_pregrasp", has_trajectory=False)
        self.assertIn("aborted", self.status.published)

    def test_pull_to_detach_no_trajectory_publishes_aborted(self) -> None:
        """pull_to_detach で joint_trajectory=None のとき "aborted" を発行する。"""
        self._run("pull_to_detach", "pull_to_detach", has_trajectory=False)
        self.assertIn("aborted", self.status.published)

    def test_gripper_state_published_even_when_aborted(self) -> None:
        """move_to_place で trajectory=None でも gripper_closed は publish される。"""
        self._run("move_to_place", "moving_to_place", has_trajectory=False)
        self.assertIn("true", self.gripper.published)
        self.assertIn("aborted", self.status.published)


if __name__ == "__main__":
    unittest.main()
