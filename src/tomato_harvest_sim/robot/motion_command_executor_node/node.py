"""motion_command_executor_node — /motion_command を JointTrajectoryController action へ変換する。

受信した MotionCommand の joint_trajectory を
/joint_trajectory_controller/follow_joint_trajectory action で実行し、
実行結果を /tomato_harvest/execution_status (String: running/succeeded/aborted) で publish する。

また gripper_closed フィールドを /tomato_harvest/gripper_closed (String: "true"/"false") で publish し、
IsaacJointRos2Bridge が指の開閉を制御できるようにする。

アーキテクチャ仕様: docs/index.html §MotionCommandExecutor
"""
from __future__ import annotations


def main() -> None:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import String

    from tomato_harvest_sim.msg.topics import (
        EXECUTION_STATUS_TOPIC,
        MOTION_COMMAND_TOPIC,
    )
    from tomato_harvest_sim.msg.serialization import motion_command_from_json

    GRIPPER_CLOSED_TOPIC = "/tomato_harvest/gripper_closed"
    JTC_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"

    rclpy.init()

    class MotionCommandExecutorNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("motion_command_executor_node")

            self._status_pub = self.create_publisher(String, EXECUTION_STATUS_TOPIC, 10)
            self._gripper_pub = self.create_publisher(String, GRIPPER_CLOSED_TOPIC, 10)

            try:
                from control_msgs.action import FollowJointTrajectory
                self._FollowJointTrajectory = FollowJointTrajectory
                self._action_client: ActionClient | None = ActionClient(
                    self, FollowJointTrajectory, JTC_ACTION
                )
            except ImportError:
                self._action_client = None
                self.get_logger().warning(
                    "control_msgs not available — trajectory execution disabled"
                )

            self._goal_handle = None
            self._gripper_closed: bool | None = None

            self.create_subscription(String, MOTION_COMMAND_TOPIC, self._on_motion_command, 10)
            self._publish_status("idle")

        # ------------------------------------------------------------------
        def _on_motion_command(self, msg: String) -> None:
            try:
                cmd = motion_command_from_json(msg.data)
            except Exception as exc:
                self.get_logger().error(f"Failed to parse motion_command: {exc}")
                return

            # Gripper control (独立して publish)
            if cmd.gripper_closed is not None and cmd.gripper_closed != self._gripper_closed:
                self._gripper_closed = cmd.gripper_closed
                out = String()
                out.data = "true" if cmd.gripper_closed else "false"
                self._gripper_pub.publish(out)

            # Trajectory execution
            if cmd.phase_motion_plan is None:
                return
            if cmd.phase_motion_plan.joint_trajectory is None:
                # "hold_*" コマンドはグリッパー制御のみで軌道不要（AT_GRASP, GRASP_EVALUATION, PLACED）
                # "move_*" / "pull_*" コマンドは軌道が必須。None の場合は計画失敗→"aborted" で再計画を促す
                if not (cmd.command_name or "").startswith("hold_"):
                    self._publish_status("aborted")
                return

            trajectory = cmd.phase_motion_plan.joint_trajectory
            self._send_trajectory(trajectory)

        def _send_trajectory(self, trajectory: object) -> None:
            if self._action_client is None:
                return

            # 既存ゴールをキャンセル
            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self._goal_handle = None

            if not self._action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().warning("JTC action server not available")
                self._publish_status("aborted")
                return

            goal = self._build_goal(trajectory)
            if goal is None:
                return

            self._publish_status("running")
            send_future = self._action_client.send_goal_async(goal)
            send_future.add_done_callback(self._on_goal_accepted)

        def _build_goal(self, trajectory: object) -> object | None:
            try:
                from builtin_interfaces.msg import Duration
                from trajectory_msgs.msg import JointTrajectoryPoint as RosPoint
            except ImportError:
                return None

            goal = self._FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(trajectory.joint_names)
            for pt in trajectory.points:
                ros_pt = RosPoint()
                ros_pt.positions = [float(v) for v in pt.positions_rad]
                sec = int(pt.time_from_start_sec)
                nanosec = int((pt.time_from_start_sec - sec) * 1_000_000_000)
                ros_pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
                goal.trajectory.points.append(ros_pt)
            return goal

        def _on_goal_accepted(self, future: object) -> None:
            try:
                goal_handle = future.result()
            except Exception as exc:
                self.get_logger().error(f"Goal send failed: {exc}")
                self._publish_status("aborted")
                return

            if not goal_handle.accepted:
                self.get_logger().warning("JTC goal rejected")
                self._publish_status("aborted")
                return

            self._goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_result)

        def _on_result(self, future: object) -> None:
            self._goal_handle = None
            try:
                from action_msgs.msg import GoalStatus
                result_response = future.result()
                status = result_response.status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self._publish_status("succeeded")
                elif status == GoalStatus.STATUS_CANCELED:
                    pass  # 意図的なキャンセル — 新ゴールが既に送信済みなので無視
                else:
                    self.get_logger().warning(f"JTC goal ended with status={status}")
                    self._publish_status("aborted")
            except Exception as exc:
                self.get_logger().error(f"Result callback error: {exc}")
                self._publish_status("aborted")

        def _publish_status(self, status: str) -> None:
            out = String()
            out.data = status
            self._status_pub.publish(out)

    node = MotionCommandExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
