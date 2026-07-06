"""trajectory_monitor_node — C++ MotionCommandExecutor の execution_status を監視し trajectory_status を publish する。

アーキテクチャ仕様: docs/index.html §trajectory_monitor_node
"""
from __future__ import annotations


def trajectory_status_from_execution_status(execution_status: str) -> str:
    """execution_status 文字列を trajectory_status 文字列へ変換する。

    Args:
        execution_status: "running", "succeeded", "aborted" のいずれか。

    Returns:
        "ok" または "aborted"。
    """
    if execution_status == "aborted":
        return "aborted"
    return "ok"


def main() -> None:
    import json
    import rclpy
    from std_msgs.msg import String
    from rclpy.node import Node
    from tomato_harvest_sim.msg.topics import EXECUTION_STATUS_TOPIC, TRAJECTORY_STATUS_TOPIC

    rclpy.init()

    class TrajectoryMonitorNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("trajectory_monitor_node")
            self._pub = self.create_publisher(String, TRAJECTORY_STATUS_TOPIC, 10)
            self.create_subscription(String, EXECUTION_STATUS_TOPIC, self._on_status, 10)

        def _on_status(self, msg: String) -> None:
            try:
                data = json.loads(msg.data)
                raw = str(data.get("status", "running"))
            except (json.JSONDecodeError, AttributeError):
                raw = msg.data.strip()
            out = String()
            out.data = trajectory_status_from_execution_status(raw)
            self._pub.publish(out)

    node = TrajectoryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
