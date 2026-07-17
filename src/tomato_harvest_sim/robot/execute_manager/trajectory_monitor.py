"""trajectory_monitor_node — servo_execution_adapter の execution_status を監視し trajectory_status を publish する。

アーキテクチャ仕様: docs/index.html §trajectory_monitor_node
"""
from __future__ import annotations

import json

# 実行中・abort診断のうち、下流 (trajectory_planner_node) の契約へ通す field。
# 最大追従誤差・律速joint・abort分類はabort原因の特定に使う (Issue #32)。
# ピーク時の律速joint目標/実位置は関節限界近傍の固着判定に使う (Issue #37)。
_EXECUTION_DIAGNOSTIC_FIELDS = (
    "tracking_error_rad",
    "max_joint_error_rad",
    "limiting_joint",
    "limiting_joint_desired_rad",
    "limiting_joint_actual_rad",
    "abort_reason",
    "scale",
    "stall_elapsed_sec",
    "stalled",
)


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


def trajectory_status_payload(execution_status_raw: str) -> str:
    """execution_status のraw値を、abort診断付きのtrajectory status JSONへ変換する。

    servo_execution_adapterはabort時に status とともに最大joint追従誤差・
    律速joint・abort reasonをJSONで報告する (Issue #32)。plain文字列
    ("running"等) の旧形式も受け付け、常にJSON文字列を返す。

    Args:
        execution_status_raw: servo_execution_adapterが publish した生のstatus文字列。

    Returns:
        {"status": "ok"|"aborted", ...診断fields} のJSON文字列。
    """
    status_value = execution_status_raw.strip()
    diagnostics: dict[str, object] = {}
    try:
        data = json.loads(execution_status_raw)
    except (json.JSONDecodeError, TypeError):
        data = None
    if isinstance(data, dict):
        status_value = str(data.get("status", "")).strip()
        diagnostics = {
            field: data[field]
            for field in _EXECUTION_DIAGNOSTIC_FIELDS
            if data.get(field) is not None
        }
    payload: dict[str, object] = {
        "status": trajectory_status_from_execution_status(status_value),
    }
    payload.update(diagnostics)
    return json.dumps(payload, sort_keys=True)


def main() -> None:
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
            out = String()
            out.data = trajectory_status_payload(msg.data)
            self._pub.publish(out)

    node = TrajectoryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
