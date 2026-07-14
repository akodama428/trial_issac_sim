"""MoveIt Servoと既存motion command契約を接続する実行adapter。"""
from __future__ import annotations

from dataclasses import dataclass
import json

from tomato_harvest_sim.msg.contracts import (
    JointStateSnapshot,
    MotionCommand,
)


SERVO_JOINT_COMMAND_TOPIC = "/tomato_harvest/moveit_servo/delta_joint_cmds"
SERVO_SWITCH_COMMAND_TYPE_SERVICE = "/servo_node/switch_command_type"
GRIPPER_CLOSED_TOPIC = "/tomato_harvest/gripper_closed"
SERVO_CONTROL_RATE_HZ = 50.0
# Issue #46-4 tuned profile.  The 0.8 rad/s cap stays below Panda's slowest
# nominal joint velocity limit (joint 2: 1.0 rad/s); MoveIt Servo still applies
# the URDF's position-dependent joint limits and output smoothing.
SERVO_JOINT_GAIN = 3.0
SERVO_MAX_VELOCITY_RAD_S = 0.8
# 0.05 rad left about 16 mm of Cartesian grasp error in the E2E scene and
# caused the physics grasp gate to fail.  Complete only after a tighter joint
# endpoint convergence suitable for the existing grasp acceptance window.
SERVO_GOAL_TOLERANCE_RAD = 0.01
SERVO_STABLE_SAMPLES = 3
SERVO_TIMEOUT_MARGIN_SEC = 5.0


@dataclass(frozen=True)
class ServoTarget:
    """motion commandから抽出したServo用の関節目標。"""

    command_name: str
    phase: str
    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...]
    deadline_sec: float
    gripper_closed: bool | None


@dataclass(frozen=True)
class JointJogDecision:
    """一周期分のJointJog指令と到達判定。"""

    joint_names: tuple[str, ...]
    velocities_rad_s: tuple[float, ...]
    max_error_rad: float
    reached: bool


def servo_target_from_command(
    command: MotionCommand,
    *,
    started_at_sec: float,
    timeout_margin_sec: float = SERVO_TIMEOUT_MARGIN_SEC,
) -> ServoTarget | None:
    """既存trajectoryの終端をServoの閉ループ目標へ変換する。"""
    phase_plan = command.phase_motion_plan
    if phase_plan is None or phase_plan.joint_trajectory is None:
        return None
    trajectory = phase_plan.joint_trajectory
    if not trajectory.points or not trajectory.joint_names:
        return None
    endpoint = trajectory.points[-1]
    if len(endpoint.positions_rad) != len(trajectory.joint_names):
        return None
    planned_duration_sec = max(
        point.time_from_start_sec for point in trajectory.points
    )
    return ServoTarget(
        command_name=command.command_name,
        phase=phase_plan.phase_id.value,
        joint_names=trajectory.joint_names,
        positions_rad=endpoint.positions_rad,
        deadline_sec=started_at_sec + planned_duration_sec + timeout_margin_sec,
        gripper_closed=command.gripper_closed,
    )


def decide_joint_jog(
    target: ServoTarget,
    current_state: JointStateSnapshot,
    *,
    gain: float = SERVO_JOINT_GAIN,
    max_velocity_rad_s: float = SERVO_MAX_VELOCITY_RAD_S,
    tolerance_rad: float = SERVO_GOAL_TOLERANCE_RAD,
) -> JointJogDecision | None:
    """現在誤差から比例JointJogを生成し、全関節到達を判定する。"""
    current_by_name = dict(zip(
        current_state.joint_names, current_state.positions_rad, strict=True
    ))
    if any(name not in current_by_name for name in target.joint_names):
        return None
    errors = tuple(
        desired - current_by_name[name]
        for name, desired in zip(
            target.joint_names, target.positions_rad, strict=True
        )
    )
    max_error_rad = max((abs(error) for error in errors), default=0.0)
    reached = max_error_rad <= tolerance_rad
    velocities = tuple(
        0.0 if reached else max(
            -max_velocity_rad_s, min(max_velocity_rad_s, gain * error)
        )
        for error in errors
    )
    return JointJogDecision(
        joint_names=target.joint_names,
        velocities_rad_s=velocities,
        max_error_rad=round(max_error_rad, 6),
        reached=reached,
    )


def execution_status_payload(
    status: str,
    *,
    max_error_rad: float | None = None,
    abort_reason: str | None = None,
) -> str:
    """既存trajectory monitorと互換のexecution status JSONを返す。"""
    payload: dict[str, object] = {"status": status}
    if max_error_rad is not None:
        payload["tracking_error_rad"] = max_error_rad
        payload["max_joint_error_rad"] = max_error_rad
    if abort_reason is not None:
        payload["abort_reason"] = abort_reason
    # Keep status first for compatibility with the existing executor's CI log
    # contract (Issue #38).
    return json.dumps(payload, separators=(",", ":"))


def main() -> None:
    import time

    import rclpy
    from control_msgs.msg import JointJog
    from moveit_msgs.srv import ServoCommandType
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    from tomato_harvest_sim.msg.serialization import motion_command_from_json
    from tomato_harvest_sim.msg.topics import (
        EXECUTION_STATUS_TOPIC,
        JOINT_STATES_TOPIC,
        MOTION_COMMAND_TOPIC,
    )
    from tomato_harvest_sim.robot.motion_planner.observability import metric_line

    rclpy.init()

    class ServoExecutionAdapter(Node):  # type: ignore[misc]
        """motion command終端へJointJogし、既存status契約を維持する。"""

        def __init__(self) -> None:
            super().__init__("servo_execution_adapter")
            self._target: ServoTarget | None = None
            self._joint_state: JointStateSnapshot | None = None
            self._stable_samples = 0
            self._target_started_sec: float | None = None
            self._command_type_ready = False
            self._last_gripper_closed: bool | None = None
            self._jog_pub = self.create_publisher(
                JointJog, SERVO_JOINT_COMMAND_TOPIC, 10
            )
            self._status_pub = self.create_publisher(
                String, EXECUTION_STATUS_TOPIC, 10
            )
            self._gripper_pub = self.create_publisher(
                String, GRIPPER_CLOSED_TOPIC, 10
            )
            self.create_subscription(
                String, MOTION_COMMAND_TOPIC, self._on_command, 10
            )
            self.create_subscription(
                JointState, JOINT_STATES_TOPIC, self._on_joint_state, 10
            )
            self._switch_client = self.create_client(
                ServoCommandType, SERVO_SWITCH_COMMAND_TYPE_SERVICE
            )
            self.create_timer(0.5, self._ensure_joint_jog_mode)
            self.create_timer(1.0 / SERVO_CONTROL_RATE_HZ, self._control_step)
            self._publish_status("idle")

        def _ensure_joint_jog_mode(self) -> None:
            if self._command_type_ready or not self._switch_client.service_is_ready():
                return
            request = ServoCommandType.Request()
            request.command_type = ServoCommandType.Request.JOINT_JOG
            future = self._switch_client.call_async(request)
            future.add_done_callback(self._on_command_type_response)

        def _on_command_type_response(self, future: object) -> None:
            try:
                response = future.result()  # type: ignore[attr-defined]
                self._command_type_ready = bool(response.success)
            except Exception as exc:  # ROS future boundary
                self.get_logger().warning(f"Servo command type switch failed: {exc}")
                return
            if self._command_type_ready:
                self.get_logger().info(metric_line(
                    "servo_joint_jog_mode_ready"
                ))

        def _on_command(self, msg: String) -> None:
            try:
                command = motion_command_from_json(msg.data)
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                self._publish_status("aborted", abort_reason="invalid_motion_command")
                return
            started_at_sec = time.monotonic()
            target = servo_target_from_command(command, started_at_sec=started_at_sec)
            if target is None:
                self._publish_status("aborted", abort_reason="missing_trajectory")
                return
            self._target = target
            self._target_started_sec = started_at_sec
            self._stable_samples = 0
            self._publish_gripper(target.gripper_closed)
            self._publish_status("running")
            self.get_logger().info(metric_line(
                "servo_target_started", phase=target.phase,
                command_name=target.command_name,
            ))

        def _on_joint_state(self, msg: JointState) -> None:
            self._joint_state = JointStateSnapshot(
                tuple(str(name) for name in msg.name),
                tuple(float(position) for position in msg.position),
            )

        def _control_step(self) -> None:
            if (
                not self._command_type_ready
                or self._target is None
                or self._joint_state is None
            ):
                return
            now_sec = time.monotonic()
            if now_sec > self._target.deadline_sec:
                self._publish_status("aborted", abort_reason="servo_target_timeout")
                self._target = None
                return
            decision = decide_joint_jog(self._target, self._joint_state)
            if decision is None:
                self._publish_status("aborted", abort_reason="incomplete_joint_state")
                self._target = None
                return
            jog = JointJog()
            jog.header.stamp = self.get_clock().now().to_msg()
            jog.joint_names = list(decision.joint_names)
            jog.velocities = list(decision.velocities_rad_s)
            self._jog_pub.publish(jog)
            self._publish_status("running", max_error_rad=decision.max_error_rad)
            self._stable_samples = (
                self._stable_samples + 1 if decision.reached else 0
            )
            if self._stable_samples < SERVO_STABLE_SAMPLES:
                return
            self.get_logger().info(metric_line(
                "servo_target_succeeded", phase=self._target.phase,
                max_error_rad=decision.max_error_rad,
                latency_ms=round(
                    (now_sec - (self._target_started_sec or now_sec)) * 1000.0,
                    3,
                ),
            ))
            self._publish_status("succeeded", max_error_rad=decision.max_error_rad)
            self._target = None
            self._target_started_sec = None
            self._stable_samples = 0

        def _publish_gripper(self, closed: bool | None) -> None:
            if closed is None or closed == self._last_gripper_closed:
                return
            self._last_gripper_closed = closed
            msg = String()
            msg.data = "true" if closed else "false"
            self._gripper_pub.publish(msg)

        def _publish_status(
            self,
            status: str,
            *,
            max_error_rad: float | None = None,
            abort_reason: str | None = None,
        ) -> None:
            msg = String()
            msg.data = execution_status_payload(
                status,
                max_error_rad=max_error_rad,
                abort_reason=abort_reason,
            )
            self._status_pub.publish(msg)
            self.get_logger().info(f"execution_status {msg.data}")

    node = ServoExecutionAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
