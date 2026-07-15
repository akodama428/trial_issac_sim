"""MoveIt Servoと既存motion command契約を接続する実行adapter。"""
from __future__ import annotations
from dataclasses import dataclass
import json

from tomato_harvest_sim.msg.contracts import (
    JointStateSnapshot,
    MotionCommand,
    PhaseId,
    Pose3D,
)
from tomato_harvest_sim.robot.execute_manager.terminal_pose_tracking import (
    PoseTrackingDecision,
    decide_pose_tracking as decide_terminal_pose_tracking,
    moveit_link_pose,
    pose_from_transform,
    quaternion_from_pose,
    select_current_link_pose,
)
from tomato_harvest_sim.robot.execute_manager.pose_tracking_observability import (
    pose_tracking_metric_fields,
    tf_lookup_failure_metric_fields,
)


SERVO_JOINT_COMMAND_TOPIC = "/tomato_harvest/moveit_servo/delta_joint_cmds"
SERVO_SWITCH_COMMAND_TYPE_SERVICE = "/servo_node/switch_command_type"
GRIPPER_CLOSED_TOPIC = "/tomato_harvest/gripper_closed"
SERVO_CONTROL_RATE_HZ = 50.0
# Issue #46-4: Pandaの最小公称上限未満とし、URDF limitとsmoothingも維持する。
SERVO_JOINT_GAIN = 3.0
SERVO_MAX_VELOCITY_RAD_S = 0.8
# 旧0.05 radでは約16 mmの終端誤差が残ったため、把持窓に合わせて厳格化する。
SERVO_GOAL_TOLERANCE_RAD = 0.01
SERVO_STABLE_SAMPLES = 3
SERVO_TIMEOUT_MARGIN_SEC = 5.0
SERVO_POSE_COMMAND_TOPIC = "/tomato_harvest/moveit_servo/pose_target_cmds"
SERVO_STATUS_TOPIC = "/tomato_harvest/moveit_servo/status"
SERVO_PLANNING_FRAME = "panda_link0"
SERVO_END_EFFECTOR_FRAME = "panda_link8"
SERVO_POSE_POSITION_TOLERANCE_M = 0.005
SERVO_POSE_ORIENTATION_TOLERANCE_RAD = 0.03
SCENE_SNAPSHOT_MAX_AGE_SEC = 0.5


@dataclass(frozen=True)
class ServoTarget:
    """motion commandから抽出したServo用の関節目標。"""

    command_name: str
    phase: str
    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...]
    deadline_sec: float
    gripper_closed: bool | None
    pose_tracking_goal: Pose3D | None


@dataclass(frozen=True)
class JointJogDecision:
    """一周期分のJointJog指令と到達判定。"""

    joint_names: tuple[str, ...]
    velocities_rad_s: tuple[float, ...]
    max_error_rad: float
    reached: bool


def decide_pose_tracking(
    target: ServoTarget,
    current_pose: Pose3D,
    *,
    position_tolerance_m: float = SERVO_POSE_POSITION_TOLERANCE_M,
    orientation_tolerance_rad: float = SERVO_POSE_ORIENTATION_TOLERANCE_RAD,
) -> PoseTrackingDecision | None:
    """現在EEF poseが終端目標の6D許容範囲内か判定する。"""
    goal = target.pose_tracking_goal
    if goal is None:
        return None
    return decide_terminal_pose_tracking(
        goal,
        current_pose,
        position_tolerance_m=position_tolerance_m,
        orientation_tolerance_rad=orientation_tolerance_rad,
    )


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
        pose_tracking_goal=(
            moveit_link_pose(phase_plan.phase_goal_pose)
            if phase_plan.phase_id is PhaseId.MOVING_TO_GRASP
            and phase_plan.phase_goal_pose is not None
            else None
        ),
    )


def gripper_state_at_tracking_start(target: ServoTarget) -> bool | None:
    """終端pose tracking中は閉爪を遅延し、整列完了後のphaseへ委ねる。"""
    if target.pose_tracking_goal is not None and target.gripper_closed:
        return False
    return target.gripper_closed


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
    # Keep status first to preserve the execution_status CI log contract
    # (Issue #38).
    return json.dumps(payload, separators=(",", ":"))


def main() -> None:
    import time

    import rclpy
    from control_msgs.msg import JointJog
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.srv import ServoCommandType
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int8, String
    from tf2_ros import Buffer, TransformException, TransformListener

    from tomato_harvest_sim.msg.serialization import (
        motion_command_from_json,
        scene_snapshot_from_json,
    )
    from tomato_harvest_sim.msg.topics import (
        EXECUTION_STATUS_TOPIC,
        JOINT_STATES_TOPIC,
        MOTION_COMMAND_TOPIC,
        SCENE_SNAPSHOT_TOPIC,
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
            self._desired_command_type = ServoCommandType.Request.JOINT_JOG
            self._active_command_type: int | None = None
            self._switch_request_pending = False
            self._last_gripper_closed: bool | None = None
            self._pose_sequence_id = 0
            self._pose_published_count = 0
            self._tf_success_count = 0
            self._tf_failure_count = 0
            self._last_tf_success_sec: float | None = None
            self._servo_status: int | None = None
            self._runtime_tool_pose: Pose3D | None = None
            self._runtime_tool_pose_updated_sec: float | None = None
            self._jog_pub = self.create_publisher(
                JointJog, SERVO_JOINT_COMMAND_TOPIC, 10
            )
            self._pose_pub = self.create_publisher(
                PoseStamped, SERVO_POSE_COMMAND_TOPIC, 10
            )
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
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
                String, SCENE_SNAPSHOT_TOPIC, self._on_scene_snapshot, 10
            )
            self.create_subscription(
                JointState, JOINT_STATES_TOPIC, self._on_joint_state, 10
            )
            self.create_subscription(
                Int8, SERVO_STATUS_TOPIC, self._on_servo_status, 10
            )
            self._switch_client = self.create_client(
                ServoCommandType, SERVO_SWITCH_COMMAND_TYPE_SERVICE
            )
            self.create_timer(0.5, self._ensure_joint_jog_mode)
            self.create_timer(1.0 / SERVO_CONTROL_RATE_HZ, self._control_step)
            self._publish_status("idle")

        def _ensure_joint_jog_mode(self) -> None:
            if (
                self._active_command_type == self._desired_command_type
                or self._switch_request_pending
                or not self._switch_client.service_is_ready()
            ):
                return
            request = ServoCommandType.Request()
            request.command_type = self._desired_command_type
            requested_command_type = self._desired_command_type
            self._switch_request_pending = True
            future = self._switch_client.call_async(request)
            future.add_done_callback(
                lambda completed: self._on_command_type_response(
                    completed, requested_command_type
                )
            )

        def _on_command_type_response(
            self, future: object, requested_command_type: int
        ) -> None:
            self._switch_request_pending = False
            try:
                response = future.result()  # type: ignore[attr-defined]
                if bool(response.success):
                    self._active_command_type = requested_command_type
            except Exception as exc:  # ROS future boundary
                self.get_logger().warning(f"Servo command type switch failed: {exc}")
                return
            if self._active_command_type == requested_command_type:
                self.get_logger().info(metric_line(
                    "servo_command_mode_ready", command_type=self._active_command_type,
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
            self._desired_command_type = (
                ServoCommandType.Request.POSE
                if target.pose_tracking_goal is not None
                else ServoCommandType.Request.JOINT_JOG
            )
            self._target_started_sec = started_at_sec
            self._stable_samples = 0
            self._reset_pose_tracking_observation()
            self._publish_gripper(gripper_state_at_tracking_start(target))
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

        def _on_servo_status(self, msg: Int8) -> None:
            self._servo_status = int(msg.data)

        def _on_scene_snapshot(self, msg: String) -> None:
            try:
                snapshot = scene_snapshot_from_json(msg.data)
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                self.get_logger().warning("Invalid scene snapshot ignored")
                return
            self._runtime_tool_pose = snapshot.robot_tool_pose
            self._runtime_tool_pose_updated_sec = time.monotonic()

        def _reset_pose_tracking_observation(self) -> None:
            self._pose_sequence_id = 0
            self._pose_published_count = 0
            self._tf_success_count = 0
            self._tf_failure_count = 0
            self._last_tf_success_sec = None

        def _control_step(self) -> None:
            if (
                self._active_command_type != self._desired_command_type
                or self._target is None
            ):
                return
            now_sec = time.monotonic()
            if now_sec > self._target.deadline_sec:
                if self._target.pose_tracking_goal is not None:
                    self.get_logger().info(metric_line(
                        "servo_pose_tracking_timeout",
                        published_count=self._pose_published_count,
                        servo_status=self._servo_status,
                        tf_failure_count=self._tf_failure_count,
                        tf_success_count=self._tf_success_count,
                    ))
                self._publish_status("aborted", abort_reason="servo_target_timeout")
                self._target = None
                return
            if self._target.pose_tracking_goal is not None:
                self._control_pose_tracking(now_sec)
                return
            if self._joint_state is None:
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

        def _control_pose_tracking(self, now_sec: float) -> None:
            """MOVING_TO_GRASP終端をServo pose commandで閉ループ追従する。"""
            if self._target is None or self._target.pose_tracking_goal is None:
                return
            goal = self._target.pose_tracking_goal
            self._pose_sequence_id += 1
            command = PoseStamped()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = SERVO_PLANNING_FRAME
            command.pose.position.x = goal.x
            command.pose.position.y = goal.y
            command.pose.position.z = goal.z
            qx, qy, qz, qw = quaternion_from_pose(goal)
            command.pose.orientation.x = qx
            command.pose.orientation.y = qy
            command.pose.orientation.z = qz
            command.pose.orientation.w = qw
            self._pose_pub.publish(command)
            self._pose_published_count += 1
            tf_pose: Pose3D | None = None
            try:
                transform = self._tf_buffer.lookup_transform(
                    SERVO_PLANNING_FRAME, SERVO_END_EFFECTOR_FRAME, rclpy.time.Time()
                )
            except TransformException as exc:
                self._tf_failure_count += 1
                last_success_age_sec = (
                    round(now_sec - self._last_tf_success_sec, 6)
                    if self._last_tf_success_sec is not None else None
                )
                self.get_logger().info(metric_line(
                    "servo_pose_tracking_sample",
                    **tf_lookup_failure_metric_fields(
                        sequence_id=self._pose_sequence_id,
                        published_count=self._pose_published_count,
                        planning_frame=SERVO_PLANNING_FRAME,
                        end_effector_frame=SERVO_END_EFFECTOR_FRAME,
                        error=str(exc),
                        servo_status=self._servo_status,
                        tf_success_count=self._tf_success_count,
                        tf_failure_count=self._tf_failure_count,
                        last_success_age_sec=last_success_age_sec,
                    ),
                ))
            else:
                self._tf_success_count += 1
                self._last_tf_success_sec = now_sec
                tf_pose = pose_from_transform(transform)
            snapshot_pose = self._runtime_tool_pose
            if (
                self._runtime_tool_pose_updated_sec is None
                or now_sec - self._runtime_tool_pose_updated_sec
                > SCENE_SNAPSHOT_MAX_AGE_SEC
            ):
                snapshot_pose = None
            selected_pose = select_current_link_pose(tf_pose, snapshot_pose)
            if selected_pose is None:
                return
            current_pose = selected_pose.pose
            decision = decide_pose_tracking(self._target, current_pose)
            if decision is None:
                return
            self._publish_status("running")
            next_stable_samples = self._stable_samples + 1 if decision.reached else 0
            self.get_logger().info(metric_line(
                "servo_pose_tracking_sample",
                **pose_tracking_metric_fields(
                    sequence_id=self._pose_sequence_id,
                    published_count=self._pose_published_count,
                    planning_frame=SERVO_PLANNING_FRAME,
                    end_effector_frame=SERVO_END_EFFECTOR_FRAME,
                    pose_source=selected_pose.source,
                    target=goal,
                    current=current_pose,
                    position_error_m=decision.position_error_m,
                    orientation_error_rad=decision.orientation_error_rad,
                    reached=decision.reached,
                    stable_samples=next_stable_samples,
                    servo_status=self._servo_status,
                    tf_success_count=self._tf_success_count,
                    tf_failure_count=self._tf_failure_count,
                ),
            ))
            self._stable_samples = next_stable_samples
            if self._stable_samples < SERVO_STABLE_SAMPLES:
                return
            self.get_logger().info(metric_line(
                "servo_pose_target_succeeded", phase=self._target.phase,
                position_error_m=decision.position_error_m,
                orientation_error_rad=decision.orientation_error_rad,
                latency_ms=round(
                    (now_sec - (self._target_started_sec or now_sec)) * 1000.0, 3
                ),
            ))
            self._publish_status("succeeded")
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
