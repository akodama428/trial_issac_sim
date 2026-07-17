"""MoveIt Servoと既存motion command契約を接続する実行adapter。"""
from __future__ import annotations
from dataclasses import dataclass
import json
import os
from tomato_harvest_sim.msg.contracts import (
    JointStateSnapshot,
    JointTrajectoryPoint,
    MotionCommand,
    Pose3D,
)
from tomato_harvest_sim.robot.execute_manager.terminal_pose_tracking import (
    PoseTrackingDecision,
    decide_pose_tracking as decide_terminal_pose_tracking,
    moveit_link_pose,
    pose_from_transform,
    quaternion_from_pose,
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
SERVO_DEADLINE_STRETCH_FACTOR = 2.0
SERVO_PROGRESS_SCALE_START_ERROR_RAD = 0.10
SERVO_PROGRESS_SCALE_ZERO_ERROR_RAD = 0.20
SERVO_STALL_VELOCITY_RAD_S = 0.05
SERVO_STALL_DURATION_SEC = 0.5
SERVO_POSE_COMMAND_TOPIC = "/tomato_harvest/moveit_servo/pose_target_cmds"
SERVO_STATUS_TOPIC = "/tomato_harvest/moveit_servo/status"
JTC_COMMAND_TOPIC = "/joint_trajectory_controller/joint_trajectory"
ISAAC_JOINT_COMMAND_TOPIC = "/isaac_joint_commands"
SERVO_PLANNING_FRAME = "panda_link0"
SERVO_END_EFFECTOR_FRAME = "panda_link8"
SERVO_POSE_POSITION_TOLERANCE_M = 0.0051
SERVO_POSE_ORIENTATION_TOLERANCE_RAD = 0.03


@dataclass(frozen=True)
class ServoTarget:
    """motion commandから抽出したServo用の関節目標。"""

    command_name: str
    phase: str
    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...]
    trajectory_points: tuple[JointTrajectoryPoint, ...]
    planned_duration_sec: float
    deadline_sec: float
    gripper_closed: bool | None
    pose_tracking_goal: Pose3D | None


@dataclass(frozen=True)
class JointJogDecision:
    """一周期分のJointJog指令と到達判定。"""

    joint_names: tuple[str, ...]
    velocities_rad_s: tuple[float, ...]
    max_error_rad: float
    progress_scale: float
    reached: bool


@dataclass(frozen=True)
class TrajectoryReference:
    """時間同期軌道から得た一周期分の位置・速度参照。"""

    positions_rad: tuple[float, ...]
    velocities_rad_s: tuple[float, ...]
    final: bool


def progress_scale(
    max_error_rad: float,
    *,
    start_error_rad: float = SERVO_PROGRESS_SCALE_START_ERROR_RAD,
    zero_error_rad: float = SERVO_PROGRESS_SCALE_ZERO_ERROR_RAD,
) -> float:
    """追従誤差を連続な軌道進行率へ変換する。"""
    if max_error_rad <= start_error_rad:
        return 1.0
    if max_error_rad >= zero_error_rad:
        return 0.0
    return (zero_error_rad - max_error_rad) / (
        zero_error_rad - start_error_rad
    )


class StallDetector:
    """feedback-onlyかつ実測静止の継続時間から回復不能停止を検出する。"""

    def __init__(
        self,
        *,
        velocity_threshold_rad_s: float = SERVO_STALL_VELOCITY_RAD_S,
        duration_sec: float = SERVO_STALL_DURATION_SEC,
    ) -> None:
        self._velocity_threshold_rad_s = velocity_threshold_rad_s
        self._duration_sec = duration_sec
        self._started_at_sec: float | None = None
        self.elapsed_sec = 0.0

    def update(
        self,
        *,
        now_sec: float,
        progress_scale: float,
        velocities_rad_s: tuple[float, ...],
    ) -> bool:
        stationary = (
            bool(velocities_rad_s)
            and progress_scale == 0.0
            and max(abs(velocity) for velocity in velocities_rad_s)
            < self._velocity_threshold_rad_s
        )
        if not stationary:
            self.reset()
            return False
        if self._started_at_sec is None:
            self._started_at_sec = now_sec
        self.elapsed_sec = max(0.0, now_sec - self._started_at_sec)
        return self.elapsed_sec >= self._duration_sec

    def reset(self) -> None:
        self._started_at_sec = None
        self.elapsed_sec = 0.0


class GripperGate:
    """gripper intentのpublish要否とcommand間dedupeを単一所有する。"""

    def __init__(self) -> None:
        self._last_closed: bool | None = None

    def command_started(self, closed: bool | None) -> bool | None:
        return self._decide(closed)

    def terminal_reached(self, closed: bool | None) -> bool | None:
        return self._decide(closed)

    def _decide(self, closed: bool | None) -> bool | None:
        if closed is None or closed == self._last_closed:
            return None
        self._last_closed = closed
        return closed


class CommandLifecycle:
    """command受理から終端確定までの実行状態を所有する。"""

    def __init__(self) -> None:
        self.target: ServoTarget | None = None
        self.started_at_sec: float | None = None
        self.stable_samples = 0
        self.reference_elapsed_sec = 0.0
        self._last_clock_sec: float | None = None

    def start(self, target: ServoTarget, started_at_sec: float) -> None:
        self.target = target
        self.started_at_sec = started_at_sec
        self.stable_samples = 0
        self.reference_elapsed_sec = 0.0
        self._last_clock_sec = started_at_sec

    def update_reference_clock(
        self, *, now_sec: float, progress_scale: float
    ) -> None:
        """軌道参照時計を追従誤差に応じた連続速度で進める。"""
        previous_sec = self._last_clock_sec
        self._last_clock_sec = now_sec
        if previous_sec is None:
            return
        self.reference_elapsed_sec = round(
            self.reference_elapsed_sec
            + progress_scale * max(0.0, now_sec - previous_sec),
            9,
        )

    def record_reached(self, reached: bool) -> bool:
        self.stable_samples = self.stable_samples + 1 if reached else 0
        return self.stable_samples >= SERVO_STABLE_SAMPLES

    def clear(self) -> None:
        self.target = None
        self.started_at_sec = None
        self.stable_samples = 0
        self.reference_elapsed_sec = 0.0
        self._last_clock_sec = None


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
    deadline_stretch_factor: float = SERVO_DEADLINE_STRETCH_FACTOR,
) -> ServoTarget | None:
    """既存trajectoryの終端をServoの閉ループ目標へ変換する。"""
    phase_plan = command.phase_motion_plan
    if phase_plan is None or phase_plan.joint_trajectory is None:
        return None
    trajectory = phase_plan.joint_trajectory
    if not trajectory.points or not trajectory.joint_names:
        return None
    endpoint = trajectory.points[-1]
    if any(len(point.positions_rad) != len(trajectory.joint_names) for point in trajectory.points):
        return None
    planned_duration_sec = max(
        point.time_from_start_sec for point in trajectory.points
    )
    return ServoTarget(
        command_name=command.command_name,
        phase=phase_plan.phase_id.value,
        joint_names=trajectory.joint_names,
        positions_rad=endpoint.positions_rad,
        trajectory_points=trajectory.points,
        planned_duration_sec=planned_duration_sec,
        deadline_sec=started_at_sec + planned_duration_sec * deadline_stretch_factor + timeout_margin_sec,
        gripper_closed=command.gripper_closed,
        pose_tracking_goal=(
            moveit_link_pose(phase_plan.phase_goal_pose)
            if command.terminal_pose_tracking and phase_plan.phase_goal_pose is not None
            else None
        ),
    )


def trajectory_reference_at(target: ServoTarget, elapsed_sec: float) -> TrajectoryReference:
    """軌道時刻を区分線形補間し、位置・速度参照を返す。"""
    points = target.trajectory_points
    if len(points) == 1:
        return TrajectoryReference(
            points[0].positions_rad, tuple(0.0 for _ in target.joint_names), True
        )
    time_sec = max(0.0, elapsed_sec)
    if time_sec >= target.planned_duration_sec:
        return TrajectoryReference(
            points[-1].positions_rad, tuple(0.0 for _ in target.joint_names), True
        )
    right_index = next(
        (index for index, point in enumerate(points) if point.time_from_start_sec > time_sec),
        len(points) - 1,
    )
    left = points[max(0, right_index - 1)]
    right = points[right_index]
    duration_sec = max(right.time_from_start_sec - left.time_from_start_sec, 1e-9)
    ratio = min(1.0, max(0.0, (time_sec - left.time_from_start_sec) / duration_sec))
    positions = tuple(
        start + ratio * (end - start)
        for start, end in zip(left.positions_rad, right.positions_rad, strict=True)
    )
    if left.velocities_rad_s is not None and right.velocities_rad_s is not None:
        velocities = tuple(
            start + ratio * (end - start)
            for start, end in zip(left.velocities_rad_s, right.velocities_rad_s, strict=True)
        )
    else:
        velocities = tuple(
            (end - start) / duration_sec
            for start, end in zip(left.positions_rad, right.positions_rad, strict=True)
        )
    return TrajectoryReference(positions, velocities, False)


def decide_time_synchronized_joint_jog(
    target: ServoTarget,
    current_state: JointStateSnapshot,
    reference: TrajectoryReference,
    *,
    gain: float = SERVO_JOINT_GAIN,
    max_velocity_rad_s: float = SERVO_MAX_VELOCITY_RAD_S,
    tolerance_rad: float = SERVO_GOAL_TOLERANCE_RAD,
) -> JointJogDecision | None:
    """時間同期参照へfeed-forwardと位置feedbackを合成する。"""
    current_by_name = dict(zip(
        current_state.joint_names, current_state.positions_rad, strict=True
    ))
    if any(name not in current_by_name for name in target.joint_names):
        return None
    errors = tuple(
        desired - current_by_name[name]
        for name, desired in zip(target.joint_names, reference.positions_rad, strict=True)
    )
    max_error_rad = max((abs(error) for error in errors), default=0.0)
    reported_max_error_rad = round(max_error_rad, 6)
    scale = progress_scale(max_error_rad)
    reached = reference.final and max_error_rad <= tolerance_rad
    velocities = tuple(
        0.0 if reached else max(
            -max_velocity_rad_s,
            min(max_velocity_rad_s, scale * feed_forward + gain * error),
        )
        for feed_forward, error in zip(reference.velocities_rad_s, errors, strict=True)
    )
    return JointJogDecision(
        target.joint_names, velocities, reported_max_error_rad, scale, reached
    )


def gripper_state_for_tracking(target: ServoTarget) -> bool | None:
    """plannerが決めたgripper指令を開始時と成功時に適用する。"""
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
        progress_scale=1.0,
        reached=reached,
    )


def execution_status_payload(
    status: str,
    *,
    max_error_rad: float | None = None,
    abort_reason: str | None = None,
    progress_scale: float | None = None,
    stall_elapsed_sec: float | None = None,
    stalled: bool | None = None,
) -> str:
    """既存trajectory monitorと互換のexecution status JSONを返す。"""
    payload: dict[str, object] = {"status": status}
    if max_error_rad is not None:
        payload["tracking_error_rad"] = max_error_rad
        payload["max_joint_error_rad"] = max_error_rad
    if abort_reason is not None:
        payload["abort_reason"] = abort_reason
    if progress_scale is not None:
        payload["scale"] = progress_scale
    if stall_elapsed_sec is not None:
        payload["stall_elapsed_sec"] = stall_elapsed_sec
    if stalled is not None:
        payload["stalled"] = stalled
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
    from trajectory_msgs.msg import JointTrajectory as RosJointTrajectory
    from tf2_ros import Buffer, TransformException, TransformListener

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
            self._lifecycle = CommandLifecycle()
            self._joint_state: JointStateSnapshot | None = None
            self._joint_velocities: dict[str, float] = {}
            self._stall_detector = StallDetector()
            self._desired_command_type = ServoCommandType.Request.JOINT_JOG
            self._active_command_type: int | None = None
            self._switch_request_pending = False
            self._gripper_gate = GripperGate()
            self._pose_sequence_id = 0
            self._pose_published_count = 0
            self._tf_success_count = 0
            self._tf_failure_count = 0
            self._last_tf_success_sec: float | None = None
            self._servo_status: int | None = None
            self._last_abort_sec: float | None = None
            self._last_jtc_observation_sec: float | None = None
            self._last_jtc_positions: dict[str, float] = {}
            self._last_hardware_positions: dict[str, float] = {}
            self._last_hardware_velocities: dict[str, float] = {}
            self._last_tracking_observation_sec: float | None = None
            self._tracking_sequence_id = 0
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
                JointState, JOINT_STATES_TOPIC, self._on_joint_state, 10
            )
            self.create_subscription(
                Int8, SERVO_STATUS_TOPIC, self._on_servo_status, 10
            )
            self.create_subscription(
                RosJointTrajectory, JTC_COMMAND_TOPIC, self._on_jtc_command, 10
            )
            self.create_subscription(
                JointState, ISAAC_JOINT_COMMAND_TOPIC, self._on_hardware_command, 10
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
            target = servo_target_from_command(
                command,
                started_at_sec=started_at_sec,
                timeout_margin_sec=float(os.environ.get(
                    "TOMATO_HARVEST_SERVO_TIMEOUT_MARGIN_SEC",
                    str(SERVO_TIMEOUT_MARGIN_SEC),
                )),
                deadline_stretch_factor=float(os.environ.get(
                    "TOMATO_HARVEST_SERVO_DEADLINE_STRETCH_FACTOR",
                    str(SERVO_DEADLINE_STRETCH_FACTOR),
                )),
            )
            if target is None:
                self._publish_status("aborted", abort_reason="missing_trajectory")
                return
            self._lifecycle.start(target, started_at_sec)
            self._stall_detector.reset()
            self._last_abort_sec = None
            self._last_tracking_observation_sec = None
            self._desired_command_type = ServoCommandType.Request.JOINT_JOG
            self._reset_pose_tracking_observation()
            self._publish_gripper(self._gripper_gate.command_started(target.gripper_closed))
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
            self._joint_velocities = {
                str(name): float(velocity)
                for name, velocity in zip(msg.name, msg.velocity)
            }

        def _on_servo_status(self, msg: Int8) -> None:
            self._servo_status = int(msg.data)

        def _on_jtc_command(self, msg: RosJointTrajectory) -> None:
            """ServoがJTCへ出した位置列をabort前後の切り分け用に記録する。"""
            if self._joint_state is None or not msg.points or not msg.joint_names:
                return
            now_sec = time.monotonic()
            endpoint = msg.points[-1].positions
            # ログのthrottleとは独立に最新指令を保持し、tracking sampleと同時刻で比較する。
            self._last_jtc_positions = {
                str(name): float(position)
                for name, position in zip(msg.joint_names, endpoint, strict=True)
            }
            if (
                self._last_jtc_observation_sec is not None
                and now_sec - self._last_jtc_observation_sec < (
                    0.1 if self._last_abort_sec is not None else 0.5
                )
            ):
                return
            current = dict(zip(
                self._joint_state.joint_names,
                self._joint_state.positions_rad,
                strict=True,
            ))
            deltas = tuple(
                abs(float(position) - current.get(name, float(position)))
                for name, position in zip(msg.joint_names, endpoint, strict=True)
            )
            self._last_jtc_observation_sec = now_sec
            self.get_logger().info(metric_line(
                "jtc_command_observed",
                adapter_target_active=self._lifecycle.target is not None,
                max_position_delta_rad=round(max(deltas, default=0.0), 6),
                phase=(self._lifecycle.target.phase if self._lifecycle.target else None),
                seconds_after_abort=(
                    round(now_sec - self._last_abort_sec, 6)
                    if self._last_abort_sec is not None else None
                ),
            ))

        def _on_hardware_command(self, msg: JointState) -> None:
            """ros2_controlがIsaacへ実際に渡したposition/velocity指令を保持する。"""
            self._last_hardware_positions = {
                str(name): float(position)
                for name, position in zip(msg.name, msg.position)
            }
            self._last_hardware_velocities = {
                str(name): float(velocity)
                for name, velocity in zip(msg.name, msg.velocity)
            }

        def _reset_pose_tracking_observation(self) -> None:
            self._pose_sequence_id = 0
            self._pose_published_count = 0
            self._tf_success_count = 0
            self._tf_failure_count = 0
            self._last_tf_success_sec = None

        def _control_step(self) -> None:
            if (
                self._active_command_type != self._desired_command_type
                or self._lifecycle.target is None
            ):
                return
            now_sec = time.monotonic()
            target = self._lifecycle.target
            if now_sec > target.deadline_sec:
                if target.pose_tracking_goal is not None:
                    self.get_logger().info(metric_line(
                        "servo_pose_tracking_timeout",
                        published_count=self._pose_published_count,
                        servo_status=self._servo_status,
                        tf_failure_count=self._tf_failure_count,
                        tf_success_count=self._tf_success_count,
                    ))
                self._publish_status("aborted", abort_reason="servo_target_timeout")
                self._last_abort_sec = now_sec
                self._lifecycle.clear()
                return
            if (
                target.pose_tracking_goal is not None
                and self._desired_command_type == ServoCommandType.Request.POSE
            ):
                self._control_pose_tracking(now_sec)
                return
            if self._joint_state is None:
                return
            reference = trajectory_reference_at(
                target, self._lifecycle.reference_elapsed_sec
            )
            decision = decide_time_synchronized_joint_jog(
                target, self._joint_state, reference
            )
            if decision is None:
                self._publish_status("aborted", abort_reason="incomplete_joint_state")
                self._lifecycle.clear()
                return
            self._lifecycle.update_reference_clock(
                now_sec=now_sec, progress_scale=decision.progress_scale
            )
            measured_velocities = tuple(
                self._joint_velocities[name]
                for name in target.joint_names
                if name in self._joint_velocities
            )
            if len(measured_velocities) != len(target.joint_names):
                measured_velocities = ()
            stalled = self._stall_detector.update(
                now_sec=now_sec,
                progress_scale=decision.progress_scale,
                velocities_rad_s=measured_velocities,
            )
            self._observe_joint_tracking(
                now_sec, target, reference, decision, stalled
            )
            jog = JointJog()
            jog.header.stamp = self.get_clock().now().to_msg()
            jog.joint_names = list(decision.joint_names)
            jog.velocities = list(decision.velocities_rad_s)
            self._jog_pub.publish(jog)
            self._publish_status(
                "running",
                max_error_rad=decision.max_error_rad,
                progress_scale=decision.progress_scale,
                stall_elapsed_sec=self._stall_detector.elapsed_sec,
                stalled=stalled,
            )
            if not self._lifecycle.record_reached(decision.reached):
                return
            if target.pose_tracking_goal is not None:
                self._desired_command_type = ServoCommandType.Request.POSE
                self._lifecycle.stable_samples = 0
                return
            self.get_logger().info(metric_line(
                "servo_target_succeeded", phase=target.phase,
                max_error_rad=decision.max_error_rad,
                latency_ms=round(
                    (now_sec - (self._lifecycle.started_at_sec or now_sec)) * 1000.0,
                    3,
                ),
            ))
            self._publish_status("succeeded", max_error_rad=decision.max_error_rad)
            self._publish_gripper(self._gripper_gate.terminal_reached(target.gripper_closed))
            self._lifecycle.clear()

        def _observe_joint_tracking(
            self,
            now_sec: float,
            target: ServoTarget,
            reference: TrajectoryReference,
            decision: JointJogDecision,
            stalled: bool,
        ) -> None:
            """同一sequenceで参照・実測・JTC位置を記録する。"""
            if decision.max_error_rad < 0.09:
                return
            if (
                self._last_tracking_observation_sec is not None
                and now_sec - self._last_tracking_observation_sec < 0.1
            ):
                return
            if self._joint_state is None:
                return
            actual_by_name = dict(zip(
                self._joint_state.joint_names,
                self._joint_state.positions_rad,
                strict=True,
            ))
            errors = tuple(
                abs(reference_position - actual_by_name[name])
                for name, reference_position in zip(
                    target.joint_names, reference.positions_rad, strict=True
                )
            )
            limiting_index = max(range(len(errors)), key=errors.__getitem__)
            limiting_joint = target.joint_names[limiting_index]
            signed_error_rad = (
                reference.positions_rad[limiting_index]
                - actual_by_name[limiting_joint]
            )
            feedback_velocity_rad_s = SERVO_JOINT_GAIN * signed_error_rad
            command_velocity_rad_s = decision.velocities_rad_s[limiting_index]
            self._tracking_sequence_id += 1
            self._last_tracking_observation_sec = now_sec
            self.get_logger().info(metric_line(
                "servo_joint_tracking_sample",
                sequence_id=self._tracking_sequence_id,
                target_id=f"{target.phase}:{self._lifecycle.started_at_sec:.6f}",
                limiting_joint=limiting_joint,
                q_ref_rad=round(reference.positions_rad[limiting_index], 6),
                q_actual_rad=round(actual_by_name[limiting_joint], 6),
                q_first_rad=round(
                    target.trajectory_points[0].positions_rad[limiting_index], 6
                ),
                q_jtc_rad=(
                    round(self._last_jtc_positions[limiting_joint], 6)
                    if limiting_joint in self._last_jtc_positions else None
                ),
                q_hardware_rad=(
                    round(self._last_hardware_positions[limiting_joint], 6)
                    if limiting_joint in self._last_hardware_positions else None
                ),
                v_hardware_rad_s=(
                    round(self._last_hardware_velocities[limiting_joint], 6)
                    if limiting_joint in self._last_hardware_velocities else None
                ),
                v_cmd_rad_s=round(decision.velocities_rad_s[limiting_index], 6),
                v_ref_rad_s=round(reference.velocities_rad_s[limiting_index], 6),
                v_feedback_rad_s=round(feedback_velocity_rad_s, 6),
                progress_scale=round(decision.progress_scale, 6),
                stall_elapsed_sec=round(self._stall_detector.elapsed_sec, 6),
                stalled=stalled,
                error_reduction_metric=round(
                    signed_error_rad * command_velocity_rad_s, 6
                ),
                reference_elapsed_sec=round(
                    self._lifecycle.reference_elapsed_sec, 6
                ),
                max_error_rad=decision.max_error_rad,
            ))

        def _control_pose_tracking(self, now_sec: float) -> None:
            """MOVING_TO_GRASP終端をServo pose commandで閉ループ追従する。"""
            target = self._lifecycle.target
            if target is None or target.pose_tracking_goal is None:
                return
            goal = target.pose_tracking_goal
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
                return
            self._tf_success_count += 1
            self._last_tf_success_sec = now_sec
            current_pose = pose_from_transform(transform)
            decision = decide_pose_tracking(target, current_pose)
            if decision is None:
                return
            self._publish_status("running")
            next_stable_samples = self._lifecycle.stable_samples + 1 if decision.reached else 0
            self.get_logger().info(metric_line(
                "servo_pose_tracking_sample",
                **pose_tracking_metric_fields(
                    sequence_id=self._pose_sequence_id,
                    published_count=self._pose_published_count,
                    planning_frame=SERVO_PLANNING_FRAME,
                    end_effector_frame=SERVO_END_EFFECTOR_FRAME,
                    pose_source="tf",
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
            if not self._lifecycle.record_reached(decision.reached):
                return
            self.get_logger().info(metric_line(
                "servo_pose_target_succeeded", phase=target.phase,
                position_error_m=decision.position_error_m,
                orientation_error_rad=decision.orientation_error_rad,
                latency_ms=round(
                    (now_sec - (self._lifecycle.started_at_sec or now_sec)) * 1000.0, 3
                ),
            ))
            self._publish_gripper(self._gripper_gate.terminal_reached(target.gripper_closed))
            self._publish_status("succeeded")
            self._lifecycle.clear()

        def _publish_gripper(self, closed: bool | None) -> None:
            if closed is None:
                return
            msg = String()
            msg.data = "true" if closed else "false"
            self._gripper_pub.publish(msg)

        def _publish_status(
            self,
            status: str,
            *,
            max_error_rad: float | None = None,
            abort_reason: str | None = None,
            progress_scale: float | None = None,
            stall_elapsed_sec: float | None = None,
            stalled: bool | None = None,
        ) -> None:
            msg = String()
            msg.data = execution_status_payload(
                status,
                max_error_rad=max_error_rad,
                abort_reason=abort_reason,
                progress_scale=progress_scale,
                stall_elapsed_sec=stall_elapsed_sec,
                stalled=stalled,
            )
            self._status_pub.publish(msg)
            self.get_logger().info(f"execution_status {msg.data}")

    node = ServoExecutionAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
