"""ROS2 ブリッジ。simulator / robot 境界のメッセージ輸送を担う。

api/bridge.py から移設。import を msg.contracts に統一済み。
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import Protocol

from tomato_harvest_sim.msg.contracts import (
    CameraFrame,
    ControlCommand,
    PhaseId,
    PhaseMotionPlan,
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
    MotionCommand,
    Pose3D,
    ScenePhase,
    SceneSnapshot,
    TargetEstimate,
    TfTreeSnapshot,
    TomatoStatus,
)
from tomato_harvest_sim.msg.topics import (
    CONTROL_TOPIC,
    FIXED_CAMERA_TOPIC,
    FOLLOW_JOINT_TRAJECTORY_ACTION,
    HAND_CAMERA_TOPIC,
    MOTION_COMMAND_TOPIC,
    MOTION_METADATA_TOPIC,
    SCENE_SNAPSHOT_TOPIC,
    TARGET_ESTIMATE_TOPIC,
    DEFAULT_JOINT_NAMES,
    DEFAULT_JOINT_POSITIONS_RAD,
)

__all__ = [
    "BridgeProtocol",
    "BridgeState",
    "InMemoryRos2Bridge",
    "Ros2LoopbackBridge",
    "create_bridge",
    "CONTROL_TOPIC",
    "FIXED_CAMERA_TOPIC",
    "HAND_CAMERA_TOPIC",
    "SCENE_SNAPSHOT_TOPIC",
    "MOTION_COMMAND_TOPIC",
    "TARGET_ESTIMATE_TOPIC",
    "DEFAULT_JOINT_NAMES",
    "DEFAULT_JOINT_POSITIONS_RAD",
]


class BridgeProtocol(Protocol):
    def publish_control(self, command: ControlCommand) -> None: ...

    def consume_control_command(self) -> ControlCommand | None: ...

    def publish_scene_snapshot(self, snapshot: SceneSnapshot) -> None: ...

    def publish_joint_state(self, joint_state: JointStateSnapshot) -> None: ...

    def read_scene_snapshot(self) -> SceneSnapshot: ...

    def read_camera_frame(self, camera_name: str) -> CameraFrame: ...

    def read_joint_state(self) -> JointStateSnapshot: ...

    def read_tf_tree(self) -> TfTreeSnapshot: ...

    def publish_target_estimate(self, estimate: TargetEstimate) -> None: ...

    def publish_motion_command(self, command: MotionCommand) -> None: ...

    def consume_motion_command(self) -> MotionCommand | None: ...

    def publish_controller_state(self, state: object) -> None: ...

    def spin_once(self) -> None: ...

    def close(self) -> None: ...


@dataclass
class BridgeState:
    last_command: ControlCommand | None = None
    pending_control_command: ControlCommand | None = None
    last_scene_snapshot: SceneSnapshot | None = None
    last_target_estimate: TargetEstimate | None = None
    last_motion_command: MotionCommand | None = None
    last_motion_metadata: MotionCommand | None = None
    pending_motion_command: MotionCommand | None = None
    last_joint_state: JointStateSnapshot | None = None
    last_tf_tree: TfTreeSnapshot | None = None


class InMemoryRos2Bridge:
    """Fallback bridge that keeps the simulator/robot boundary explicit."""

    def __init__(self) -> None:
        self.state = BridgeState()

    def publish_control(self, command: ControlCommand) -> None:
        self.state.last_command = command
        self.state.pending_control_command = command

    def consume_control_command(self) -> ControlCommand | None:
        command = self.state.pending_control_command
        self.state.pending_control_command = None
        return command

    def publish_scene_snapshot(self, snapshot: SceneSnapshot) -> None:
        self.state.last_scene_snapshot = snapshot

    def publish_joint_state(self, joint_state: JointStateSnapshot) -> None:
        self.state.last_joint_state = joint_state

    def read_scene_snapshot(self) -> SceneSnapshot:
        return self._require_scene_snapshot()

    def read_camera_frame(self, camera_name: str) -> CameraFrame:
        snapshot = self._require_scene_snapshot()
        camera_pose, topic_name, frame_id = _camera_spec_from_snapshot(snapshot, camera_name)
        return CameraFrame(
            camera_name=camera_name,
            topic_name=topic_name,
            frame_id=frame_id,
            camera_pose=camera_pose,
            target_world_pose=snapshot.tomato_pose,
        )

    def read_joint_state(self) -> JointStateSnapshot:
        if self.state.last_joint_state is not None:
            return self.state.last_joint_state
        snapshot = self.state.last_scene_snapshot
        joint_state = _joint_state_from_snapshot(snapshot)
        self.state.last_joint_state = joint_state
        return joint_state

    def read_tf_tree(self) -> TfTreeSnapshot:
        if self.state.last_tf_tree is not None:
            return self.state.last_tf_tree
        snapshot = self._require_scene_snapshot()
        tf_tree = _tf_tree_from_snapshot(snapshot)
        self.state.last_tf_tree = tf_tree
        return tf_tree

    def publish_target_estimate(self, estimate: TargetEstimate) -> None:
        self.state.last_target_estimate = estimate

    def publish_motion_command(self, command: MotionCommand) -> None:
        self.state.last_motion_command = command
        self.state.pending_motion_command = command

    def consume_motion_command(self) -> MotionCommand | None:
        command = self.state.pending_motion_command
        self.state.pending_motion_command = None
        return command

    def publish_controller_state(self, state: object) -> None:
        return None

    def spin_once(self) -> None:
        return None

    def close(self) -> None:
        return None

    def _require_scene_snapshot(self) -> SceneSnapshot:
        if self.state.last_scene_snapshot is None:
            raise RuntimeError("Scene snapshot is not available in bridge state.")
        return self.state.last_scene_snapshot


class Ros2LoopbackBridge:
    """Real ROS 2 transport for the robot/simulator boundary inside one process."""

    def __init__(self) -> None:
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient, ActionServer
        from rclpy.action.server import GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.node import Node
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        self.state = BridgeState()
        self._rclpy = rclpy
        self._initialized_here = False
        self._spin_iterations = max(1, int(os.environ.get("TOMATO_HARVEST_ROS2_SPIN_ITERATIONS", "1")))
        self._spin_timeout_sec = max(0.0, float(os.environ.get("TOMATO_HARVEST_ROS2_SPIN_TIMEOUT_SEC", "0.001")))
        self._action_wait_timeout_sec = max(
            0.0,
            float(os.environ.get("TOMATO_HARVEST_ROS2_ACTION_WAIT_TIMEOUT_SEC", "0.05")),
        )
        self._action_goal_timeout_sec = max(
            0.0,
            float(os.environ.get("TOMATO_HARVEST_ROS2_ACTION_GOAL_TIMEOUT_SEC", "0.10")),
        )
        self._trajectory_server_ready = False
        if not self._rclpy.ok():
            self._rclpy.init(args=None)
            self._initialized_here = True

        self._callback_group = ReentrantCallbackGroup()
        self._sim_node: Node = self._rclpy.create_node("tomato_harvest_sim_bridge")
        self._robot_node: Node = self._rclpy.create_node("tomato_harvest_robot_bridge")

        self._control_publisher = self._robot_node.create_publisher(String, CONTROL_TOPIC, 10)
        self._scene_snapshot_publisher = self._sim_node.create_publisher(String, SCENE_SNAPSHOT_TOPIC, 10)
        self._motion_command_publisher = self._robot_node.create_publisher(String, MOTION_COMMAND_TOPIC, 10)
        self._motion_metadata_publisher = self._robot_node.create_publisher(String, MOTION_METADATA_TOPIC, 10)
        self._target_estimate_publisher = self._robot_node.create_publisher(String, TARGET_ESTIMATE_TOPIC, 10)
        self._fixed_camera_publisher = self._sim_node.create_publisher(Image, FIXED_CAMERA_TOPIC, 10)
        self._hand_camera_publisher = self._sim_node.create_publisher(Image, HAND_CAMERA_TOPIC, 10)
        self._joint_state_publisher = self._sim_node.create_publisher(JointState, "/joint_states", 10)
        self._joint_state_desired_publisher = self._sim_node.create_publisher(JointState, "/joint_states_desired", 10)
        self._tf_publisher = self._sim_node.create_publisher(TFMessage, "/tf", 10)

        self._sim_node.create_subscription(String, CONTROL_TOPIC, self._on_control_message, 10)
        self._robot_node.create_subscription(String, SCENE_SNAPSHOT_TOPIC, self._on_scene_snapshot_message, 10)
        self._sim_node.create_subscription(String, MOTION_COMMAND_TOPIC, self._on_motion_command_message, 10)
        self._sim_node.create_subscription(String, MOTION_METADATA_TOPIC, self._on_motion_metadata_message, 10)
        self._robot_node.create_subscription(JointState, "/joint_states", self._on_joint_state_message, 10)
        self._robot_node.create_subscription(TFMessage, "/tf", self._on_tf_message, 10)

        self._trajectory_action_client = ActionClient(
            self._robot_node,
            FollowJointTrajectory,
            FOLLOW_JOINT_TRAJECTORY_ACTION,
            callback_group=self._callback_group,
        )
        self._trajectory_action_server = ActionServer(
            self._sim_node,
            FollowJointTrajectory,
            FOLLOW_JOINT_TRAJECTORY_ACTION,
            execute_callback=self._execute_trajectory_goal,
            goal_callback=lambda goal_request: GoalResponse.ACCEPT,
            callback_group=self._callback_group,
        )

    def publish_control(self, command: ControlCommand) -> None:
        from std_msgs.msg import String

        self.state.last_command = command
        message = String()
        message.data = command.value
        self._control_publisher.publish(message)
        self.spin_once()

    def consume_control_command(self) -> ControlCommand | None:
        self.spin_once()
        command = self.state.pending_control_command
        self.state.pending_control_command = None
        return command

    def publish_scene_snapshot(self, snapshot: SceneSnapshot) -> None:
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage

        self.state.last_scene_snapshot = snapshot

        scene_message = String()
        scene_message.data = json.dumps(_scene_snapshot_to_dict(snapshot))
        self._scene_snapshot_publisher.publish(scene_message)

        fixed_image = _build_image_message(Image, frame_id="fixed_camera_frame")
        hand_image = _build_image_message(Image, frame_id="hand_camera_frame")
        self._fixed_camera_publisher.publish(fixed_image)
        self._hand_camera_publisher.publish(hand_image)

        joint_state = self.state.last_joint_state or _joint_state_from_snapshot(snapshot)
        joint_message = JointState()
        joint_message.name = list(joint_state.joint_names)
        joint_message.position = [float(value) for value in joint_state.positions_rad]
        self._joint_state_publisher.publish(joint_message)

        tf_message = _build_tf_message(TFMessage, snapshot)
        self._tf_publisher.publish(tf_message)
        self.spin_once()

    def publish_joint_state(self, joint_state: JointStateSnapshot) -> None:
        from sensor_msgs.msg import JointState

        self.state.last_joint_state = joint_state
        joint_message = JointState()
        joint_message.name = list(joint_state.joint_names)
        joint_message.position = [float(value) for value in joint_state.positions_rad]
        self._joint_state_publisher.publish(joint_message)

    def publish_controller_state(self, state: object) -> None:
        from sensor_msgs.msg import JointState

        desired = getattr(state, "desired_positions_rad", None)
        actual = getattr(state, "actual_positions_rad", None)
        if desired is None or actual is None:
            return

        n = len(desired)
        joint_names = [f"panda_joint{i + 1}" for i in range(n)]

        desired_msg = JointState()
        desired_msg.name = joint_names
        desired_msg.position = [float(v) for v in desired]
        desired_msg.velocity = [float(v) for v in getattr(state, "desired_velocities_rad_s", [0.0] * n)]
        self._joint_state_desired_publisher.publish(desired_msg)

        actual_msg = JointState()
        actual_msg.name = joint_names
        actual_msg.position = [float(v) for v in actual]
        actual_msg.velocity = [float(v) for v in getattr(state, "actual_velocities_rad_s", [0.0] * n)]
        self._joint_state_publisher.publish(actual_msg)
        self.spin_once()

    def read_scene_snapshot(self) -> SceneSnapshot:
        self.spin_once()
        if self.state.last_scene_snapshot is None:
            raise RuntimeError("Scene snapshot is not available on the ROS 2 bridge.")
        return self.state.last_scene_snapshot

    def read_camera_frame(self, camera_name: str) -> CameraFrame:
        snapshot = self.read_scene_snapshot()
        camera_pose, topic_name, frame_id = _camera_spec_from_snapshot(snapshot, camera_name)
        return CameraFrame(
            camera_name=camera_name,
            topic_name=topic_name,
            frame_id=frame_id,
            camera_pose=camera_pose,
            target_world_pose=snapshot.tomato_pose,
        )

    def read_joint_state(self) -> JointStateSnapshot:
        self.spin_once()
        if self.state.last_joint_state is not None:
            return self.state.last_joint_state
        snapshot = self.state.last_scene_snapshot
        joint_state = _joint_state_from_snapshot(snapshot)
        self.state.last_joint_state = joint_state
        return joint_state

    def read_tf_tree(self) -> TfTreeSnapshot:
        self.spin_once()
        if self.state.last_tf_tree is not None:
            return self.state.last_tf_tree
        snapshot = self.read_scene_snapshot()
        tf_tree = _tf_tree_from_snapshot(snapshot)
        self.state.last_tf_tree = tf_tree
        return tf_tree

    def publish_target_estimate(self, estimate: TargetEstimate) -> None:
        from std_msgs.msg import String

        self.state.last_target_estimate = estimate
        message = String()
        message.data = json.dumps(_target_estimate_to_dict(estimate))
        self._target_estimate_publisher.publish(message)
        self.spin_once()

    def publish_motion_command(self, command: MotionCommand) -> None:
        from control_msgs.action import FollowJointTrajectory
        from std_msgs.msg import String

        self.state.last_motion_command = command

        metadata_message = String()
        metadata_message.data = json.dumps(_motion_command_to_dict(command))
        self._motion_metadata_publisher.publish(metadata_message)

        joint_trajectory = command.phase_motion_plan.joint_trajectory if command.phase_motion_plan is not None else None

        if joint_trajectory is None:
            command_message = String()
            command_message.data = json.dumps(_motion_command_to_dict(command))
            self._motion_command_publisher.publish(command_message)
            self.spin_once()
            return

        if not self._ensure_trajectory_server_ready():
            command_message = String()
            command_message.data = json.dumps(_motion_command_to_dict(command))
            self._motion_command_publisher.publish(command_message)
            self.spin_once()
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_trajectory.joint_names)
        for point in joint_trajectory.points:
            trajectory_point = _ros_trajectory_point_from_contract(point)
            goal.trajectory.points.append(trajectory_point)
        future = self._trajectory_action_client.send_goal_async(goal)
        self._spin_until_future_complete(self._robot_node, future, timeout_sec=self._action_goal_timeout_sec)
        self.spin_once()

    def consume_motion_command(self) -> MotionCommand | None:
        self.spin_once()
        command = self.state.pending_motion_command
        self.state.pending_motion_command = None
        return command

    def spin_once(self) -> None:
        for _ in range(self._spin_iterations):
            self._rclpy.spin_once(self._sim_node, timeout_sec=self._spin_timeout_sec)
            self._rclpy.spin_once(self._robot_node, timeout_sec=self._spin_timeout_sec)

    def close(self) -> None:
        try:
            self._trajectory_action_server.destroy()
        except Exception:
            pass
        try:
            self._trajectory_action_client.destroy()
        except Exception:
            pass
        self._sim_node.destroy_node()
        self._robot_node.destroy_node()
        if self._initialized_here and self._rclpy.ok():
            self._rclpy.shutdown()

    def _on_control_message(self, message: object) -> None:
        data = str(getattr(message, "data", "")).strip().lower()
        if not data:
            return
        self.state.pending_control_command = ControlCommand(data)

    def _on_scene_snapshot_message(self, message: object) -> None:
        payload = json.loads(str(getattr(message, "data", "{}")))
        snapshot = _scene_snapshot_from_dict(payload)
        self.state.last_scene_snapshot = snapshot
        self.state.last_tf_tree = _tf_tree_from_snapshot(snapshot)

    def _on_joint_state_message(self, message: object) -> None:
        self.state.last_joint_state = JointStateSnapshot(
            joint_names=tuple(str(name) for name in getattr(message, "name", ())),
            positions_rad=tuple(float(value) for value in getattr(message, "position", ())),
        )

    def _on_tf_message(self, _message: object) -> None:
        if self.state.last_scene_snapshot is None:
            return
        self.state.last_tf_tree = _tf_tree_from_snapshot(self.state.last_scene_snapshot)

    def _on_motion_metadata_message(self, message: object) -> None:
        payload = json.loads(str(getattr(message, "data", "{}")))
        self.state.last_motion_metadata = _motion_command_from_dict(payload)

    def _on_motion_command_message(self, message: object) -> None:
        payload = json.loads(str(getattr(message, "data", "{}")))
        self.state.pending_motion_command = _motion_command_from_dict(payload)

    def _execute_trajectory_goal(self, goal_handle: object) -> object:
        from control_msgs.action import FollowJointTrajectory

        metadata = self.state.last_motion_metadata or MotionCommand(
            command_name="move_to_pregrasp",
            planner_name="ros2_topic_action_fallback",
        )
        trajectory = _joint_trajectory_from_ros_msg(getattr(goal_handle.request, "trajectory", None))
        phase_motion_plan = metadata.phase_motion_plan
        if phase_motion_plan is not None and trajectory is not None:
            phase_motion_plan = PhaseMotionPlan(
                phase_id=phase_motion_plan.phase_id,
                phase_goal_pose=phase_motion_plan.phase_goal_pose,
                active_waypoints=phase_motion_plan.active_waypoints,
                joint_trajectory=trajectory,
            )
        self.state.pending_motion_command = MotionCommand(
            command_name=metadata.command_name,
            planner_name=metadata.planner_name,
            target_pose=metadata.target_pose,
            gripper_closed=metadata.gripper_closed,
            phase_motion_plan=phase_motion_plan,
        )
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        if hasattr(result, "error_code"):
            result.error_code = 0
        if hasattr(result, "error_string"):
            result.error_string = ""
        return result

    def _spin_until_future_complete(self, node: object, future: object, *, timeout_sec: float) -> None:
        self._rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    def _ensure_trajectory_server_ready(self) -> bool:
        if self._trajectory_server_ready:
            return True
        self.spin_once()
        if not self._trajectory_action_client.wait_for_server(timeout_sec=self._action_wait_timeout_sec):
            return False
        self._trajectory_server_ready = True
        return True


def create_bridge(*, transport: str | None = None) -> BridgeProtocol:
    transport = (transport or os.environ.get("TOMATO_HARVEST_TRANSPORT", "auto")).strip().lower()
    if transport == "in_memory":
        return InMemoryRos2Bridge()
    if transport == "ros2":
        return Ros2LoopbackBridge()
    try:
        import rclpy  # noqa: F401
    except Exception:
        return InMemoryRos2Bridge()
    return Ros2LoopbackBridge()


# ---------------------------------------------------------------------------
# Private helpers (available for import by simulator_node.py etc.)
# ---------------------------------------------------------------------------

def _camera_spec_from_snapshot(snapshot: SceneSnapshot, camera_name: str) -> tuple[Pose3D, str, str]:
    if camera_name == "fixed_camera":
        return snapshot.fixed_camera_pose, FIXED_CAMERA_TOPIC, "fixed_camera_frame"
    if camera_name == "hand_camera":
        return snapshot.hand_camera_pose, HAND_CAMERA_TOPIC, "hand_camera_frame"
    raise ValueError(f"Unsupported camera: {camera_name}")


def _joint_state_from_snapshot(snapshot: SceneSnapshot | None) -> JointStateSnapshot:
    plan = snapshot.active_phase_motion_plan if snapshot is not None else None
    if plan is None or plan.joint_trajectory is None or not plan.joint_trajectory.points:
        return JointStateSnapshot(
            joint_names=DEFAULT_JOINT_NAMES,
            positions_rad=DEFAULT_JOINT_POSITIONS_RAD,
        )
    positions = plan.joint_trajectory.points[-1].positions_rad
    return JointStateSnapshot(
        joint_names=plan.joint_trajectory.joint_names,
        positions_rad=positions,
    )


def _tf_tree_from_snapshot(snapshot: SceneSnapshot) -> TfTreeSnapshot:
    return TfTreeSnapshot(
        robot_base_frame_id="panda_link0",
        camera_frame_id="fixed_camera_frame" if snapshot.active_camera == "fixed_camera" else "hand_camera_frame",
        target_frame_id="target_tomato_frame",
        robot_base_pose=snapshot.robot_base_pose,
        camera_pose=snapshot.fixed_camera_pose if snapshot.active_camera == "fixed_camera" else snapshot.hand_camera_pose,
        target_pose=snapshot.tomato_pose,
    )


def _build_image_message(image_type: type[object], *, frame_id: str) -> object:
    image = image_type()
    if hasattr(image, "header"):
        image.header.frame_id = frame_id
    image.height = 16
    image.width = 16
    image.encoding = "rgb8"
    image.is_bigendian = 0
    image.step = 16 * 3
    image.data = bytes(16 * 16 * 3)
    return image


def _build_tf_message(tf_message_type: type[object], snapshot: SceneSnapshot) -> object:
    from geometry_msgs.msg import TransformStamped

    tf_message = tf_message_type()

    fixed = TransformStamped()
    fixed.header.frame_id = "panda_link0"
    fixed.child_frame_id = "fixed_camera_frame"
    _assign_transform(fixed, snapshot.fixed_camera_pose)

    hand = TransformStamped()
    hand.header.frame_id = "panda_hand"
    hand.child_frame_id = "hand_camera_frame"
    _assign_transform(hand, snapshot.hand_camera_pose)

    tomato = TransformStamped()
    tomato.header.frame_id = "panda_link0"
    tomato.child_frame_id = "target_tomato_frame"
    _assign_transform(tomato, snapshot.tomato_pose)

    tf_message.transforms = [fixed, hand, tomato]
    return tf_message


def _assign_transform(transform_stamped: object, pose: Pose3D) -> None:
    transform_stamped.transform.translation.x = float(pose.x)
    transform_stamped.transform.translation.y = float(pose.y)
    transform_stamped.transform.translation.z = float(pose.z)
    quaternion = _quaternion_from_pose(pose)
    transform_stamped.transform.rotation.x = quaternion[0]
    transform_stamped.transform.rotation.y = quaternion[1]
    transform_stamped.transform.rotation.z = quaternion[2]
    transform_stamped.transform.rotation.w = quaternion[3]


def _quaternion_from_pose(pose: Pose3D) -> tuple[float, float, float, float]:
    roll = math.radians(pose.roll)
    pitch = math.radians(pose.pitch)
    yaw = math.radians(pose.yaw)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _ros_trajectory_point_from_contract(point: JointTrajectoryPoint) -> object:
    from builtin_interfaces.msg import Duration
    from trajectory_msgs.msg import JointTrajectoryPoint as RosJointTrajectoryPoint

    ros_point = RosJointTrajectoryPoint()
    ros_point.positions = [float(value) for value in point.positions_rad]
    duration = Duration()
    duration.sec = int(point.time_from_start_sec)
    duration.nanosec = int((point.time_from_start_sec - duration.sec) * 1_000_000_000)
    ros_point.time_from_start = duration
    return ros_point


def _joint_trajectory_from_ros_msg(message: object) -> JointTrajectory | None:
    if message is None:
        return None
    joint_names = tuple(str(name) for name in getattr(message, "joint_names", ()))
    points = []
    for point in getattr(message, "points", ()):
        duration = getattr(point, "time_from_start", None)
        seconds = 0.0
        if duration is not None:
            seconds = float(getattr(duration, "sec", 0)) + float(getattr(duration, "nanosec", 0)) / 1_000_000_000.0
        points.append(
            JointTrajectoryPoint(
                positions_rad=tuple(float(value) for value in getattr(point, "positions", ())),
                time_from_start_sec=seconds,
            )
        )
    if not joint_names or not points:
        return None
    return JointTrajectory(joint_names=joint_names, points=tuple(points))


def _pose_to_dict(pose: Pose3D | None) -> dict[str, float] | None:
    if pose is None:
        return None
    return {"x": pose.x, "y": pose.y, "z": pose.z,
            "roll": pose.roll, "pitch": pose.pitch, "yaw": pose.yaw}


def _pose_from_dict(data: dict[str, object] | None) -> Pose3D | None:
    if data is None:
        return None
    return Pose3D(
        x=float(data["x"]), y=float(data["y"]), z=float(data["z"]),
        roll=float(data["roll"]), pitch=float(data["pitch"]), yaw=float(data["yaw"]),
    )


def _trajectory_to_dict(trajectory: JointTrajectory | None) -> dict[str, object] | None:
    if trajectory is None:
        return None
    return {
        "joint_names": list(trajectory.joint_names),
        "points": [
            {"positions_rad": list(p.positions_rad), "time_from_start_sec": p.time_from_start_sec}
            for p in trajectory.points
        ],
    }


def _trajectory_from_dict(data: dict[str, object] | None) -> JointTrajectory | None:
    if data is None:
        return None
    points = []
    for point in data.get("points", []):
        pd = point if isinstance(point, dict) else {}
        points.append(JointTrajectoryPoint(
            positions_rad=tuple(float(v) for v in pd.get("positions_rad", [])),
            time_from_start_sec=float(pd.get("time_from_start_sec", 0.0)),
        ))
    return JointTrajectory(
        joint_names=tuple(str(n) for n in data.get("joint_names", [])),
        points=tuple(points),
    )


def _phase_motion_plan_to_dict(plan: PhaseMotionPlan | None) -> dict[str, object] | None:
    if plan is None:
        return None
    return {
        "phase_id": plan.phase_id.value,
        "phase_goal_pose": _pose_to_dict(plan.phase_goal_pose),
        "active_waypoints": [_pose_to_dict(p) for p in plan.active_waypoints],
        "joint_trajectory": _trajectory_to_dict(plan.joint_trajectory),
    }


def _phase_motion_plan_from_dict(data: dict[str, object] | None) -> PhaseMotionPlan | None:
    if data is None:
        return None
    waypoints = [
        pose
        for pose in (_pose_from_dict(w if isinstance(w, dict) else None) for w in data.get("active_waypoints", []))
        if pose is not None
    ]
    return PhaseMotionPlan(
        phase_id=PhaseId(str(data["phase_id"])),
        phase_goal_pose=_pose_from_dict(data.get("phase_goal_pose") if isinstance(data.get("phase_goal_pose"), dict) else None),
        active_waypoints=tuple(waypoints),
        joint_trajectory=_trajectory_from_dict(
            data.get("joint_trajectory") if isinstance(data.get("joint_trajectory"), dict) else None
        ),
    )


def _motion_command_to_dict(command: MotionCommand) -> dict[str, object]:
    return {
        "command_name": command.command_name,
        "planner_name": command.planner_name,
        "target_pose": _pose_to_dict(command.target_pose),
        "gripper_closed": command.gripper_closed,
        "phase_motion_plan": _phase_motion_plan_to_dict(command.phase_motion_plan),
        "motion_kind": command.motion_kind.value,
        "terminal_pose_tracking": command.terminal_pose_tracking,
    }


def _motion_command_from_dict(data: dict[str, object]) -> MotionCommand:
    from tomato_harvest_sim.msg.contracts import MotionKind

    return MotionCommand(
        command_name=str(data["command_name"]),
        planner_name=str(data["planner_name"]),
        target_pose=_pose_from_dict(data.get("target_pose")),
        gripper_closed=bool(data["gripper_closed"]) if data.get("gripper_closed") is not None else None,
        phase_motion_plan=_phase_motion_plan_from_dict(
            data.get("phase_motion_plan") if isinstance(data.get("phase_motion_plan"), dict) else None
        ),
        motion_kind=MotionKind(str(data.get("motion_kind", MotionKind.FOLLOW_TRAJECTORY.value))),
        terminal_pose_tracking=bool(data.get("terminal_pose_tracking", False)),
    )


def _target_estimate_to_dict(estimate: TargetEstimate) -> dict[str, object]:
    return {
        "camera_name": estimate.camera_name,
        "target_world_pose": _pose_to_dict(estimate.target_world_pose),
        "target_camera_pose": _pose_to_dict(estimate.target_camera_pose),
        "confidence": estimate.confidence,
    }


def _scene_snapshot_to_dict(snapshot: SceneSnapshot) -> dict[str, object]:
    return {
        "phase": snapshot.phase.value,
        "active_camera": snapshot.active_camera,
        "tomato_attached": snapshot.tomato_attached,
        "tomato_status": snapshot.tomato_status.value,
        "gripper_closed": snapshot.gripper_closed,
        "robot_home": snapshot.robot_home,
        "cycle_id": snapshot.cycle_id,
        "robot_model": snapshot.robot_model,
        "robot_base_pose": _pose_to_dict(snapshot.robot_base_pose),
        "fixed_camera_pose": _pose_to_dict(snapshot.fixed_camera_pose),
        "hand_camera_pose": _pose_to_dict(snapshot.hand_camera_pose),
        "branch_pose": _pose_to_dict(snapshot.branch_pose),
        "stem_pose": _pose_to_dict(snapshot.stem_pose),
        "tomato_pose": _pose_to_dict(snapshot.tomato_pose),
        "tray_pose": _pose_to_dict(snapshot.tray_pose),
        "robot_tool_pose": _pose_to_dict(snapshot.robot_tool_pose),
        "target_tool_pose": _pose_to_dict(snapshot.target_tool_pose),
        "grasp_result_reason": snapshot.grasp_result_reason,
        "active_phase_motion_plan": _phase_motion_plan_to_dict(snapshot.active_phase_motion_plan),
    }


def _scene_snapshot_from_dict(data: dict[str, object]) -> SceneSnapshot:
    _zero = Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return SceneSnapshot(
        phase=ScenePhase(str(data["phase"])),
        active_camera=str(data["active_camera"]),
        tomato_attached=bool(data["tomato_attached"]),
        tomato_status=TomatoStatus(str(data["tomato_status"])),
        gripper_closed=bool(data["gripper_closed"]),
        robot_home=bool(data["robot_home"]),
        cycle_id=int(data["cycle_id"]),
        robot_model=str(data["robot_model"]),
        robot_base_pose=_pose_from_dict(data.get("robot_base_pose")) or _zero,
        fixed_camera_pose=_pose_from_dict(data.get("fixed_camera_pose")) or _zero,
        hand_camera_pose=_pose_from_dict(data.get("hand_camera_pose")) or _zero,
        branch_pose=_pose_from_dict(data.get("branch_pose")) or _zero,
        stem_pose=_pose_from_dict(data.get("stem_pose")) or _zero,
        tomato_pose=_pose_from_dict(data.get("tomato_pose")) or _zero,
        tray_pose=_pose_from_dict(data.get("tray_pose")) or _zero,
        robot_tool_pose=_pose_from_dict(data.get("robot_tool_pose")) or _zero,
        target_tool_pose=_pose_from_dict(data.get("target_tool_pose")),
        grasp_result_reason=str(data["grasp_result_reason"]) if data.get("grasp_result_reason") is not None else None,
        active_phase_motion_plan=_phase_motion_plan_from_dict(
            data.get("active_phase_motion_plan") if isinstance(data.get("active_phase_motion_plan"), dict) else None
        ),
    )
