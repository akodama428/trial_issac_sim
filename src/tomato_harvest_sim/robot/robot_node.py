"""tomato_harvest_robot_node — 独立 ROS2 ノード。

HarvestRuntime を 30 Hz タイマーで駆動し、
ROS2 topic / action でシミュレータノードおよび franka_ros2_control と通信する。
"""
from __future__ import annotations

import json

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from tomato_harvest_sim.api.bridge import (
    CONTROL_TOPIC,
    MOTION_COMMAND_TOPIC,
    MOTION_METADATA_TOPIC,
    SCENE_SNAPSHOT_TOPIC,
    TARGET_ESTIMATE_TOPIC,
    _camera_spec_from_snapshot,
    _joint_state_from_snapshot,
    _motion_command_to_dict,
    _scene_snapshot_from_dict,
    _target_estimate_to_dict,
    _tf_tree_from_snapshot,
)
from tomato_harvest_sim.api.contracts import (
    CameraFrame,
    ControlCommand,
    JointStateSnapshot,
    Pose3D,
    RobotRuntimeState,
    SceneSnapshot,
    TfTreeSnapshot,
)
from tomato_harvest_sim.robot.api.trajectory_tracking import ObservationData
from tomato_harvest_sim.robot.runtime import HarvestRuntime
from tomato_harvest_sim.robot.trajectory_tracking import TrajectoryTrackingCoordinator
from tomato_harvest_sim.robot.trajectory_tracking.ros2_action_trajectory_port import Ros2ActionTrajectoryPort
from tomato_harvest_sim.simulator.ros2_joint_state_hardware_port import Ros2JointStateHardwarePort


_HOME_JOINT_POSITIONS = np.array([0.0, -0.4, 0.0, -2.1, 0.0, 1.7, 0.8])


class RobotNodeDriver:
    """FrankaExecutionDriverProtocol のスタブ実装。

    独立ロボットノードでは Isaac Sim API を使えないため、IK・直接ドライブは no-op とし、
    軌道展開はパススルー（アーム 7 関節のみ）、ホーム位置は固定値を返す。
    """

    def initialize_if_needed(self) -> bool:
        return True

    def get_observation(self) -> ObservationData:
        return ObservationData(
            joint_positions=None,
            joint_velocities=None,
            end_effector_pose=None,
            joint_state_snapshot=None,
        )

    def current_joint_positions(self) -> np.ndarray | None:
        return None

    def current_joint_velocities(self) -> np.ndarray | None:
        return None

    def current_end_effector_pose(self) -> Pose3D | None:
        return None

    def current_joint_state_snapshot(self) -> JointStateSnapshot | None:
        return None

    def home_joint_positions(self) -> np.ndarray:
        return _HOME_JOINT_POSITIONS.copy()

    def expand_joint_targets(self, joint_positions: np.ndarray) -> np.ndarray:
        return joint_positions

    def solve_joint_targets_for_pose(
        self, target_pose: Pose3D, *, position_tolerance_m: float
    ) -> np.ndarray | None:
        return None

    def set_joint_positions_with_debug(self, positions: np.ndarray, *, context: str) -> None:
        pass

    def set_joint_velocity_targets_with_debug(
        self,
        *,
        positions: np.ndarray | None,
        velocities: np.ndarray,
        context: str,
    ) -> None:
        pass


class RobotNodeBridge:
    """BridgeProtocol の ROS2 topic 実装。

    RobotNode のサブスクリプションキャッシュから scene_snapshot / joint_state / TF を読み取り、
    motion_command / target_estimate を ROS2 topic へ publish する。
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._last_scene_snapshot: SceneSnapshot | None = None
        self._last_joint_state: JointStateSnapshot | None = None
        self._last_tf_tree: TfTreeSnapshot | None = None
        self._pending_control: ControlCommand | None = None

        node.create_publisher(String, CONTROL_TOPIC, 10)  # ユーザー UI 用（現在は未使用）
        self._motion_command_pub = node.create_publisher(String, MOTION_COMMAND_TOPIC, 10)
        self._motion_metadata_pub = node.create_publisher(String, MOTION_METADATA_TOPIC, 10)
        self._target_estimate_pub = node.create_publisher(String, TARGET_ESTIMATE_TOPIC, 10)

        node.create_subscription(String, SCENE_SNAPSHOT_TOPIC, self._on_scene_snapshot, 10)
        node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        node.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        node.create_subscription(String, CONTROL_TOPIC, self._on_control, 10)

    def _on_scene_snapshot(self, msg: String) -> None:
        self._last_scene_snapshot = _scene_snapshot_from_dict(json.loads(msg.data))
        self._last_tf_tree = _tf_tree_from_snapshot(self._last_scene_snapshot)

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state = JointStateSnapshot(
            joint_names=tuple(str(n) for n in msg.name),
            positions_rad=tuple(float(v) for v in msg.position),
        )

    def _on_tf(self, _msg: TFMessage) -> None:
        if self._last_scene_snapshot is not None:
            self._last_tf_tree = _tf_tree_from_snapshot(self._last_scene_snapshot)

    def _on_control(self, msg: String) -> None:
        try:
            self._pending_control = ControlCommand(msg.data.strip().lower())
        except ValueError:
            self._node.get_logger().warning(f"Unknown control command: {msg.data!r}")

    def publish_control(self, command: ControlCommand) -> None:
        msg = String()
        msg.data = command.value
        self._node.create_publisher(String, CONTROL_TOPIC, 10).publish(msg)

    def consume_control_command(self) -> ControlCommand | None:
        command = self._pending_control
        self._pending_control = None
        return command

    def publish_scene_snapshot(self, snapshot: SceneSnapshot) -> None:
        pass  # robot ノードは scene_snapshot を publish しない

    def publish_joint_state(self, joint_state: JointStateSnapshot) -> None:
        pass  # hardware port が /joint_states を担う

    def read_scene_snapshot(self) -> SceneSnapshot:
        if self._last_scene_snapshot is None:
            raise RuntimeError("scene_snapshot 未受信")
        return self._last_scene_snapshot

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
        if self._last_joint_state is not None:
            return self._last_joint_state
        return _joint_state_from_snapshot(self._last_scene_snapshot)

    def read_tf_tree(self) -> TfTreeSnapshot:
        if self._last_tf_tree is not None:
            return self._last_tf_tree
        return _tf_tree_from_snapshot(self.read_scene_snapshot())

    def publish_target_estimate(self, estimate: object) -> None:
        msg = String()
        msg.data = json.dumps(_target_estimate_to_dict(estimate))
        self._target_estimate_pub.publish(msg)

    def publish_motion_command(self, command: object) -> None:
        metadata_msg = String()
        metadata_msg.data = json.dumps(_motion_command_to_dict(command, include_trajectory=False))
        self._motion_metadata_pub.publish(metadata_msg)

        command_msg = String()
        command_msg.data = json.dumps(_motion_command_to_dict(command, include_trajectory=True))
        self._motion_command_pub.publish(command_msg)

    def consume_motion_command(self) -> None:
        return None

    def publish_controller_state(self, state: object) -> None:
        pass

    def spin_once(self) -> None:
        pass  # ROS2 executor が管理

    def close(self) -> None:
        pass


class RobotNode(Node):
    """tomato_harvest_robot_node。

    30 Hz タイマーで HarvestRuntime を駆動し、
    ROS2 topic / action でシミュレータノードおよび franka_ros2_control と通信する。
    """

    TIMER_HZ = 30

    def __init__(self) -> None:
        super().__init__("tomato_harvest_robot_node")

        hw_port = Ros2JointStateHardwarePort(driver=None, external_node=self)
        traj_port = Ros2ActionTrajectoryPort()
        driver = RobotNodeDriver()

        coordinator = TrajectoryTrackingCoordinator(
            driver=driver,
            hardware_control_port=hw_port,
            trajectory_execution_port=traj_port,
            allow_direct_drive=False,
        )

        self._bridge = RobotNodeBridge(self)
        self._runtime = HarvestRuntime(executor=coordinator)
        self.create_timer(1.0 / self.TIMER_HZ, self._step)
        self.get_logger().info("tomato_harvest_robot_node 起動")

    def _step(self) -> None:
        snapshot = self._bridge._last_scene_snapshot
        if snapshot is None:
            return

        if self._runtime.state.runtime_state is RobotRuntimeState.BOOTING:
            self._runtime.boot()

        self._runtime.observe_scene(snapshot)

        control = self._bridge.consume_control_command()
        if control is not None:
            self._runtime.apply_control(control)

        logs = self._runtime.step(self._bridge)
        for log in logs:
            self.get_logger().debug(log)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
