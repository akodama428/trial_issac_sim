"""tomato_harvest_simulator_node — Isaac Sim プロセス内で動作する ROS2 ノード。

Isaac Sim の物理ステップに同期して tick() を呼ばれ、
シーン状態を ROS2 topic へ publish し、ロボットノードからの命令を受け取る。

使い方（Isaac Sim スクリプトから）:

    import rclpy
    from tomato_harvest_sim.simulator.simulator_node import SimulatorNode
    from tomato_harvest_sim.simulator.scene_runtime import IsaacSceneRuntime

    rclpy.init()
    scene_runtime = IsaacSceneRuntime()
    node = SimulatorNode(scene_runtime)
    snapshot = node.boot()

    # Isaac Sim 物理ループ内で:
    while simulation_app.is_running():
        simulation_app.update()
        joint_state = driver.current_joint_state_snapshot()
        snapshot = node.tick(joint_state=joint_state)
"""
from __future__ import annotations

import json
from typing import TYPE_CHECKING

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from tomato_harvest_sim.api.bridge import (
    CONTROL_TOPIC,
    FIXED_CAMERA_TOPIC,
    HAND_CAMERA_TOPIC,
    MOTION_COMMAND_TOPIC,
    MOTION_METADATA_TOPIC,
    SCENE_SNAPSHOT_TOPIC,
    _build_image_message,
    _build_tf_message,
    _motion_command_from_dict,
    _scene_snapshot_to_dict,
)
from tomato_harvest_sim.api.contracts import (
    ControlCommand,
    JointStateSnapshot,
    MotionCommand,
    SceneSnapshot,
)

if TYPE_CHECKING:
    from tomato_harvest_sim.simulator.scene_runtime import IsaacSceneRuntime


class SimulatorNode(Node):
    """tomato_harvest_simulator_node。

    IsaacSceneRuntime を包み、シーン状態を ROS2 topic 経由でロボットノードへ配信する。
    Isaac Sim の物理ステップごとに tick() を呼ぶこと。
    """

    def __init__(self, scene_runtime: IsaacSceneRuntime) -> None:
        super().__init__("tomato_harvest_simulator_node")

        self._scene_runtime = scene_runtime
        self._pending_control: ControlCommand | None = None
        self._pending_motion_command: MotionCommand | None = None

        self._scene_snapshot_pub = self.create_publisher(String, SCENE_SNAPSHOT_TOPIC, 10)
        self._fixed_camera_pub = self.create_publisher(Image, FIXED_CAMERA_TOPIC, 10)
        self._hand_camera_pub = self.create_publisher(Image, HAND_CAMERA_TOPIC, 10)
        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._tf_pub = self.create_publisher(TFMessage, "/tf", 10)

        self.create_subscription(String, CONTROL_TOPIC, self._on_control, 10)
        self.create_subscription(String, MOTION_COMMAND_TOPIC, self._on_motion_command, 10)
        self.create_subscription(String, MOTION_METADATA_TOPIC, self._on_motion_metadata, 10)

        self.get_logger().info("tomato_harvest_simulator_node 起動")

    def boot(self) -> SceneSnapshot:
        """シーンを初期化して最初のスナップショットを publish する。"""
        snapshot = self._scene_runtime.boot()
        self._publish_snapshot(snapshot)
        return snapshot

    def tick(self, joint_state: JointStateSnapshot | None = None) -> SceneSnapshot:
        """Isaac Sim 物理ステップ後に呼ぶ。

        ROS2 コールバックを処理し、受信コマンドをシーンへ適用して
        最新の SceneSnapshot を publish する。

        Args:
            joint_state: Isaac Sim から取得した最新の関節状態。
                         指定された場合 /joint_states へ publish する。

        Returns:
            処理後のシーン状態。
        """
        rclpy.spin_once(self, timeout_sec=0.0)

        if self._pending_control is not None:
            self._scene_runtime.apply_control(self._pending_control)
            self._pending_control = None

        if self._pending_motion_command is not None:
            snapshot = self._scene_runtime.apply_motion_command(self._pending_motion_command)
            self._pending_motion_command = None
        else:
            snapshot = self._scene_runtime.snapshot()

        if joint_state is not None:
            self._publish_joint_state(joint_state)

        self._publish_snapshot(snapshot)
        return snapshot

    def _on_control(self, msg: String) -> None:
        try:
            self._pending_control = ControlCommand(msg.data.strip().lower())
        except ValueError:
            self.get_logger().warning(f"Unknown control command: {msg.data!r}")

    def _on_motion_command(self, msg: String) -> None:
        payload = json.loads(msg.data)
        self._pending_motion_command = _motion_command_from_dict(payload)

    def _on_motion_metadata(self, _msg: String) -> None:
        pass

    def _publish_snapshot(self, snapshot: SceneSnapshot) -> None:
        scene_msg = String()
        scene_msg.data = json.dumps(_scene_snapshot_to_dict(snapshot))
        self._scene_snapshot_pub.publish(scene_msg)

        self._fixed_camera_pub.publish(_build_image_message(Image, frame_id="fixed_camera_frame"))
        self._hand_camera_pub.publish(_build_image_message(Image, frame_id="hand_camera_frame"))
        self._tf_pub.publish(_build_tf_message(TFMessage, snapshot))

    def _publish_joint_state(self, joint_state: JointStateSnapshot) -> None:
        msg = JointState()
        msg.name = list(joint_state.joint_names)
        msg.position = [float(v) for v in joint_state.positions_rad]
        self._joint_state_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    """スタンドアロン起動用エントリポイント（テスト・非 Isaac Sim 環境向け）。

    Isaac Sim 環境では SimulatorNode を直接インポートして使うこと。
    """
    from sensor_msgs.msg import JointState

    from tomato_harvest_sim.simulator.scene_runtime import IsaacSceneRuntime

    rclpy.init(args=args)
    scene_runtime = IsaacSceneRuntime()
    node = SimulatorNode(scene_runtime)

    # DDS publisher-subscriber discovery が完了するまで待機してから boot する。
    # create_publisher() 直後に publish() すると subscriber に届かないため。
    rclpy.spin_once(node, timeout_sec=1.0)

    snapshot = node.boot()

    # スタンドアロンモード（Isaac Sim なし）では IsaacSimHardwareInterface が
    # 全関節 0 で初期化される。panda_joint4 の上限が -0.07 rad のため 0 は範囲外。
    # /isaac_joint_states に有効な Panda ready 姿勢を送って HW interface を初期化する。
    _PANDA_JOINT_NAMES = [
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7",
    ]
    # Franka Panda "ready" 姿勢 (panda_joint4=-2.356 は limits [-3.07, -0.07] 内)
    _PANDA_READY_POSITIONS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    isaac_js_pub = node.create_publisher(JointState, "/isaac_joint_states", 10)

    def _publish_ready_joint_state() -> None:
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = _PANDA_JOINT_NAMES
        msg.position = _PANDA_READY_POSITIONS
        msg.velocity = [0.0] * len(_PANDA_JOINT_NAMES)
        isaac_js_pub.publish(msg)

    # スタンドアロンモード（Isaac Sim なし）では tick() が呼ばれないため、
    # 1Hz タイマーで最新スナップショットと ready 姿勢を定期再送する。
    current_snapshot: list[SceneSnapshot] = [snapshot]

    def _republish() -> None:
        node._publish_snapshot(current_snapshot[0])
        _publish_ready_joint_state()

    node.create_timer(1.0, _republish)
    _publish_ready_joint_state()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
