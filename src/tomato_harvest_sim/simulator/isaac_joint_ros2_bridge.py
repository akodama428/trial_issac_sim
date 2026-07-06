"""
Python bridge between Isaac Sim Python API and ROS2 topics for the C++ ros2_control node.

Publishes:
  /isaac_joint_states  (sensor_msgs/JointState) — positions + velocities read from Isaac Sim

Subscribes:
  /isaac_joint_commands (sensor_msgs/JointState) — position + velocity targets from C++
    IsaacSimHardwareInterface, applied to Isaac Sim via IsaacFrankaDriver.
    Accepts arm[0-6] + finger[7-8] (panda_finger_joint1, panda_finger_joint2).

Lifecycle: call step() once per simulation tick so callbacks are processed and
state is published; call close() on shutdown.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from tomato_harvest_sim.simulator.isaac_franka_driver import IsaacFrankaDriver


class IsaacJointRos2Bridge:
    GRIPPER_OPEN_RAD = 0.04
    GRIPPER_CLOSED_RAD = 0.0
    FINGER_JOINT_NAMES = ("panda_finger_joint1", "panda_finger_joint2")
    _gripper_closed: bool = False  # クラス変数: __new__ 経由でも参照可能

    def __init__(
        self,
        *,
        driver: IsaacFrankaDriver,
        joint_states_topic: str = "/isaac_joint_states",
        joint_commands_topic: str = "/isaac_joint_commands",
        spin_timeout_sec: float = 0.001,
    ) -> None:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        self._driver = driver
        self._spin_timeout_sec = spin_timeout_sec
        self._rclpy = rclpy
        self._initialized_here = False

        if not rclpy.ok():
            rclpy.init(args=None)
            self._initialized_here = True

        self._node: Node = rclpy.create_node("isaac_joint_ros2_bridge")

        from rosgraph_msgs.msg import Clock

        self._pub = self._node.create_publisher(
            JointState, joint_states_topic, rclpy.qos.QoSProfile(depth=1)
        )
        self._sub = self._node.create_subscription(
            JointState,
            joint_commands_topic,
            self._on_joint_command,
            rclpy.qos.QoSProfile(depth=1),
        )
        # Publish simulation time to /clock so that ros2_control (use_sim_time: true)
        # advances the JointTrajectoryController in sync with Isaac Sim physics.
        self._clock_pub = self._node.create_publisher(
            Clock, "/clock", rclpy.qos.QoSProfile(depth=1)
        )

        self._pending_command: tuple[np.ndarray, np.ndarray] | None = None
        self._gripper_closed: bool = False  # デフォルトは開（False=open）
        self._JointState = JointState
        self._Clock = Clock

        # motion_command_executor_node からのグリッパー状態を購読する
        from std_msgs.msg import String as StringMsg
        self._node.create_subscription(
            StringMsg,
            "/tomato_harvest/gripper_closed",
            self._on_gripper_command,
            10,
        )

    def step(self) -> None:
        self._publish_clock()
        self._publish_state()
        self._rclpy.spin_once(self._node, timeout_sec=self._spin_timeout_sec)
        self._apply_combined_command()

    def close(self) -> None:
        self._node.destroy_node()
        if self._initialized_here and self._rclpy.ok():
            self._rclpy.shutdown()

    def _publish_clock(self) -> None:
        try:
            import omni.timeline
            sim_time_sec = float(omni.timeline.get_timeline_interface().get_current_time())
        except Exception:
            return
        msg = self._Clock()
        msg.clock.sec = int(sim_time_sec)
        msg.clock.nanosec = int((sim_time_sec % 1.0) * 1_000_000_000)
        self._clock_pub.publish(msg)

    def _publish_state(self) -> None:
        # TrajectoryTrackingCoordinator を経由しないため articulation が未初期化のまま残る。
        # ここで初期化することでデッドロックを防ぐ。
        # (hardware interface は joint_state 受信まで commands を送らず、
        #  driver は commands 受信まで joint_state を返さない、という循環依存を断ち切る)
        if not self._driver.initialize_if_needed():
            return
        positions = self._driver.current_joint_positions()
        if positions is None:
            return
        velocities = self._driver.current_joint_velocities()
        if velocities is None:
            velocities = np.zeros_like(positions)

        msg = self._JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(self._driver.ARM_JOINT_NAMES)
        msg.position = [float(v) for v in positions[: len(self._driver.ARM_JOINT_NAMES)]]
        msg.velocity = [float(v) for v in velocities[: len(self._driver.ARM_JOINT_NAMES)]]
        self._pub.publish(msg)

    def _on_gripper_command(self, msg: object) -> None:
        """motion_command_executor_node からの gripper_closed 文字列を受け取る。"""
        data = str(getattr(msg, "data", "")).strip().lower()
        prev = self._gripper_closed
        self._gripper_closed = (data == "true")
        if prev != self._gripper_closed:
            print(
                f"[Bridge] gripper_closed: {prev} → {self._gripper_closed} (msg={data!r})",
                flush=True,
            )

    def _on_joint_command(self, msg: object) -> None:
        """JTC からのアーム関節コマンドを保存する（フィンガーは _apply_combined_command で処理）。"""
        names = list(getattr(msg, "name", []))
        positions_list = list(getattr(msg, "position", []))
        velocities_list = list(getattr(msg, "velocity", []))

        n_arm = len(self._driver.ARM_JOINT_NAMES)
        pos = np.zeros(n_arm, dtype=float)
        vel = np.zeros(n_arm, dtype=float)

        for i, joint_name in enumerate(self._driver.ARM_JOINT_NAMES):
            if joint_name in names:
                j = names.index(joint_name)
                if j < len(positions_list):
                    pos[i] = float(positions_list[j])
                if j < len(velocities_list):
                    vel[i] = float(velocities_list[j])

        self._pending_command = (pos, vel)

    def _apply_combined_command(self) -> None:
        """アームとフィンガーを毎ステップ適用する。

        アーム: JTCコマンドが届いていればそれを使用、なければ現在位置を維持。
        フィンガー: 毎ステップ gripper_closed 状態から決定。
        JTCがコマンド停止後も（AT_GRASP/GRASP_EVALUATION など）グリッパーを維持するため、
        フィンガー制御をJTCコマンドに依存させない。
        """
        if not self._driver.initialize_if_needed():
            return
        current = self._driver.current_joint_positions()
        if current is None:
            return

        n_arm = len(self._driver.ARM_JOINT_NAMES)
        n_finger = len(self.FINGER_JOINT_NAMES)
        finger_target = self.GRIPPER_CLOSED_RAD if self._gripper_closed else self.GRIPPER_OPEN_RAD

        full_positions = np.asarray(current, dtype=float).copy()
        full_velocities = np.zeros_like(full_positions)

        # アーム: JTCコマンドが届いていればそれを適用、なければ現在位置を維持
        has_jtc_command = self._pending_command is not None
        if has_jtc_command:
            arm_pos, arm_vel = self._pending_command
            self._pending_command = None
            full_positions[:n_arm] = arm_pos[:n_arm]
            full_velocities[:n_arm] = arm_vel[:n_arm]

        # フィンガー: 常に gripper_closed 状態から決定（JTCコマンドに依存しない）
        n_total = len(full_positions)
        has_finger_dofs = n_total > n_arm and n_total >= n_arm + n_finger
        if has_finger_dofs:
            full_positions[n_arm:n_arm + n_finger] = finger_target
        else:
            # アーティキュレーションがフィンガーDOFを持たない場合
            print(
                f"[Bridge] WARNING: articulation has {n_total} DOFs (< {n_arm + n_finger}), "
                f"finger joints not in articulation. gripper_closed={self._gripper_closed}",
                flush=True,
            )

        if has_jtc_command:
            # JTCコマンドあり → 速度+位置でアーム制御（フィンガーも同じアクションに含める）
            self._driver.set_joint_velocity_targets_with_debug(
                positions=full_positions,
                velocities=full_velocities,
                context="combined_command",
            )
        elif has_finger_dofs:
            # JTCコマンドなし（AT_GRASP/GRASP_EVALUATION など）
            # フィンガーのみ position-only action で駆動し、アーム関節は変更しない。
            # joint_indices を使うことで velocity=0 がアームに干渉するのを防ぐ。
            finger_indices = list(range(n_arm, n_arm + n_finger))
            self._driver.set_finger_positions_only(
                full_positions[n_arm:n_arm + n_finger],
                joint_indices=finger_indices,
            )

    def apply_gripper_state(self, closed: bool) -> None:
        """scene_runtime 経由で受け取ったグリッパー状態を _gripper_closed フラグへ同期する。

        spin_once のキューが /isaac_joint_commands で溢れているため
        _on_gripper_command サブスクリプション経由では確実に届かない。
        node.tick() 後に scene_runtime.state.gripper_closed を直接渡すことで回避する。

        フィンガーの実際の適用は次の bridge.step() → _apply_combined_command() で行う。
        ここで apply_action を呼ぶと、アーム JTC コマンドをアーム現在位置+速度ゼロで
        上書きしてしまい、軌跡追従が停止するため避ける。
        """
        if closed != self._gripper_closed:
            print(
                f"[Bridge] gripper_closed sync: {self._gripper_closed} → {closed}",
                flush=True,
            )
        self._gripper_closed = closed
