"""
Python bridge between Isaac Sim Python API and ROS2 topics for the C++ ros2_control node.

Publishes:
  /isaac_joint_states  (sensor_msgs/JointState) — positions + velocities read from Isaac Sim

Subscribes:
  /isaac_joint_commands (sensor_msgs/JointState) — position + velocity targets from
    C++ IsaacSimHardwareInterface, applied to Isaac Sim via IsaacFrankaDriver.
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
    FINGER_JOINT_NAMES = ("panda_finger_joint1", "panda_finger_joint2")

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
        self._JointState = JointState
        self._Clock = Clock

    def step(self) -> None:
        self._publish_clock()
        self._publish_state()
        self._rclpy.spin_once(self._node, timeout_sec=self._spin_timeout_sec)
        self._apply_pending_command()

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

        joint_names = self._joint_names_for_dof_count(int(positions.shape[0]))
        msg = self._JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(joint_names)
        msg.position = [float(v) for v in positions[: len(joint_names)]]
        msg.velocity = [float(v) for v in velocities[: len(joint_names)]]
        self._pub.publish(msg)

    def _on_joint_command(self, msg: object) -> None:
        """HWI からの関節コマンドを保存する。"""
        names = list(getattr(msg, "name", []))
        positions_list = list(getattr(msg, "position", []))
        velocities_list = list(getattr(msg, "velocity", []))

        current = self._driver.current_joint_positions()
        n_dofs = len(current) if current is not None else len(self._driver.ARM_JOINT_NAMES) + len(self.FINGER_JOINT_NAMES)
        pos = np.asarray(current, dtype=float).copy() if current is not None else np.zeros(n_dofs, dtype=float)
        vel = np.zeros(n_dofs, dtype=float)
        joint_indices = {
            joint_name: index
            for index, joint_name in enumerate(self._joint_names_for_dof_count(n_dofs))
        }

        for message_index, joint_name in enumerate(names):
            joint_index = joint_indices.get(joint_name)
            if joint_index is None:
                continue
            if message_index < len(positions_list):
                pos[joint_index] = float(positions_list[message_index])
            if message_index < len(velocities_list):
                vel[joint_index] = float(velocities_list[message_index])

        self._pending_command = (pos, vel)

    def _apply_pending_command(self) -> None:
        """HWI から受けた最新の関節コマンドを articulation へ適用する。"""
        if self._pending_command is None:
            return
        if not self._driver.initialize_if_needed():
            return
        current = self._driver.current_joint_positions()
        if current is None:
            return

        pending_positions, pending_velocities = self._pending_command
        full_positions = np.asarray(current, dtype=float).copy()
        full_velocities = np.zeros_like(full_positions)
        copy_length = min(len(full_positions), len(pending_positions))
        full_positions[:copy_length] = pending_positions[:copy_length]
        full_velocities[:copy_length] = pending_velocities[:copy_length]
        self._pending_command = None

        self._driver.set_joint_velocity_targets_with_debug(
            positions=full_positions,
            velocities=full_velocities,
            context="isaac_joint_command",
        )

    def _joint_names_for_dof_count(self, dof_count: int) -> tuple[str, ...]:
        arm_joint_names = tuple(self._driver.ARM_JOINT_NAMES)
        if dof_count <= len(arm_joint_names):
            return arm_joint_names[:dof_count]
        finger_count = min(dof_count - len(arm_joint_names), len(self.FINGER_JOINT_NAMES))
        return arm_joint_names + self.FINGER_JOINT_NAMES[:finger_count]
