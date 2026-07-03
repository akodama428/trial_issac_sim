"""
Python bridge between Isaac Sim Python API and ROS2 topics for the C++ ros2_control node.

Publishes:
  /isaac_joint_states  (sensor_msgs/JointState) — positions + velocities read from Isaac Sim

Subscribes:
  /isaac_joint_commands (sensor_msgs/JointState) — position + velocity targets from C++
    IsaacSimHardwareInterface, applied to Isaac Sim via IsaacFrankaDriver.

Lifecycle: call step() once per simulation tick so callbacks are processed and
state is published; call close() on shutdown.
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from tomato_harvest_sim.simulator.isaac_franka_driver import IsaacFrankaDriver


class IsaacJointRos2Bridge:
    GRIPPER_OPEN_RAD = 0.04
    GRIPPER_CLOSED_RAD = 0.0

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

        msg = self._JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(self._driver.ARM_JOINT_NAMES)
        msg.position = [float(v) for v in positions[: len(self._driver.ARM_JOINT_NAMES)]]
        msg.velocity = [float(v) for v in velocities[: len(self._driver.ARM_JOINT_NAMES)]]
        self._pub.publish(msg)

    def _on_joint_command(self, msg: object) -> None:
        names = list(getattr(msg, "name", []))
        positions_list = list(getattr(msg, "position", []))
        velocities_list = list(getattr(msg, "velocity", []))

        n = len(self._driver.ARM_JOINT_NAMES)
        pos = np.zeros(n, dtype=float)
        vel = np.zeros(n, dtype=float)

        for i, joint_name in enumerate(self._driver.ARM_JOINT_NAMES):
            if joint_name in names:
                j = names.index(joint_name)
                if j < len(positions_list):
                    pos[i] = float(positions_list[j])
                if j < len(velocities_list):
                    vel[i] = float(velocities_list[j])

        self._pending_command = (pos, vel)

    def _apply_pending_command(self) -> None:
        if self._pending_command is None:
            return
        positions, velocities = self._pending_command
        self._pending_command = None

        if not self._driver.initialize_if_needed():
            return

        current = self._driver.current_joint_positions()
        if current is None:
            return

        full_positions = np.asarray(current, dtype=float).copy()
        full_positions[: len(positions)] = positions
        full_velocities = np.zeros_like(full_positions)
        full_velocities[: len(velocities)] = velocities

        self._driver.set_joint_velocity_targets_with_debug(
            positions=full_positions,
            velocities=full_velocities,
            context="isaac_joint_ros2_bridge",
        )
