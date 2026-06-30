"""
HardwareControlPort that reads joint state from the /joint_states ROS2 topic
published by the C++ joint_state_broadcaster (part of franka_ros2_control).

write_command() is a no-op because the C++ JointTrajectoryController owns
command writing in the new architecture. Direct step-mode commands still go
through IsaacRos2ControlSystem (the caller decides which port to use based on
whether trajectory mode or step mode is active).
"""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

import numpy as np

from tomato_harvest_sim.api.hardware_control import HardwareCommandSample, HardwareControlPort, HardwareStateSample
from tomato_harvest_sim.api.contracts import Pose3D

if TYPE_CHECKING:
    from tomato_harvest_sim.simulator.isaac_franka_driver import IsaacFrankaDriver


class Ros2JointStateHardwarePort:
    """
    HardwareControlPort that provides joint observations from /joint_states
    and EE pose from the Isaac Sim Python API.

    This is the observation-only port used by TrajectoryTrackingCoordinator
    when the C++ JointTrajectoryController is active.
    """

    def __init__(
        self,
        *,
        driver: IsaacFrankaDriver,
        joint_states_topic: str = "/joint_states",
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

        self._node: Node = rclpy.create_node("ros2_joint_state_hardware_port")
        self._sub = self._node.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_state,
            rclpy.qos.QoSProfile(depth=1),
        )

        self._last_joint_names: tuple[str, ...] = ()
        self._last_positions: tuple[float, ...] | None = None
        self._last_velocities: tuple[float, ...] | None = None
        self._last_stamp_sec: float = 0.0

    def initialize_if_needed(self) -> bool:
        return self._driver.initialize_if_needed()

    def read_state(self) -> HardwareStateSample | None:
        self._rclpy.spin_once(self._node, timeout_sec=self._spin_timeout_sec)

        if self._last_positions is None:
            return None

        ee_pose: Pose3D | None = None
        if self._driver.initialize_if_needed():
            ee_pose = self._driver.current_end_effector_pose()

        from tomato_harvest_sim.api.contracts import JointStateSnapshot
        joint_state_snapshot = JointStateSnapshot(
            joint_names=self._last_joint_names,
            positions_rad=self._last_positions,
        )

        return HardwareStateSample(
            joint_names=self._last_joint_names,
            positions_rad=self._last_positions,
            velocities_rad_s=self._last_velocities or tuple(0.0 for _ in self._last_positions),
            timestamp_sec=self._last_stamp_sec,
            end_effector_pose=ee_pose,
            joint_state_snapshot=joint_state_snapshot,
        )

    def write_command(self, command: HardwareCommandSample) -> None:
        # C++ JointTrajectoryController owns writes in trajectory mode.
        # Step-mode callers should use IsaacRos2ControlSystem directly.
        pass

    def close(self) -> None:
        self._node.destroy_node()
        if self._initialized_here and self._rclpy.ok():
            self._rclpy.shutdown()

    def _on_joint_state(self, msg: object) -> None:
        names = tuple(str(n) for n in getattr(msg, "name", ()))
        positions = tuple(float(v) for v in getattr(msg, "position", ()))
        velocities = tuple(float(v) for v in getattr(msg, "velocity", ()))
        stamp = getattr(msg, "header", None)
        if stamp is not None:
            stamp = getattr(stamp, "stamp", None)
        if stamp is not None:
            self._last_stamp_sec = float(getattr(stamp, "sec", 0)) + float(
                getattr(stamp, "nanosec", 0)
            ) / 1_000_000_000.0
        else:
            self._last_stamp_sec = time.monotonic()

        self._last_joint_names = names
        self._last_positions = positions
        self._last_velocities = velocities if velocities else None
