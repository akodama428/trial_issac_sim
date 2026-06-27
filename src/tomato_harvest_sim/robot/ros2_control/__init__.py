from __future__ import annotations

from typing import TYPE_CHECKING

__all__ = [
    "ControllerManager",
    "JointTrajectoryControllerBridge",
    "JointTrajectoryControllerState",
]

if TYPE_CHECKING:
    from tomato_harvest_sim.robot.ros2_control.controller_manager import ControllerManager
    from tomato_harvest_sim.robot.ros2_control.controller_state import JointTrajectoryControllerState
    from tomato_harvest_sim.robot.ros2_control.joint_trajectory_controller_bridge import JointTrajectoryControllerBridge


def __getattr__(name: str):
    if name == "ControllerManager":
        from tomato_harvest_sim.robot.ros2_control import controller_manager as module

        return module.ControllerManager
    if name == "JointTrajectoryControllerState":
        from tomato_harvest_sim.robot.ros2_control import controller_state as module

        return module.JointTrajectoryControllerState
    if name == "JointTrajectoryControllerBridge":
        from tomato_harvest_sim.robot.ros2_control import joint_trajectory_controller_bridge as module

        return module.JointTrajectoryControllerBridge
    raise AttributeError(name)
