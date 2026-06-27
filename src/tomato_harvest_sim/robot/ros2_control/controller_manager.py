from __future__ import annotations


class ControllerManager:
    def __init__(self, *, available_controllers: tuple[str, ...] = ("joint_trajectory_controller", "gripper_controller")) -> None:
        self._available_controllers = set(available_controllers)

    def ensure_controller(self, controller_name: str) -> bool:
        return controller_name in self._available_controllers
