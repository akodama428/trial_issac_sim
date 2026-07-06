from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from tomato_harvest_sim.msg.contracts import JointStateSnapshot, Pose3D


@dataclass(frozen=True)
class HardwareStateSample:
    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...]
    velocities_rad_s: tuple[float, ...]
    timestamp_sec: float
    end_effector_pose: Pose3D | None = None
    joint_state_snapshot: JointStateSnapshot | None = None


@dataclass(frozen=True)
class HardwareCommandSample:
    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...] | None
    velocities_rad_s: tuple[float, ...] | None
    context: str
    gripper_closed: bool | None = None


class HardwareControlPort(Protocol):
    def initialize_if_needed(self) -> bool: ...

    def read_state(self) -> HardwareStateSample | None: ...

    def write_command(self, command: HardwareCommandSample) -> None: ...
