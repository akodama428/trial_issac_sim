"""把持成立と滑落を物理観測だけから判定する純粋ロジック。"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

from tomato_harvest_sim.msg.contracts import Pose3D


class GraspDecision(StrEnum):
    NONE = "none"
    HELD = "held"
    LOST = "lost"
    RELEASED = "released"


@dataclass(frozen=True)
class FrictionGraspConfig:
    required_steps: int
    minimum_force_n: float
    maximum_relative_speed_m_s: float
    maximum_slip_m: float


def _relative_position(hand: Pose3D, tomato: Pose3D) -> tuple[float, float, float]:
    """world差分をhand local frameへ回し、剛体回転と実滑りを分離する。"""
    x, y, z = tomato.x - hand.x, tomato.y - hand.y, tomato.z - hand.z
    roll, pitch, yaw = (math.radians(value) for value in (hand.roll, hand.pitch, hand.yaw))
    # world R = Rz(yaw) Ry(pitch) Rx(roll) の逆回転を逆順で適用する。
    cy, sy = math.cos(yaw), math.sin(yaw)
    x, y = cy * x + sy * y, -sy * x + cy * y
    cp, sp = math.cos(pitch), math.sin(pitch)
    x, z = cp * x - sp * z, sp * x + cp * z
    cr, sr = math.cos(roll), math.sin(roll)
    y, z = cr * y + sr * z, -sr * y + cr * z
    return x, y, z


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b, strict=True)))


class FrictionGraspStrategy:
    """両指力・相対速度・相対変位から摩擦把持の状態遷移を判定する。"""

    def __init__(self, config: FrictionGraspConfig) -> None:
        if config.required_steps < 1:
            raise ValueError("required_steps must be at least 1")
        self._config = config
        self.reset()

    def reset(self) -> None:
        self._qualifying_steps = 0
        self._previous_relative_position: tuple[float, float, float] | None = None
        self._held_relative_position: tuple[float, float, float] | None = None

    def observe(self, gripper_closed: bool, left_force_n: float, right_force_n: float,
                hand_pose: Pose3D, tomato_pose: Pose3D, dt_sec: float) -> GraspDecision:
        relative_position = _relative_position(hand_pose, tomato_pose)
        if not gripper_closed:
            was_held = self._held_relative_position is not None
            self.reset()
            return GraspDecision.RELEASED if was_held else GraspDecision.NONE

        if self._held_relative_position is not None:
            if _distance(relative_position, self._held_relative_position) > self._config.maximum_slip_m:
                self.reset()
                return GraspDecision.LOST
            self._previous_relative_position = relative_position
            return GraspDecision.NONE

        relative_speed = 0.0
        if self._previous_relative_position is not None and dt_sec > 0.0:
            relative_speed = _distance(relative_position, self._previous_relative_position) / dt_sec
        self._previous_relative_position = relative_position
        qualifies = (
            left_force_n >= self._config.minimum_force_n
            and right_force_n >= self._config.minimum_force_n
            and relative_speed <= self._config.maximum_relative_speed_m_s
        )
        self._qualifying_steps = self._qualifying_steps + 1 if qualifies else 0
        if self._qualifying_steps < self._config.required_steps:
            return GraspDecision.NONE
        self._held_relative_position = relative_position
        return GraspDecision.HELD
