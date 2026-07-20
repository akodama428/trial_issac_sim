"""Issue #4の0.1 m lift後5秒保持をphysics stepで採点する純粋ロジック。"""
from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.msg.contracts import Pose3D
from tomato_harvest_sim.simulator.grasp_strategy import (
    distance_between_positions,
    relative_position_in_hand_frame,
)


@dataclass(frozen=True)
class FrictionHoldEvaluationConfig:
    minimum_lift_distance_m: float
    required_steps: int

    def __post_init__(self) -> None:
        if self.minimum_lift_distance_m <= 0.0:
            raise ValueError("minimum_lift_distance_m must be positive")
        if self.required_steps < 1:
            raise ValueError("required_steps must be at least 1")


@dataclass(frozen=True)
class FrictionHoldEvaluationResult:
    active: bool
    complete: bool
    elapsed_steps: int
    slip_m: float
    maximum_slip_m: float


class FrictionHoldEvaluation:
    """lift到達時のhand-local相対位置を基準に保持時間と滑りを測る。"""

    def __init__(self, config: FrictionHoldEvaluationConfig) -> None:
        self._config = config
        self.reset()

    def reset(self) -> None:
        self._anchor: tuple[float, float, float] | None = None
        self._elapsed_steps = 0
        self._maximum_slip_m = 0.0

    def observe(
        self,
        *,
        stem_distance_m: float,
        hand_pose: Pose3D,
        tomato_pose: Pose3D,
    ) -> FrictionHoldEvaluationResult:
        relative = relative_position_in_hand_frame(hand_pose, tomato_pose)
        if self._anchor is None:
            if stem_distance_m < self._config.minimum_lift_distance_m:
                return FrictionHoldEvaluationResult(False, False, 0, 0.0, 0.0)
            self._anchor = relative
        else:
            self._elapsed_steps += 1

        slip_m = distance_between_positions(relative, self._anchor)
        self._maximum_slip_m = max(self._maximum_slip_m, slip_m)
        return FrictionHoldEvaluationResult(
            active=True,
            complete=self._elapsed_steps >= self._config.required_steps,
            elapsed_steps=self._elapsed_steps,
            slip_m=slip_m,
            maximum_slip_m=self._maximum_slip_m,
        )
