"""Planner が共有する最新入力状態の集約境界 (Issue #10, Step 2)。"""
from __future__ import annotations

from dataclasses import dataclass, replace

from tomato_harvest_sim.msg.contracts import (
    HarvestTaskPhase,
    JointStateSnapshot,
    SceneSnapshot,
    TargetEstimate,
)


@dataclass(frozen=True)
class PlannerState:
    """global/local planner と trigger policy が共有する一時点の入力。"""

    phase: HarvestTaskPhase | None = None
    joint_state: JointStateSnapshot | None = None
    scene_snapshot: SceneSnapshot | None = None
    target_estimate: TargetEstimate | None = None
    tracking_error_rad: float | None = None
    abort_generation: int = 0
    scene_generation: int = 0


class PlannerStateAggregator:
    """ROS callback から届く最新値を一箇所に集約する。"""

    def __init__(self) -> None:
        self._state = PlannerState()

    def snapshot(self) -> PlannerState:
        """現在の immutable snapshot を返す。"""
        return self._state

    def update_phase(self, phase: HarvestTaskPhase) -> None:
        self._state = replace(self._state, phase=phase)

    def update_joint_state(self, joint_state: JointStateSnapshot) -> None:
        self._state = replace(self._state, joint_state=joint_state)

    def update_target_estimate(self, estimate: TargetEstimate) -> None:
        self._state = replace(self._state, target_estimate=estimate)

    def update_tracking_error(self, tracking_error_rad: float | None) -> None:
        self._state = replace(self._state, tracking_error_rad=tracking_error_rad)

    def observe_abort(self) -> None:
        self._state = replace(
            self._state, abort_generation=self._state.abort_generation + 1
        )

    def update_scene_snapshot(self, scene_snapshot: SceneSnapshot) -> None:
        generation = self._state.scene_generation
        if self._state.scene_snapshot is not None and scene_snapshot != self._state.scene_snapshot:
            generation += 1
        self._state = replace(
            self._state,
            scene_snapshot=scene_snapshot,
            scene_generation=generation,
        )
