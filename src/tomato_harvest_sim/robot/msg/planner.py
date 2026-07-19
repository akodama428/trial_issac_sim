from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    JointTrajectory,
    SceneSnapshot,
    TargetEstimate,
)


@dataclass(frozen=True)
class MoveIt2PlanningResult:
    success: bool
    backend_name: str
    reason: str
    joint_trajectory: JointTrajectory | None = None
    planning_scene_object_ids: tuple[str, ...] = ()


class MotionPlanner(Protocol):
    def plan(
        self,
        target_estimate: TargetEstimate,
        scene_snapshot: SceneSnapshot,
    ) -> HarvestMotionPlan: ...

    def plan_phase_trajectory(
        self,
        phase: HarvestTaskPhase,
        prior_plan: HarvestMotionPlan,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        scene_snapshot: SceneSnapshot,
    ) -> HarvestMotionPlan | None: ...


class MoveIt2PlannerBridge(Protocol):
    def plan_phase_trajectory(
        self,
        *,
        phase: HarvestTaskPhase,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        scene_snapshot: SceneSnapshot,
        plan: HarvestMotionPlan,
    ) -> MoveIt2PlanningResult: ...
