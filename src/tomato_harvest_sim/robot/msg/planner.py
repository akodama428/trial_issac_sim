from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    JointStateSnapshot,
    JointTrajectory,
    SceneSnapshot,
    TargetEstimate,
    TfTreeSnapshot,
)


@dataclass(frozen=True)
class PlannerBackendInfo:
    name: str
    moveit2_enabled: bool


@dataclass(frozen=True)
class MoveIt2PlanningResult:
    success: bool
    backend_name: str
    reason: str
    pregrasp_joint_trajectory: JointTrajectory | None = None
    grasp_joint_trajectory: JointTrajectory | None = None
    pull_joint_trajectory: JointTrajectory | None = None
    place_joint_trajectory: JointTrajectory | None = None
    planning_scene_object_ids: tuple[str, ...] = ()


class MotionPlanner(Protocol):
    def plan(
        self,
        target_estimate: TargetEstimate,
        joint_state: JointStateSnapshot,
        tf_tree: TfTreeSnapshot,
        scene_snapshot: SceneSnapshot,
    ) -> HarvestMotionPlan: ...


class MoveIt2PlannerBridge(Protocol):
    def plan_phase_trajectories(
        self,
        *,
        joint_state: JointStateSnapshot,
        tf_tree: TfTreeSnapshot,
        scene_snapshot: SceneSnapshot,
        plan: HarvestMotionPlan,
    ) -> MoveIt2PlanningResult: ...
