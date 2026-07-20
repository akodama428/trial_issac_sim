from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    Pose3D,
)
from tomato_harvest_sim.msg.topics import DEFAULT_JOINT_NAMES, home_joint_state

PlanningTarget = Pose3D | JointStateSnapshot


@dataclass(frozen=True)
class PhasePlanningSpec:
    """Common phase planner inputs which vary by harvest phase."""

    phase: HarvestTaskPhase
    target_sequences: tuple[tuple[PlanningTarget, ...], ...]
    attach_tomato: bool
    allow_gripper_target_contact: bool
    failure_reason: str
    joint_fallback_success_reason: str | None = None


def phase_planning_specs(
    *,
    plan: HarvestMotionPlan,
    joint_state: JointStateSnapshot,
    home_via_threshold_rad: float,
) -> tuple[PhasePlanningSpec, ...]:
    """Build the ordered target alternatives for every moving phase."""
    direct_pregrasp: tuple[PlanningTarget, ...] = (plan.pregrasp_pose,)
    pregrasp_sequences = (
        ((home_joint_state(), *direct_pregrasp), direct_pregrasp)
        if should_start_via_home(
            joint_state, threshold_rad=home_via_threshold_rad
        )
        else (direct_pregrasp,)
    )
    place_targets: tuple[PlanningTarget, ...] = (
        plan.place_waypoints or (plan.place_pose,)
    )
    # place_waypointsは「上空→設置点」の順で保持する。設置点は現在位置なので
    # 除外し、残りを逆順に辿ってからhomeへ移動する。
    retreat_targets = tuple(reversed(plan.place_waypoints[:-1]))
    return_home_sequences: tuple[tuple[PlanningTarget, ...], ...] = (
        ((*retreat_targets, home_joint_state()),)
        if retreat_targets
        else ()
    )
    return (
        PhasePlanningSpec(
            phase=HarvestTaskPhase.MOVING_TO_PREGRASP,
            target_sequences=pregrasp_sequences,
            attach_tomato=False,
            allow_gripper_target_contact=False,
            failure_reason="pregrasp_plan_failed",
        ),
        PhasePlanningSpec(
            phase=HarvestTaskPhase.MOVING_TO_GRASP,
            target_sequences=((plan.grasp_pose,),),
            attach_tomato=False,
            allow_gripper_target_contact=True,
            failure_reason="grasp_plan_failed",
        ),
        PhasePlanningSpec(
            phase=HarvestTaskPhase.DETACHING,
            target_sequences=((plan.pull_pose,),),
            attach_tomato=True,
            allow_gripper_target_contact=False,
            failure_reason="pull_plan_failed",
        ),
        PhasePlanningSpec(
            phase=HarvestTaskPhase.MOVING_TO_PLACE,
            target_sequences=(place_targets,),
            attach_tomato=True,
            allow_gripper_target_contact=False,
            failure_reason="place_plan_failed",
            joint_fallback_success_reason="joint_goal_fallback",
        ),
        PhasePlanningSpec(
            phase=HarvestTaskPhase.RETURNING_HOME,
            target_sequences=return_home_sequences,
            attach_tomato=False,
            allow_gripper_target_contact=False,
            failure_reason="home_plan_failed",
        ),
    )


def arm_joint_goal_from_ik_solution(
    *,
    solution_joint_names: tuple[str, ...],
    solution_positions_rad: tuple[float, ...],
    arm_joint_names: tuple[str, ...],
) -> JointStateSnapshot | None:
    """Project a full IK solution onto the ordered arm joint set."""
    by_name = dict(zip(solution_joint_names, solution_positions_rad))
    if any(name not in by_name for name in arm_joint_names):
        return None
    return JointStateSnapshot(
        joint_names=arm_joint_names,
        positions_rad=tuple(float(by_name[name]) for name in arm_joint_names),
    )


def should_start_via_home(
    joint_state: JointStateSnapshot, *, threshold_rad: float
) -> bool:
    """Return whether a far initial configuration should first pass home."""
    if threshold_rad <= 0.0:
        return False
    home = home_joint_state()
    home_by_name = dict(zip(home.joint_names, home.positions_rad))
    deltas = [
        abs(position - home_by_name[name])
        for name, position in zip(
            joint_state.joint_names, joint_state.positions_rad
        )
        if name in home_by_name
    ]
    return bool(deltas) and max(deltas) > threshold_rad


def ik_goal_is_near_seed(
    *,
    seed: JointStateSnapshot,
    goal: JointStateSnapshot,
    max_joint_delta_rad: float,
) -> bool:
    """Accept only the IK branch close to the current joint seed."""
    seed_by_name = dict(zip(seed.joint_names, seed.positions_rad))
    common = [name for name in goal.joint_names if name in seed_by_name]
    if not common:
        return False
    goal_by_name = dict(zip(goal.joint_names, goal.positions_rad))
    return all(
        abs(goal_by_name[name] - seed_by_name[name]) <= max_joint_delta_rad
        for name in common
    )


def goal_joint_window(
    joint_state: JointStateSnapshot, *, window_rad: float
) -> tuple[tuple[str, float, float], ...] | None:
    """Build per-arm-joint windows around the current configuration."""
    if window_rad <= 0.0:
        return None
    windows = tuple(
        (name, float(position), float(window_rad))
        for name, position in zip(
            joint_state.joint_names, joint_state.positions_rad
        )
        if name in DEFAULT_JOINT_NAMES
    )
    return windows or None
