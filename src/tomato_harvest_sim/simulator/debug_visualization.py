from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from tomato_harvest_sim.msg.contracts import HarvestMotionPlan, JointTrajectory, MotionCommand, Pose3D, SceneSnapshot, TargetEstimate


@dataclass(frozen=True)
class SceneRuntimeDebugState:
    target_estimate_pose: Pose3D | None = None
    pregrasp_pose: Pose3D | None = None
    grasp_pose: Pose3D | None = None
    pull_pose: Pose3D | None = None
    place_pose: Pose3D | None = None
    active_target_pose: Pose3D | None = None
    active_waypoint_pose: Pose3D | None = None
    perception_ray_points: tuple[Pose3D, ...] = ()
    pregrasp_path_points: tuple[Pose3D, ...] = ()
    grasp_path_points: tuple[Pose3D, ...] = ()
    pull_path_points: tuple[Pose3D, ...] = ()
    place_path_points: tuple[Pose3D, ...] = ()
    tracking_path_points: tuple[Pose3D, ...] = ()


def build_scene_runtime_debug_state(
    *,
    snapshot: SceneSnapshot,
    target_estimate: TargetEstimate | None,
    plan: HarvestMotionPlan | None,
    active_motion_command: MotionCommand | None,
    trajectory_path_provider: Callable[[JointTrajectory], tuple[Pose3D, ...]] | None = None,
) -> SceneRuntimeDebugState:
    # active_waypoint_pose: get from active_phase_motion_plan if available
    active_waypoint_pose = None
    active_plan = snapshot.active_phase_motion_plan
    if active_plan is not None and len(active_plan.active_waypoints) >= 2:
        # Use the last waypoint as the active one (matches old behavior of index=1)
        active_waypoint_pose = active_plan.active_waypoints[-1]
    elif active_plan is not None and len(active_plan.active_waypoints) >= 1:
        active_waypoint_pose = active_plan.active_waypoints[0]

    perception_ray_points: tuple[Pose3D, ...] = ()
    target_estimate_pose = None
    if target_estimate is not None:
        target_estimate_pose = target_estimate.target_world_pose
        camera_pose = snapshot.fixed_camera_pose if target_estimate.camera_name == "fixed_camera" else snapshot.hand_camera_pose
        perception_ray_points = _dedupe_pose_points((camera_pose, target_estimate.target_world_pose))

    pregrasp_path_points: tuple[Pose3D, ...] = ()
    grasp_path_points: tuple[Pose3D, ...] = ()
    pull_path_points: tuple[Pose3D, ...] = ()
    place_path_points: tuple[Pose3D, ...] = ()
    if plan is not None:
        pregrasp_path_points = _resolve_path_points(
            waypoints=plan.pregrasp_waypoints or (plan.pregrasp_pose,),
            trajectory=plan.pregrasp_joint_trajectory,
            trajectory_path_provider=trajectory_path_provider,
        )
        grasp_path_points = _resolve_path_points(
            waypoints=plan.grasp_waypoints or (plan.grasp_pose,),
            trajectory=plan.grasp_joint_trajectory,
            trajectory_path_provider=trajectory_path_provider,
        )
        pull_path_points = _resolve_path_points(
            waypoints=plan.pull_waypoints or (plan.pull_pose,),
            trajectory=plan.pull_joint_trajectory,
            trajectory_path_provider=trajectory_path_provider,
        )
        place_path_points = _resolve_path_points(
            waypoints=plan.place_waypoints or (plan.place_pose,),
            trajectory=plan.place_joint_trajectory,
            trajectory_path_provider=trajectory_path_provider,
        )

    # Fallback waypoints come from active_phase_motion_plan if available
    fallback_waypoints: tuple[Pose3D, ...] = ()
    if active_plan is not None:
        fallback_waypoints = active_plan.active_waypoints

    tracking_path_points = _resolve_motion_command_path_points(
        command=active_motion_command,
        fallback_waypoints=fallback_waypoints,
        trajectory_path_provider=trajectory_path_provider,
    )

    return SceneRuntimeDebugState(
        target_estimate_pose=target_estimate_pose,
        pregrasp_pose=plan.pregrasp_pose if plan is not None else None,
        grasp_pose=plan.grasp_pose if plan is not None else None,
        pull_pose=plan.pull_pose if plan is not None else None,
        place_pose=plan.place_pose if plan is not None else None,
        active_target_pose=snapshot.target_tool_pose,
        active_waypoint_pose=active_waypoint_pose,
        perception_ray_points=perception_ray_points,
        pregrasp_path_points=pregrasp_path_points,
        grasp_path_points=grasp_path_points,
        pull_path_points=pull_path_points,
        place_path_points=place_path_points,
        tracking_path_points=tracking_path_points,
    )


def _resolve_motion_command_path_points(
    *,
    command: MotionCommand | None,
    fallback_waypoints: tuple[Pose3D, ...],
    trajectory_path_provider: Callable[[JointTrajectory], tuple[Pose3D, ...]] | None,
) -> tuple[Pose3D, ...]:
    if command is None:
        return _dedupe_pose_points(fallback_waypoints)
    # Get waypoints and trajectory from phase_motion_plan if available
    cmd_waypoints = fallback_waypoints
    cmd_trajectory: JointTrajectory | None = None
    if command.phase_motion_plan is not None:
        cmd_waypoints = command.phase_motion_plan.active_waypoints or fallback_waypoints
        cmd_trajectory = command.phase_motion_plan.joint_trajectory
    return _resolve_path_points(
        waypoints=cmd_waypoints,
        trajectory=cmd_trajectory,
        trajectory_path_provider=trajectory_path_provider,
    )


def _resolve_path_points(
    *,
    waypoints: tuple[Pose3D, ...],
    trajectory: JointTrajectory | None,
    trajectory_path_provider: Callable[[JointTrajectory], tuple[Pose3D, ...]] | None,
) -> tuple[Pose3D, ...]:
    if trajectory is not None and trajectory_path_provider is not None:
        path_points = _dedupe_pose_points(trajectory_path_provider(trajectory))
        if path_points:
            return path_points
    return _dedupe_pose_points(waypoints)


def _dedupe_pose_points(points: tuple[Pose3D, ...]) -> tuple[Pose3D, ...]:
    deduped: list[Pose3D] = []
    for point in points:
        if not deduped or deduped[-1] != point:
            deduped.append(point)
    return tuple(deduped)
