from __future__ import annotations

from tomato_harvest_sim.msg.contracts import (
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
)

PANDA_JOINT_BOUNDS: dict[str, tuple[float, float]] = {
    "panda_joint1": (-2.8973, 2.8973),
    "panda_joint2": (-1.7628, 1.7628),
    "panda_joint3": (-2.8973, 2.8973),
    "panda_joint4": (-3.0718, -0.069),
    "panda_joint5": (-2.8973, 2.8973),
    "panda_joint6": (-0.017, 3.7525),
    "panda_joint7": (-2.8973, 2.8973),
}


def joint_trajectory_from_msg(
    joint_trajectory_msg: object,
) -> JointTrajectory | None:
    joint_names = tuple(getattr(joint_trajectory_msg, "joint_names", ()))
    points_msg = getattr(joint_trajectory_msg, "points", ())
    if not joint_names or not points_msg:
        return None
    points: list[JointTrajectoryPoint] = []
    for point in points_msg:
        positions = tuple(
            float(value) for value in getattr(point, "positions", ())
        )
        if not positions:
            return None
        duration = getattr(point, "time_from_start", None)
        time_from_start_sec = 0.0
        if duration is not None:
            time_from_start_sec = float(
                getattr(duration, "sec", 0)
            ) + float(getattr(duration, "nanosec", 0)) / 1_000_000_000.0
        velocities_msg = getattr(point, "velocities", ())
        velocities = (
            tuple(float(value) for value in velocities_msg)
            if velocities_msg
            else None
        )
        points.append(
            JointTrajectoryPoint(
                positions_rad=positions,
                time_from_start_sec=time_from_start_sec,
                velocities_rad_s=velocities,
            )
        )
    return JointTrajectory(joint_names=joint_names, points=tuple(points))


def joint_trajectory_from_request_start_state(
    request: object,
) -> JointTrajectory | None:
    motion_plan_request = getattr(request, "motion_plan_request", None)
    start_state = getattr(motion_plan_request, "start_state", None)
    joint_state = getattr(start_state, "joint_state", None)
    if joint_state is None:
        return None
    joint_names = tuple(str(name) for name in getattr(joint_state, "name", ()))
    positions = tuple(
        float(value) for value in getattr(joint_state, "position", ())
    )
    if not joint_names or not positions:
        return None
    return JointTrajectory(
        joint_names=joint_names,
        points=(
            JointTrajectoryPoint(
                positions_rad=positions,
                time_from_start_sec=0.0,
            ),
        ),
    )


def trajectory_is_noop(
    trajectory: JointTrajectory,
    *,
    start_joint_state: JointStateSnapshot,
    tolerance_rad: float,
) -> bool:
    if trajectory.joint_names != start_joint_state.joint_names:
        return False
    if not trajectory.points:
        return True
    end_positions = trajectory.points[-1].positions_rad
    if len(end_positions) != len(start_joint_state.positions_rad):
        return False
    return max(
        abs(float(end) - float(start))
        for end, start in zip(
            end_positions, start_joint_state.positions_rad, strict=True
        )
    ) <= tolerance_rad


def clamp_joint_state_to_bounds(
    joint_state: JointStateSnapshot,
) -> JointStateSnapshot:
    """Clamp the Isaac initial state to the Panda URDF joint bounds."""
    clamped = list(joint_state.positions_rad)
    for index, name in enumerate(joint_state.joint_names):
        if index >= len(clamped):
            break
        bounds = PANDA_JOINT_BOUNDS.get(name)
        if bounds is not None:
            clamped[index] = min(max(clamped[index], bounds[0]), bounds[1])
    return JointStateSnapshot(
        joint_names=joint_state.joint_names,
        positions_rad=tuple(clamped),
    )


def joint_state_from_trajectory(
    trajectory: JointTrajectory,
) -> JointStateSnapshot:
    last_point = trajectory.points[-1]
    return JointStateSnapshot(
        joint_names=trajectory.joint_names,
        positions_rad=last_point.positions_rad,
    )


def concatenate_trajectories(
    first: JointTrajectory,
    second: JointTrajectory,
) -> JointTrajectory:
    if not first.points:
        return second
    if not second.points:
        return first
    time_offset = first.points[-1].time_from_start_sec
    second_points = (
        second.points[1:]
        if second.points[0].time_from_start_sec == 0.0
        else second.points
    )
    if not second_points:
        return first
    shifted = tuple(
        JointTrajectoryPoint(
            positions_rad=point.positions_rad,
            time_from_start_sec=point.time_from_start_sec + time_offset,
        )
        for point in second_points
    )
    return JointTrajectory(
        joint_names=first.joint_names,
        points=first.points + shifted,
    )
