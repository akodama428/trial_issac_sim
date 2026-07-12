"""JSON シリアライズ / デシリアライズ関数。

msg 型を std_msgs/String (JSON) で ROS2 トピック越しに輸送するための変換ロジック。
"""
from __future__ import annotations

import json

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointTrajectory,
    JointTrajectoryPoint,
    MotionCommand,
    PhaseId,
    PhaseMotionPlan,
    PlanProducerKind,
    Pose3D,
    ScenePhase,
    SceneSnapshot,
    TargetEstimate,
    TomatoStatus,
)


# ---------------------------------------------------------------------------
# Pose3D
# ---------------------------------------------------------------------------

def pose_to_dict(pose: Pose3D | None) -> dict[str, float] | None:
    if pose is None:
        return None
    return {"x": pose.x, "y": pose.y, "z": pose.z,
            "roll": pose.roll, "pitch": pose.pitch, "yaw": pose.yaw}


def pose_from_dict(data: dict[str, object] | None) -> Pose3D | None:
    if data is None:
        return None
    return Pose3D(
        x=float(data["x"]), y=float(data["y"]), z=float(data["z"]),
        roll=float(data["roll"]), pitch=float(data["pitch"]), yaw=float(data["yaw"]),
    )


# ---------------------------------------------------------------------------
# JointTrajectory
# ---------------------------------------------------------------------------

def trajectory_to_dict(trajectory: JointTrajectory | None) -> dict[str, object] | None:
    if trajectory is None:
        return None
    return {
        "joint_names": list(trajectory.joint_names),
        "points": [
            {"positions_rad": list(p.positions_rad), "time_from_start_sec": p.time_from_start_sec}
            for p in trajectory.points
        ],
    }


def trajectory_from_dict(data: dict[str, object] | None) -> JointTrajectory | None:
    if data is None:
        return None
    points = []
    for p in data.get("points", []):
        pd = p if isinstance(p, dict) else {}
        points.append(JointTrajectoryPoint(
            positions_rad=tuple(float(v) for v in pd.get("positions_rad", [])),
            time_from_start_sec=float(pd.get("time_from_start_sec", 0.0)),
        ))
    return JointTrajectory(
        joint_names=tuple(str(n) for n in data.get("joint_names", [])),
        points=tuple(points),
    )


# ---------------------------------------------------------------------------
# PhaseMotionPlan
# ---------------------------------------------------------------------------

def phase_motion_plan_to_dict(plan: PhaseMotionPlan | None) -> dict[str, object] | None:
    if plan is None:
        return None
    return {
        "phase_id": plan.phase_id.value,
        "phase_goal_pose": pose_to_dict(plan.phase_goal_pose),
        "active_waypoints": [pose_to_dict(p) for p in plan.active_waypoints],
        "joint_trajectory": trajectory_to_dict(plan.joint_trajectory),
    }


def phase_motion_plan_from_dict(data: dict[str, object] | None) -> PhaseMotionPlan | None:
    if data is None:
        return None
    waypoints = [
        pose_from_dict(w if isinstance(w, dict) else None)
        for w in data.get("active_waypoints", [])
    ]
    return PhaseMotionPlan(
        phase_id=PhaseId(str(data["phase_id"])),
        phase_goal_pose=pose_from_dict(data.get("phase_goal_pose") if isinstance(data.get("phase_goal_pose"), dict) else None),
        active_waypoints=tuple(w for w in waypoints if w is not None),
        joint_trajectory=trajectory_from_dict(
            data.get("joint_trajectory") if isinstance(data.get("joint_trajectory"), dict) else None
        ),
    )


# ---------------------------------------------------------------------------
# MotionCommand
# ---------------------------------------------------------------------------

def motion_command_to_dict(command: MotionCommand) -> dict[str, object]:
    return {
        "command_name": command.command_name,
        "planner_name": command.planner_name,
        "target_pose": pose_to_dict(command.target_pose),
        "gripper_closed": command.gripper_closed,
        "phase_motion_plan": phase_motion_plan_to_dict(command.phase_motion_plan),
    }


def motion_command_from_dict(data: dict[str, object]) -> MotionCommand:
    return MotionCommand(
        command_name=str(data["command_name"]),
        planner_name=str(data["planner_name"]),
        target_pose=pose_from_dict(data.get("target_pose") if isinstance(data.get("target_pose"), dict) else None),
        gripper_closed=bool(data["gripper_closed"]) if data.get("gripper_closed") is not None else None,
        phase_motion_plan=phase_motion_plan_from_dict(
            data.get("phase_motion_plan") if isinstance(data.get("phase_motion_plan"), dict) else None
        ),
    )


def motion_command_to_json(command: MotionCommand) -> str:
    return json.dumps(motion_command_to_dict(command))


def motion_command_from_json(json_str: str) -> MotionCommand:
    return motion_command_from_dict(json.loads(json_str))


# ---------------------------------------------------------------------------
# TargetEstimate
# ---------------------------------------------------------------------------

def target_estimate_to_dict(estimate: TargetEstimate) -> dict[str, object]:
    return {
        "camera_name": estimate.camera_name,
        "target_world_pose": pose_to_dict(estimate.target_world_pose),
        "target_camera_pose": pose_to_dict(estimate.target_camera_pose),
        "confidence": estimate.confidence,
    }


def target_estimate_from_dict(data: dict[str, object]) -> TargetEstimate:
    return TargetEstimate(
        camera_name=str(data["camera_name"]),
        target_world_pose=pose_from_dict(data.get("target_world_pose")) or Pose3D(0, 0, 0, 0, 0, 0),
        target_camera_pose=pose_from_dict(data.get("target_camera_pose")) or Pose3D(0, 0, 0, 0, 0, 0),
        confidence=float(data.get("confidence", 0.0)),
    )


def target_estimate_to_json(estimate: TargetEstimate) -> str:
    return json.dumps(target_estimate_to_dict(estimate))


def target_estimate_from_json(json_str: str) -> TargetEstimate:
    return target_estimate_from_dict(json.loads(json_str))


# ---------------------------------------------------------------------------
# SceneSnapshot
# ---------------------------------------------------------------------------

def scene_snapshot_to_dict(snapshot: SceneSnapshot) -> dict[str, object]:
    return {
        "phase": snapshot.phase.value,
        "active_camera": snapshot.active_camera,
        "tomato_attached": snapshot.tomato_attached,
        "tomato_status": snapshot.tomato_status.value,
        "gripper_closed": snapshot.gripper_closed,
        "robot_home": snapshot.robot_home,
        "cycle_id": snapshot.cycle_id,
        "robot_model": snapshot.robot_model,
        "robot_base_pose": pose_to_dict(snapshot.robot_base_pose),
        "fixed_camera_pose": pose_to_dict(snapshot.fixed_camera_pose),
        "hand_camera_pose": pose_to_dict(snapshot.hand_camera_pose),
        "branch_pose": pose_to_dict(snapshot.branch_pose),
        "stem_pose": pose_to_dict(snapshot.stem_pose),
        "tomato_pose": pose_to_dict(snapshot.tomato_pose),
        "tray_pose": pose_to_dict(snapshot.tray_pose),
        "robot_tool_pose": pose_to_dict(snapshot.robot_tool_pose),
        "target_tool_pose": pose_to_dict(snapshot.target_tool_pose),
        "grasp_result_reason": snapshot.grasp_result_reason,
        "active_phase_motion_plan": phase_motion_plan_to_dict(snapshot.active_phase_motion_plan),
        "left_finger_contact": snapshot.left_finger_contact,
        "right_finger_contact": snapshot.right_finger_contact,
        "left_finger_force_n": snapshot.left_finger_force_n,
        "right_finger_force_n": snapshot.right_finger_force_n,
    }


def scene_snapshot_from_dict(data: dict[str, object]) -> SceneSnapshot:
    _zero = Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return SceneSnapshot(
        phase=ScenePhase(str(data["phase"])),
        active_camera=str(data["active_camera"]),
        tomato_attached=bool(data["tomato_attached"]),
        tomato_status=TomatoStatus(str(data["tomato_status"])),
        gripper_closed=bool(data["gripper_closed"]),
        robot_home=bool(data["robot_home"]),
        cycle_id=int(data["cycle_id"]),
        robot_model=str(data["robot_model"]),
        robot_base_pose=pose_from_dict(data.get("robot_base_pose")) or _zero,
        fixed_camera_pose=pose_from_dict(data.get("fixed_camera_pose")) or _zero,
        hand_camera_pose=pose_from_dict(data.get("hand_camera_pose")) or _zero,
        branch_pose=pose_from_dict(data.get("branch_pose")) or _zero,
        stem_pose=pose_from_dict(data.get("stem_pose")) or _zero,
        tomato_pose=pose_from_dict(data.get("tomato_pose")) or _zero,
        tray_pose=pose_from_dict(data.get("tray_pose")) or _zero,
        robot_tool_pose=pose_from_dict(data.get("robot_tool_pose")) or _zero,
        target_tool_pose=pose_from_dict(data.get("target_tool_pose")),
        grasp_result_reason=str(data["grasp_result_reason"]) if data.get("grasp_result_reason") is not None else None,
        active_phase_motion_plan=phase_motion_plan_from_dict(
            data.get("active_phase_motion_plan") if isinstance(data.get("active_phase_motion_plan"), dict) else None
        ),
        left_finger_contact=bool(data.get("left_finger_contact", False)),
        right_finger_contact=bool(data.get("right_finger_contact", False)),
        left_finger_force_n=float(data["left_finger_force_n"]) if data.get("left_finger_force_n") is not None else None,
        right_finger_force_n=float(data["right_finger_force_n"]) if data.get("right_finger_force_n") is not None else None,
    )


def scene_snapshot_from_json(json_str: str) -> SceneSnapshot:
    return scene_snapshot_from_dict(json.loads(json_str))


# ---------------------------------------------------------------------------
# HarvestMotionPlan
# ---------------------------------------------------------------------------

def harvest_motion_plan_to_dict(plan: HarvestMotionPlan) -> dict[str, object]:
    return {
        "planner_name": plan.planner_name,
        "target_pose": pose_to_dict(plan.target_pose),
        "pregrasp_pose": pose_to_dict(plan.pregrasp_pose),
        "grasp_pose": pose_to_dict(plan.grasp_pose),
        "pull_pose": pose_to_dict(plan.pull_pose),
        "place_pose": pose_to_dict(plan.place_pose),
        "pregrasp_joint_trajectory": trajectory_to_dict(plan.pregrasp_joint_trajectory),
        "grasp_joint_trajectory": trajectory_to_dict(plan.grasp_joint_trajectory),
        "pull_joint_trajectory": trajectory_to_dict(plan.pull_joint_trajectory),
        "place_joint_trajectory": trajectory_to_dict(plan.place_joint_trajectory),
        "home_joint_trajectory": trajectory_to_dict(plan.home_joint_trajectory),
        "plan_revision": plan.plan_revision,
        "generated_at_sec": plan.generated_at_sec,
        "planned_from_phase": plan.planned_from_phase.value if plan.planned_from_phase is not None else None,
        "producer_kind": plan.producer_kind.value,
        "producer_instance_id": plan.producer_instance_id,
    }


def _planned_from_phase_from_value(value: object) -> HarvestTaskPhase | None:
    """未知の phase 値は None へ落とし、旧契約と将来契約の双方を許容する。"""
    if value is None:
        return None
    try:
        return HarvestTaskPhase(str(value))
    except ValueError:
        return None


def _producer_kind_from_value(value: object) -> PlanProducerKind:
    """未知の producer 種別はエラーにせず UNKNOWN として消費側の規則に委ねる。"""
    if value is None:
        return PlanProducerKind.GLOBAL_PLANNER
    try:
        return PlanProducerKind(str(value))
    except ValueError:
        return PlanProducerKind.UNKNOWN


def harvest_motion_plan_from_dict(data: dict[str, object]) -> HarvestMotionPlan:
    _zero = Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return HarvestMotionPlan(
        planner_name=str(data.get("planner_name", "unknown")),
        target_pose=pose_from_dict(data.get("target_pose")) or _zero,
        pregrasp_pose=pose_from_dict(data.get("pregrasp_pose")) or _zero,
        grasp_pose=pose_from_dict(data.get("grasp_pose")) or _zero,
        pull_pose=pose_from_dict(data.get("pull_pose")) or _zero,
        place_pose=pose_from_dict(data.get("place_pose")) or _zero,
        pregrasp_joint_trajectory=trajectory_from_dict(
            data.get("pregrasp_joint_trajectory") if isinstance(data.get("pregrasp_joint_trajectory"), dict) else None
        ),
        grasp_joint_trajectory=trajectory_from_dict(
            data.get("grasp_joint_trajectory") if isinstance(data.get("grasp_joint_trajectory"), dict) else None
        ),
        pull_joint_trajectory=trajectory_from_dict(
            data.get("pull_joint_trajectory") if isinstance(data.get("pull_joint_trajectory"), dict) else None
        ),
        place_joint_trajectory=trajectory_from_dict(
            data.get("place_joint_trajectory") if isinstance(data.get("place_joint_trajectory"), dict) else None
        ),
        home_joint_trajectory=trajectory_from_dict(
            data.get("home_joint_trajectory") if isinstance(data.get("home_joint_trajectory"), dict) else None
        ),
        plan_revision=int(data.get("plan_revision", 0) or 0),
        generated_at_sec=float(data["generated_at_sec"]) if data.get("generated_at_sec") is not None else None,
        planned_from_phase=_planned_from_phase_from_value(data.get("planned_from_phase")),
        producer_kind=_producer_kind_from_value(data.get("producer_kind")),
        producer_instance_id=(
            str(data["producer_instance_id"])
            if data.get("producer_instance_id") is not None else None
        ),
    )


def harvest_motion_plan_to_json(plan: HarvestMotionPlan) -> str:
    return json.dumps(harvest_motion_plan_to_dict(plan))


def harvest_motion_plan_from_json(json_str: str) -> HarvestMotionPlan:
    return harvest_motion_plan_from_dict(json.loads(json_str))
