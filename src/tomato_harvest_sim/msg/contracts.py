from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum


class ControlCommand(StrEnum):
    START = "start"
    STOP = "stop"
    RESET = "reset"


class ScenePhase(StrEnum):
    BOOTING = "booting"
    READY = "ready"
    RUNNING = "running"
    STOPPED = "stopped"


class RobotRuntimeState(StrEnum):
    BOOTING = "booting"
    READY = "ready"
    RUNNING = "running"
    STOPPED = "stopped"


class HarvestTaskPhase(StrEnum):
    IDLE = "idle"
    DETECTING = "detecting"
    TARGET_FOUND = "target_found"
    PLANNING = "planning"
    MOVING_TO_PREGRASP = "moving_to_pregrasp"
    PREGRASP_REACHED = "pregrasp_reached"
    MOVING_TO_GRASP = "moving_to_grasp"
    AT_GRASP = "at_grasp"
    GRASP_EVALUATION = "grasp_evaluation"
    DETACHING = "detaching"
    DETACHED = "detached"
    MOVING_TO_PLACE = "moving_to_place"
    PLACED = "placed"
    RETURNING_HOME = "returning_home"
    COMPLETE = "complete"
    FAILED = "failed"
    STOPPED = "stopped"


class TomatoStatus(StrEnum):
    ATTACHED = "attached"
    HELD = "held"
    DETACHED = "detached"
    PLACED = "placed"
    FALLEN = "fallen"


class PhaseId(StrEnum):
    MOVING_TO_PREGRASP = "moving_to_pregrasp"
    MOVING_TO_GRASP = "moving_to_grasp"
    PULL_TO_DETACH = "pull_to_detach"
    MOVING_TO_PLACE = "moving_to_place"
    RETURNING_HOME = "returning_home"


class PoseSemantics(StrEnum):
    TOOL_CENTER = "tool_center"
    GRASP_CENTER = "grasp_center"
    MOVEIT_LINK = "moveit_link"


class SuccessJudge(StrEnum):
    END_EFFECTOR_POSE = "end_effector_pose"
    JOINT_TRAJECTORY_COMPLETED = "joint_trajectory_completed"
    TOMATO_STATE = "tomato_state"


@dataclass(frozen=True)
class Pose3D:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class SceneSnapshot:
    phase: ScenePhase
    active_camera: str
    tomato_attached: bool
    tomato_status: TomatoStatus
    gripper_closed: bool
    robot_home: bool
    cycle_id: int
    robot_model: str
    robot_base_pose: Pose3D
    fixed_camera_pose: Pose3D
    hand_camera_pose: Pose3D
    branch_pose: Pose3D
    stem_pose: Pose3D
    tomato_pose: Pose3D
    tray_pose: Pose3D
    robot_tool_pose: Pose3D
    target_tool_pose: Pose3D | None
    grasp_result_reason: str | None
    active_phase_motion_plan: "PhaseMotionPlan | None" = None


@dataclass(frozen=True)
class CameraFrame:
    camera_name: str
    topic_name: str
    frame_id: str
    camera_pose: Pose3D
    target_world_pose: Pose3D


@dataclass(frozen=True)
class JointStateSnapshot:
    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...]


@dataclass(frozen=True)
class JointTrajectoryPoint:
    positions_rad: tuple[float, ...]
    time_from_start_sec: float = 0.0
    velocities_rad_s: tuple[float, ...] | None = None


@dataclass(frozen=True)
class JointTrajectory:
    joint_names: tuple[str, ...]
    points: tuple[JointTrajectoryPoint, ...]


@dataclass(frozen=True)
class SuccessPolicy:
    judge: SuccessJudge
    position_tolerance_m: float | None = None
    stable_steps: int = 1
    required_tomato_status: TomatoStatus | None = None


@dataclass(frozen=True)
class AbortPolicy:
    nominal_timeout_sec: float | None = None
    stall_timeout_sec: float | None = None
    min_progress_delta_m: float | None = None
    joint_path_tolerance_rad: float | None = None
    allow_replan: bool = True


@dataclass(frozen=True)
class PhaseExecutionIntent:
    phase_id: PhaseId
    phase_goal_pose: Pose3D | None
    pose_semantics: PoseSemantics
    success: SuccessPolicy
    abort: AbortPolicy


@dataclass(frozen=True)
class PhaseMotionPlan:
    phase_id: "PhaseId"
    phase_goal_pose: Pose3D | None
    active_waypoints: tuple[Pose3D, ...]
    joint_trajectory: JointTrajectory | None = None


@dataclass(frozen=True)
class ExecutionPhaseSpec:
    phase_id: PhaseId
    intent: PhaseExecutionIntent
    motion: PhaseMotionPlan


@dataclass(frozen=True)
class TfTreeSnapshot:
    robot_base_frame_id: str
    camera_frame_id: str
    target_frame_id: str
    robot_base_pose: Pose3D
    camera_pose: Pose3D
    target_pose: Pose3D


@dataclass(frozen=True)
class TargetEstimate:
    camera_name: str
    target_world_pose: Pose3D
    target_camera_pose: Pose3D
    confidence: float


@dataclass(frozen=True)
class HarvestMotionPlan:
    planner_name: str
    target_pose: Pose3D
    pregrasp_pose: Pose3D
    grasp_pose: Pose3D
    pull_pose: Pose3D
    place_pose: Pose3D
    pregrasp_waypoints: tuple[Pose3D, ...] = ()
    grasp_waypoints: tuple[Pose3D, ...] = ()
    pull_waypoints: tuple[Pose3D, ...] = ()
    place_waypoints: tuple[Pose3D, ...] = ()
    pregrasp_joint_trajectory: JointTrajectory | None = None
    grasp_joint_trajectory: JointTrajectory | None = None
    pull_joint_trajectory: JointTrajectory | None = None
    place_joint_trajectory: JointTrajectory | None = None
    planning_scene_object_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class MotionCommand:
    command_name: str
    planner_name: str
    target_pose: Pose3D | None = None
    gripper_closed: bool | None = None
    phase_motion_plan: PhaseMotionPlan | None = None


@dataclass(frozen=True)
class ControlResult:
    command: ControlCommand
    accepted: bool
    scene_phase: ScenePhase
    robot_state: RobotRuntimeState
