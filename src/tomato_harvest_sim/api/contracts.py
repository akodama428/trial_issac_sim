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
    pregrasp_pose: Pose3D | None
    grasp_pose: Pose3D | None
    pull_pose: Pose3D | None
    place_pose: Pose3D | None
    grasp_result_reason: str | None
    motion_waypoints: tuple[Pose3D, ...] = ()
    active_waypoint_index: int | None = None
    motion_joint_trajectory: "JointTrajectory" | None = None


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


@dataclass(frozen=True)
class JointTrajectory:
    joint_names: tuple[str, ...]
    points: tuple[JointTrajectoryPoint, ...]


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
    waypoint_poses: tuple[Pose3D, ...] = ()
    joint_trajectory: JointTrajectory | None = None


@dataclass(frozen=True)
class ControlResult:
    command: ControlCommand
    accepted: bool
    scene_phase: ScenePhase
    robot_state: RobotRuntimeState
