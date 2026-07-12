"""ROS2 トピック名定数。robot / simulator 共通で使用する。"""
from __future__ import annotations

CONTROL_TOPIC = "/tomato_harvest/control"
SCENE_SNAPSHOT_TOPIC = "/tomato_harvest/scene_snapshot"
MOTION_COMMAND_TOPIC = "/tomato_harvest/motion_command"
MOTION_METADATA_TOPIC = "/tomato_harvest/motion_metadata"
TARGET_ESTIMATE_TOPIC = "/tomato_harvest/target_estimate"
PHASE_TOPIC = "/tomato_harvest/phase"
HARVEST_MOTION_PLAN_TOPIC = "/tomato_harvest/harvest_motion_plan"
EXECUTION_STATUS_TOPIC = "/tomato_harvest/execution_status"
TRAJECTORY_STATUS_TOPIC = "/tomato_harvest/trajectory_status"
HYBRID_PLANNING_EVENT_TOPIC = "/tomato_harvest/hybrid_planning_event"
FIXED_CAMERA_TOPIC = "/camera/fixed/image_raw"
HAND_CAMERA_TOPIC = "/camera/hand/image_raw"
JOINT_STATES_TOPIC = "/joint_states"
FOLLOW_JOINT_TRAJECTORY_ACTION = "/tomato_harvest/follow_joint_trajectory"

DEFAULT_JOINT_NAMES = (
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
)
DEFAULT_JOINT_POSITIONS_RAD = (0.0, -0.4, 0.0, -2.1, 0.0, 1.7, 0.8)
