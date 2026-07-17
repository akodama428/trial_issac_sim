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


def home_joint_state():
    """home復帰の目標となる既知の関節構成を返す (Issue #32)。

    motion_commandの直行home軌道と、returning_homeのsuffix replanが
    同じ終端構成を共有するための単一の定義点。

    Returns:
        DEFAULT_JOINT_NAMES / DEFAULT_JOINT_POSITIONS_RAD からなる
        JointStateSnapshot。
    """
    from tomato_harvest_sim.msg.contracts import JointStateSnapshot

    return JointStateSnapshot(
        joint_names=DEFAULT_JOINT_NAMES,
        positions_rad=DEFAULT_JOINT_POSITIONS_RAD,
    )
