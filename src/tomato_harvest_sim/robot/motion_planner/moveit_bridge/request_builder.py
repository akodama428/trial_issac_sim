from __future__ import annotations

from collections.abc import Callable

from tomato_harvest_sim.msg.contracts import JointStateSnapshot, Pose3D
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.config import (
    MoveItPlannerConfig,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.geometry import (
    moveit_link_target_pose_from_runtime_tool_pose,
    quaternion_from_pose,
)


def build_pose_goal_request(
    *,
    config: MoveItPlannerConfig,
    joint_state: JointStateSnapshot,
    base_frame_id: str,
    target_pose: Pose3D,
    joint_window: tuple[tuple[str, float, float], ...] | None = None,
    debug_log: Callable[[str], None] = lambda _message: None,
) -> object:
    """Build a GetMotionPlan request constrained by an end-effector pose."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import (
        BoundingVolume,
        Constraints,
        JointConstraint,
        OrientationConstraint,
        PositionConstraint,
    )
    from moveit_msgs.srv import GetMotionPlan
    from shape_msgs.msg import SolidPrimitive

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [config.position_tolerance_m]
    moveit_target_pose = moveit_link_target_pose_from_runtime_tool_pose(
        target_pose,
        link_to_tool_offset_m=config.moveit_link_to_runtime_tool_offset_m,
    )

    target_region_pose = Pose()
    target_region_pose.position.x = float(moveit_target_pose.x)
    target_region_pose.position.y = float(moveit_target_pose.y)
    target_region_pose.position.z = float(moveit_target_pose.z)
    target_region_pose.orientation.w = 1.0

    bounding_volume = BoundingVolume()
    bounding_volume.primitives = [primitive]
    bounding_volume.primitive_poses = [target_region_pose]

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = base_frame_id
    position_constraint.link_name = config.end_effector_link
    position_constraint.constraint_region = bounding_volume
    position_constraint.weight = 1.0

    goal_constraints = Constraints()
    goal_constraints.position_constraints = [position_constraint]
    if config.enforce_orientation_constraint:
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = base_frame_id
        orientation_constraint.link_name = config.end_effector_link
        orientation_constraint.orientation = quaternion_from_pose(
            moveit_target_pose
        )
        orientation_constraint.absolute_x_axis_tolerance = (
            config.orientation_tolerance_rad
        )
        orientation_constraint.absolute_y_axis_tolerance = (
            config.orientation_tolerance_rad
        )
        orientation_constraint.absolute_z_axis_tolerance = (
            config.orientation_tolerance_rad
        )
        orientation_constraint.weight = 1.0
        goal_constraints.orientation_constraints = [orientation_constraint]

    if joint_window is not None:
        for name, center, half_width in joint_window:
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = center
            constraint.tolerance_above = half_width
            constraint.tolerance_below = half_width
            constraint.weight = 1.0
            goal_constraints.joint_constraints.append(constraint)

    motion_plan_request = new_motion_plan_request(
        config=config,
        joint_state=joint_state,
        base_frame_id=base_frame_id,
    )
    motion_plan_request.goal_constraints = [goal_constraints]
    debug_log(
        "[MoveItBridge] request "
        f"ee_link={config.end_effector_link} "
        f"orientation_constraint={config.enforce_orientation_constraint} "
        f"runtime_target_xyz=({target_pose.x:.4f}, {target_pose.y:.4f}, "
        f"{target_pose.z:.4f}) "
        f"moveit_target_xyz=({moveit_target_pose.x:.4f}, "
        f"{moveit_target_pose.y:.4f}, {moveit_target_pose.z:.4f}) "
        f"start_q={joint_state.positions_rad}"
    )
    request = GetMotionPlan.Request()
    request.motion_plan_request = motion_plan_request
    return request


def build_joint_goal_request(
    *,
    config: MoveItPlannerConfig,
    joint_state: JointStateSnapshot,
    base_frame_id: str,
    goal_joint_state: JointStateSnapshot,
    debug_log: Callable[[str], None] = lambda _message: None,
) -> object:
    """Build a GetMotionPlan request constrained by known joint positions."""
    from moveit_msgs.msg import Constraints, JointConstraint
    from moveit_msgs.srv import GetMotionPlan

    goal_constraints = Constraints()
    for name, position in zip(
        goal_joint_state.joint_names, goal_joint_state.positions_rad
    ):
        constraint = JointConstraint()
        constraint.joint_name = name
        constraint.position = float(position)
        constraint.tolerance_above = config.joint_goal_tolerance_rad
        constraint.tolerance_below = config.joint_goal_tolerance_rad
        constraint.weight = 1.0
        goal_constraints.joint_constraints.append(constraint)

    motion_plan_request = new_motion_plan_request(
        config=config,
        joint_state=joint_state,
        base_frame_id=base_frame_id,
    )
    motion_plan_request.goal_constraints = [goal_constraints]
    debug_log(
        "[MoveItBridge] joint goal request "
        f"group={config.group_name} "
        f"goal_q={goal_joint_state.positions_rad} "
        f"start_q={joint_state.positions_rad}"
    )
    request = GetMotionPlan.Request()
    request.motion_plan_request = motion_plan_request
    return request


def new_motion_plan_request(
    *,
    config: MoveItPlannerConfig,
    joint_state: JointStateSnapshot,
    base_frame_id: str,
) -> object:
    """Build common workspace, start-state, and planner request fields."""
    from moveit_msgs.msg import MotionPlanRequest, RobotState, WorkspaceParameters
    from sensor_msgs.msg import JointState

    workspace = WorkspaceParameters()
    workspace.header.frame_id = base_frame_id
    workspace.min_corner.x = -1.5
    workspace.min_corner.y = -1.5
    workspace.min_corner.z = -0.2
    workspace.max_corner.x = 1.5
    workspace.max_corner.y = 1.5
    workspace.max_corner.z = 1.8

    start_joint_state = JointState()
    start_joint_state.name = list(joint_state.joint_names)
    start_joint_state.position = [
        float(position) for position in joint_state.positions_rad
    ]
    start_state = RobotState()
    start_state.joint_state = start_joint_state
    start_state.is_diff = False

    request = MotionPlanRequest()
    request.workspace_parameters = workspace
    request.start_state = start_state
    request.group_name = config.group_name
    request.num_planning_attempts = 4
    request.allowed_planning_time = config.allowed_planning_time_sec
    request.max_velocity_scaling_factor = 0.2
    request.max_acceleration_scaling_factor = 0.2
    return request
