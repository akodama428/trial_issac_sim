from __future__ import annotations

import math
import os
from dataclasses import dataclass, replace

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
    Pose3D,
    SceneSnapshot,
    TargetEstimate,
    TfTreeSnapshot,
)
from tomato_harvest_sim.robot.msg.planner import MotionPlanner, MoveIt2PlannerBridge, MoveIt2PlanningResult, PlannerBackendInfo
from tomato_harvest_sim.robot.motion_planner.phase_suffix_replan import (
    SUFFIX_TRAJECTORY_FIELD_BY_PHASE,
)
from tomato_harvest_sim.robot.motion_planner.pregrasp_planner import MoveItStylePreGraspPlanner
from tomato_harvest_sim.robot.motion_planner.ros_python import ensure_ros_python_modules_available


def _moveit2_python_available() -> bool:
    return ensure_ros_python_modules_available("rclpy", "moveit_msgs")


@dataclass(frozen=True)
class _TomatoPlanningSceneOps:
    add_world_tomato: bool
    remove_world_tomato: bool
    add_attached_tomato: bool
    remove_attached_tomato: bool

class MoveIt2ServiceBridgePlanner(MotionPlanner):
    """MoveIt2-aware planner that applies a planning scene and returns joint trajectories."""

    def __init__(
        self,
        *,
        grasp_lateral_offset_m: float = 0.0,
        bridge: MoveIt2PlannerBridge | None = None,
    ) -> None:
        self._fallback = MoveItStylePreGraspPlanner(grasp_lateral_offset_m=grasp_lateral_offset_m)
        self._bridge = bridge or Ros2MoveIt2PlannerBridge()

    def plan(
        self,
        target_estimate: TargetEstimate,
        joint_state: JointStateSnapshot,
        tf_tree: TfTreeSnapshot,
        scene_snapshot: SceneSnapshot,
    ) -> HarvestMotionPlan:
        fallback_plan = self._fallback.plan(target_estimate, joint_state, tf_tree, scene_snapshot)
        result = self._bridge.plan_phase_trajectories(
            joint_state=joint_state,
            tf_tree=tf_tree,
            scene_snapshot=scene_snapshot,
            plan=fallback_plan,
        )
        # MoveIt2 が None を返したフェーズはフォールバック（幾何学的）軌道で補完する
        return replace(
            fallback_plan,
            planner_name=result.backend_name,
            pregrasp_joint_trajectory=result.pregrasp_joint_trajectory or fallback_plan.pregrasp_joint_trajectory,
            grasp_joint_trajectory=result.grasp_joint_trajectory or fallback_plan.grasp_joint_trajectory,
            pull_joint_trajectory=result.pull_joint_trajectory or fallback_plan.pull_joint_trajectory,
            place_joint_trajectory=result.place_joint_trajectory or fallback_plan.place_joint_trajectory,
            planning_scene_object_ids=result.planning_scene_object_ids or fallback_plan.planning_scene_object_ids,
        )

    def plan_from_phase(
        self,
        phase: HarvestTaskPhase,
        prior_plan: HarvestMotionPlan,
        joint_state: JointStateSnapshot,
        tf_tree: TfTreeSnapshot,
        scene_snapshot: SceneSnapshot,
    ) -> HarvestMotionPlan | None:
        """実行中phaseの残区間だけを最新joint stateから再計画する (Issue #12)。

        フルチェーン（pregrasp→grasp→pull→place）を経由せず、phaseに対応する
        trajectory 1区間のみを差し替えたplanを返す。差し替え区間の選択を
        ここ（planner adapter）に寄せることで、node側のphase分岐を増やさない。

        Args:
            phase: 実行中のharvest phase。
            prior_plan: 現在採用中のplan。差し替えないphaseの軌道はこれを保持する。
            joint_state: 再計画の起点にする最新joint state。
            tf_tree: 座標系snapshot。
            scene_snapshot: planning scene更新に使うscene snapshot。

        Returns:
            phaseの残区間だけ更新したplan。suffix replan対象外のphase、または
            計画失敗時はNone。
        """
        field = SUFFIX_TRAJECTORY_FIELD_BY_PHASE.get(phase)
        if field is None:
            return None
        plan_suffix_fn = getattr(self._bridge, "plan_suffix_trajectory", None)
        if plan_suffix_fn is None:
            return None
        result = None
        trajectory = None
        for _attempt in range(3):
            result = plan_suffix_fn(
                phase=phase,
                joint_state=joint_state,
                tf_tree=tf_tree,
                scene_snapshot=scene_snapshot,
                plan=prior_plan,
            )
            trajectory = getattr(result, field)
            if result.success and trajectory is not None:
                break
        if result is None or not result.success or trajectory is None:
            return None
        return replace(prior_plan, planner_name=result.backend_name, **{field: trajectory})


class Ros2MoveIt2PlannerBridge:
    MOVEIT_LINK_TO_RUNTIME_TOOL_OFFSET_M = (0.0, 0.0, 0.0584)
    TRAY_INNER_SIZE_M = (0.22, 0.16, 0.05)
    TRAY_WALL_THICKNESS_M = 0.012
    BRANCH_SIZE_M = (0.18, 0.02, 0.02)
    STEM_SIZE_M = (0.008, 0.008, 0.06)
    ATTACHED_TOMATO_RADIUS_M = 0.01
    ATTACHED_TOMATO_OFFSET_M = (0.0, 0.0, 0.1034)
    NOOP_TRAJECTORY_TOLERANCE_RAD = 1e-3

    def __init__(
        self,
        *,
        service_name: str | None = None,
        scene_service_name: str | None = None,
        group_name: str | None = None,
        end_effector_link: str | None = None,
        planning_timeout_sec: float | None = None,
        allowed_planning_time_sec: float | None = None,
        position_tolerance_m: float = 0.01,
        orientation_tolerance_rad: float = 0.10,
    ) -> None:
        self._service_name = service_name or os.environ.get("TOMATO_HARVEST_MOVEIT_SERVICE", "/plan_kinematic_path")
        self._scene_service_name = scene_service_name or os.environ.get(
            "TOMATO_HARVEST_MOVEIT_SCENE_SERVICE",
            "/apply_planning_scene",
        )
        self._group_name = group_name or os.environ.get("TOMATO_HARVEST_MOVEIT_GROUP", "panda_arm")
        self._end_effector_link = end_effector_link or os.environ.get("TOMATO_HARVEST_MOVEIT_EE_LINK", "panda_hand")
        self._planning_timeout_sec = planning_timeout_sec or float(
            os.environ.get("TOMATO_HARVEST_MOVEIT_SERVICE_TIMEOUT_SEC", "1.50")
        )
        self._allowed_planning_time_sec = allowed_planning_time_sec or float(
            os.environ.get("TOMATO_HARVEST_MOVEIT_ALLOWED_PLANNING_TIME_SEC", "1.00")
        )
        self._position_tolerance_m = float(os.environ.get("TOMATO_HARVEST_MOVEIT_POSITION_TOLERANCE_M", position_tolerance_m))
        self._orientation_tolerance_rad = float(
            os.environ.get("TOMATO_HARVEST_MOVEIT_ORIENTATION_TOLERANCE_RAD", orientation_tolerance_rad)
        )
        self._enforce_orientation_constraint = os.environ.get(
            "TOMATO_HARVEST_MOVEIT_ENFORCE_ORIENTATION",
            "1",
        ).strip() not in {"0", "false", "False"}
        self._debug_enabled = os.environ.get(
            "TOMATO_HARVEST_DEBUG_MOVEIT",
            "",
        ).strip() not in {"", "0", "false", "False"}
        self._clients = None
        self._planning_scene_has_attached_tomato = False

    def plan_phase_trajectories(
        self,
        *,
        joint_state: JointStateSnapshot,
        tf_tree: TfTreeSnapshot,
        scene_snapshot: SceneSnapshot,
        plan: HarvestMotionPlan,
    ) -> MoveIt2PlanningResult:
        if not _moveit2_python_available():
            return MoveIt2PlanningResult(
                success=False,
                backend_name="moveit2_service_bridge_fallback",
                reason="moveit2_python_unavailable",
            )

        clients = self._require_clients()
        if clients is None:
            return MoveIt2PlanningResult(
                success=False,
                backend_name="moveit2_service_bridge_fallback",
                reason="service_client_unavailable",
            )

        if not clients.wait_for_services(timeout_sec=self._planning_timeout_sec):
            return MoveIt2PlanningResult(
                success=False,
                backend_name="moveit2_service_bridge_fallback",
                reason="service_unavailable",
            )

        base_frame_id = tf_tree.robot_base_frame_id
        planning_scene_object_ids = _planning_scene_object_ids()
        current_joint_state = _clamp_joint_state_to_bounds(joint_state)

        pregrasp_trajectory = self._plan_phase(
            clients=clients,
            joint_state=current_joint_state,
            base_frame_id=base_frame_id,
            scene_snapshot=scene_snapshot,
            target_pose=plan.pregrasp_pose,
            attach_tomato=False,
        )
        if pregrasp_trajectory is None:
            return self._fallback_result("pregrasp_plan_failed")
        current_joint_state = _joint_state_from_trajectory(pregrasp_trajectory)

        grasp_trajectory = self._plan_phase(
            clients=clients,
            joint_state=current_joint_state,
            base_frame_id=base_frame_id,
            scene_snapshot=scene_snapshot,
            target_pose=plan.grasp_pose,
            attach_tomato=False,
        )
        if grasp_trajectory is None:
            return self._fallback_result("grasp_plan_failed")
        current_joint_state = _joint_state_from_trajectory(grasp_trajectory)

        pull_trajectory = self._plan_phase(
            clients=clients,
            joint_state=current_joint_state,
            base_frame_id=base_frame_id,
            scene_snapshot=scene_snapshot,
            target_pose=plan.pull_pose,
            attach_tomato=True,
        )
        if pull_trajectory is None:
            return self._fallback_result("pull_plan_failed")
        current_joint_state = _joint_state_from_trajectory(pull_trajectory)

        pre_place_pose = plan.place_waypoints[0] if plan.place_waypoints else None
        if pre_place_pose is not None:
            approach_trajectory = self._plan_phase(
                clients=clients,
                joint_state=current_joint_state,
                base_frame_id=base_frame_id,
                scene_snapshot=scene_snapshot,
                target_pose=pre_place_pose,
                attach_tomato=True,
            )
            if approach_trajectory is None:
                return MoveIt2PlanningResult(
                    success=False,
                    backend_name="moveit2_service_bridge_partial",
                    reason="pre_place_plan_failed",
                    pregrasp_joint_trajectory=pregrasp_trajectory,
                    grasp_joint_trajectory=grasp_trajectory,
                    pull_joint_trajectory=pull_trajectory,
                    place_joint_trajectory=None,
                    planning_scene_object_ids=planning_scene_object_ids,
                )
            current_joint_state = _joint_state_from_trajectory(approach_trajectory)
        else:
            approach_trajectory = None

        place_trajectory = self._plan_phase(
            clients=clients,
            joint_state=current_joint_state,
            base_frame_id=base_frame_id,
            scene_snapshot=scene_snapshot,
            target_pose=plan.place_pose,
            attach_tomato=True,
        )
        if place_trajectory is None:
            # place は失敗したが pregrasp/grasp/pull は成功済み → 部分結果を返す
            # MoveIt2ServiceBridgePlanner.plan() がフォールバック軌道で補完する
            return MoveIt2PlanningResult(
                success=False,
                backend_name="moveit2_service_bridge_partial",
                reason="place_plan_failed",
                pregrasp_joint_trajectory=pregrasp_trajectory,
                grasp_joint_trajectory=grasp_trajectory,
                pull_joint_trajectory=pull_trajectory,
                place_joint_trajectory=None,
                planning_scene_object_ids=planning_scene_object_ids,
            )

        if approach_trajectory is not None:
            place_trajectory = _concatenate_trajectories(approach_trajectory, place_trajectory)

        return MoveIt2PlanningResult(
            success=True,
            backend_name="moveit2_service_bridge",
            reason="service_ok",
            pregrasp_joint_trajectory=pregrasp_trajectory,
            grasp_joint_trajectory=grasp_trajectory,
            pull_joint_trajectory=pull_trajectory,
            place_joint_trajectory=place_trajectory,
            planning_scene_object_ids=planning_scene_object_ids,
        )

    def plan_suffix_trajectory(
        self,
        *,
        phase: HarvestTaskPhase,
        joint_state: JointStateSnapshot,
        tf_tree: TfTreeSnapshot,
        scene_snapshot: SceneSnapshot,
        plan: HarvestMotionPlan,
    ) -> MoveIt2PlanningResult:
        """ロボットの現在位置から、実行中phaseの残区間軌道のみを計画する。

        自由空間phaseのリプランで使用し、フルチェーン再計画による位置ズレと
        完了済み区間の無駄な作り直しを防ぐ。phaseごとの目標poseとplanning scene
        設定（トマト把持前/後）の違いはここで吸収する。

        Args:
            phase: 実行中のharvest phase。suffix replan対象外なら失敗を返す。
            joint_state: 再計画の起点にする最新joint state。
            tf_tree: 座標系snapshot。
            scene_snapshot: planning scene更新に使うscene snapshot。
            plan: 目標poseの参照元にする現在採用中のplan。

        Returns:
            成功時はphaseに対応するtrajectoryだけを持つMoveIt2PlanningResult。
        """
        if not _moveit2_python_available():
            return self._fallback_result("moveit2_python_unavailable")

        clients = self._require_clients()
        if clients is None:
            return self._fallback_result("service_client_unavailable")

        if not clients.wait_for_services(timeout_sec=self._planning_timeout_sec):
            return self._fallback_result("service_unavailable")

        base_frame_id = tf_tree.robot_base_frame_id
        current_joint_state = _clamp_joint_state_to_bounds(joint_state)

        # 把持前phaseは単一区間の再計画で完結する。トマトはまだworld側にある。
        if phase is HarvestTaskPhase.MOVING_TO_PREGRASP:
            pregrasp_trajectory = self._plan_phase(
                clients=clients,
                joint_state=current_joint_state,
                base_frame_id=base_frame_id,
                scene_snapshot=scene_snapshot,
                target_pose=plan.pregrasp_pose,
                attach_tomato=False,
            )
            if pregrasp_trajectory is None:
                return self._fallback_result("pregrasp_replan_failed")
            return MoveIt2PlanningResult(
                success=True,
                backend_name="moveit2_service_bridge",
                reason="service_ok",
                pregrasp_joint_trajectory=pregrasp_trajectory,
            )

        if phase is HarvestTaskPhase.MOVING_TO_GRASP:
            grasp_trajectory = self._plan_phase(
                clients=clients,
                joint_state=current_joint_state,
                base_frame_id=base_frame_id,
                scene_snapshot=scene_snapshot,
                target_pose=plan.grasp_pose,
                attach_tomato=False,
            )
            if grasp_trajectory is None:
                return self._fallback_result("grasp_replan_failed")
            return MoveIt2PlanningResult(
                success=True,
                backend_name="moveit2_service_bridge",
                reason="service_ok",
                grasp_joint_trajectory=grasp_trajectory,
            )

        if phase is HarvestTaskPhase.MOVING_TO_PLACE:
            return self._plan_place_suffix(
                clients=clients,
                joint_state=current_joint_state,
                base_frame_id=base_frame_id,
                scene_snapshot=scene_snapshot,
                plan=plan,
            )

        return self._fallback_result("unsupported_suffix_phase")

    def _plan_place_suffix(
        self,
        *,
        clients: "_Ros2MoveIt2Clients",
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        scene_snapshot: SceneSnapshot,
        plan: HarvestMotionPlan,
    ) -> MoveIt2PlanningResult:
        """approach waypoint経由でplaceまでの残区間を計画する。トマトは把持中。"""
        current_joint_state = joint_state
        pre_place_pose = plan.place_waypoints[0] if plan.place_waypoints else None
        if pre_place_pose is not None:
            approach_trajectory = self._plan_phase(
                clients=clients,
                joint_state=current_joint_state,
                base_frame_id=base_frame_id,
                scene_snapshot=scene_snapshot,
                target_pose=pre_place_pose,
                attach_tomato=True,
            )
            if approach_trajectory is None:
                return self._fallback_result("pre_place_replan_failed")
            current_joint_state = _joint_state_from_trajectory(approach_trajectory)
        else:
            approach_trajectory = None

        place_trajectory = self._plan_phase(
            clients=clients,
            joint_state=current_joint_state,
            base_frame_id=base_frame_id,
            scene_snapshot=scene_snapshot,
            target_pose=plan.place_pose,
            attach_tomato=True,
        )
        if place_trajectory is None:
            return self._fallback_result("place_replan_failed")

        if approach_trajectory is not None:
            place_trajectory = _concatenate_trajectories(approach_trajectory, place_trajectory)

        return MoveIt2PlanningResult(
            success=True,
            backend_name="moveit2_service_bridge",
            reason="service_ok",
            place_joint_trajectory=place_trajectory,
        )

    def _fallback_result(self, reason: str) -> MoveIt2PlanningResult:
        return MoveIt2PlanningResult(
            success=False,
            backend_name="moveit2_service_bridge_fallback",
            reason=reason,
        )

    def _plan_phase(
        self,
        *,
        clients: "_Ros2MoveIt2Clients",
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        scene_snapshot: SceneSnapshot,
        target_pose: Pose3D,
        attach_tomato: bool,
    ) -> JointTrajectory | None:
        apply_request = _build_planning_scene_request(
            scene_snapshot=scene_snapshot,
            base_frame_id=base_frame_id,
            end_effector_link=self._end_effector_link,
            tomato_ops=_tomato_planning_scene_ops(
                attach_tomato=attach_tomato,
                planning_scene_has_attached_tomato=self._planning_scene_has_attached_tomato,
            ),
            tray_inner_size_m=self.TRAY_INNER_SIZE_M,
            tray_wall_thickness_m=self.TRAY_WALL_THICKNESS_M,
            branch_size_m=self.BRANCH_SIZE_M,
            stem_size_m=self.STEM_SIZE_M,
            attached_tomato_radius_m=self.ATTACHED_TOMATO_RADIUS_M,
            attached_tomato_offset_m=self.ATTACHED_TOMATO_OFFSET_M,
        )
        if not clients.apply_planning_scene(apply_request, timeout_sec=self._planning_timeout_sec):
            return None
        self._planning_scene_has_attached_tomato = attach_tomato
        request = self._build_motion_plan_request(
            joint_state=joint_state,
            base_frame_id=base_frame_id,
            target_pose=target_pose,
        )
        trajectory = clients.plan_to_pose(request, timeout_sec=self._planning_timeout_sec)
        if trajectory is None:
            self._debug_log(
                f"[MoveItBridge] phase planning failed: ee_link={self._end_effector_link} "
                f"target_xyz=({target_pose.x:.4f}, {target_pose.y:.4f}, {target_pose.z:.4f})"
            )
            return None
        if _trajectory_is_noop(
            trajectory,
            start_joint_state=joint_state,
            tolerance_rad=self.NOOP_TRAJECTORY_TOLERANCE_RAD,
        ):
            self._debug_log(
                "[MoveItBridge] rejecting no-op trajectory and falling back to geometric execution. "
                f"ee_link={self._end_effector_link} "
                f"target_xyz=({target_pose.x:.4f}, {target_pose.y:.4f}, {target_pose.z:.4f}) "
                f"start_q={joint_state.positions_rad} "
                f"end_q={trajectory.points[-1].positions_rad}"
            )
            return None
        self._debug_log(
            "[MoveItBridge] accepted trajectory "
            f"points={len(trajectory.points)} "
            f"ee_link={self._end_effector_link} "
            f"target_xyz=({target_pose.x:.4f}, {target_pose.y:.4f}, {target_pose.z:.4f}) "
            f"end_q={trajectory.points[-1].positions_rad}"
        )
        return trajectory

    def _build_motion_plan_request(
        self,
        *,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        target_pose: Pose3D,
    ) -> object:
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import (
            BoundingVolume,
            Constraints,
            MotionPlanRequest,
            OrientationConstraint,
            PositionConstraint,
            RobotState,
            WorkspaceParameters,
        )
        from moveit_msgs.srv import GetMotionPlan
        from sensor_msgs.msg import JointState
        from shape_msgs.msg import SolidPrimitive

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self._position_tolerance_m]

        moveit_target_pose = _moveit_link_target_pose_from_runtime_tool_pose(
            target_pose,
            link_to_tool_offset_m=self.MOVEIT_LINK_TO_RUNTIME_TOOL_OFFSET_M,
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
        position_constraint.link_name = self._end_effector_link
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [position_constraint]
        if self._enforce_orientation_constraint:
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = base_frame_id
            orientation_constraint.link_name = self._end_effector_link
            orientation_constraint.orientation = _quaternion_from_pose(moveit_target_pose)
            orientation_constraint.absolute_x_axis_tolerance = self._orientation_tolerance_rad
            orientation_constraint.absolute_y_axis_tolerance = self._orientation_tolerance_rad
            orientation_constraint.absolute_z_axis_tolerance = self._orientation_tolerance_rad
            orientation_constraint.weight = 1.0
            goal_constraints.orientation_constraints = [orientation_constraint]

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
        start_joint_state.position = [float(position) for position in joint_state.positions_rad]

        start_state = RobotState()
        start_state.joint_state = start_joint_state
        start_state.is_diff = False

        motion_plan_request = MotionPlanRequest()
        motion_plan_request.workspace_parameters = workspace
        motion_plan_request.start_state = start_state
        motion_plan_request.goal_constraints = [goal_constraints]
        motion_plan_request.group_name = self._group_name
        motion_plan_request.num_planning_attempts = 4
        motion_plan_request.allowed_planning_time = self._allowed_planning_time_sec
        motion_plan_request.max_velocity_scaling_factor = 0.2
        motion_plan_request.max_acceleration_scaling_factor = 0.2
        self._debug_log(
            "[MoveItBridge] request "
            f"ee_link={self._end_effector_link} "
            f"orientation_constraint={self._enforce_orientation_constraint} "
            f"runtime_target_xyz=({target_pose.x:.4f}, {target_pose.y:.4f}, {target_pose.z:.4f}) "
            f"moveit_target_xyz=({moveit_target_pose.x:.4f}, {moveit_target_pose.y:.4f}, {moveit_target_pose.z:.4f}) "
            f"start_q={joint_state.positions_rad}"
        )

        request = GetMotionPlan.Request()
        request.motion_plan_request = motion_plan_request
        return request

    def _require_clients(self) -> "_Ros2MoveIt2Clients | None":
        if self._clients is not None:
            return self._clients
        try:
            self._clients = _Ros2MoveIt2Clients(
                motion_plan_service_name=self._service_name,
                planning_scene_service_name=self._scene_service_name,
            )
        except Exception:
            self._clients = None
        return self._clients

    def _debug_log(self, message: str) -> None:
        if self._debug_enabled:
            print(message, flush=True)


class _Ros2MoveIt2Clients:
    def __init__(
        self,
        *,
        motion_plan_service_name: str,
        planning_scene_service_name: str,
    ) -> None:
        import rclpy
        from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan
        from rclpy.executors import SingleThreadedExecutor

        self._rclpy = rclpy
        if not self._rclpy.ok():
            self._rclpy.init(args=None)
        self._node = self._rclpy.create_node("tomato_harvest_moveit_bridge")
        self._motion_plan_client = self._node.create_client(GetMotionPlan, motion_plan_service_name)
        self._planning_scene_client = self._node.create_client(ApplyPlanningScene, planning_scene_service_name)
        # robot_node の executor とは独立した専用 executor で spin する。
        # rclpy.spin_until_future_complete() はデフォルト executor を使うため、
        # robot_node の rclpy.spin() 内から呼ぶと "Executor is already spinning" になる。
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

    def _spin_until_done(self, future: object, timeout_sec: float) -> None:
        self._executor.spin_until_future_complete(future, timeout_sec=timeout_sec)

    def wait_for_services(self, *, timeout_sec: float) -> bool:
        motion_ready = bool(self._motion_plan_client.wait_for_service(timeout_sec=timeout_sec))
        if not motion_ready:
            return False
        return bool(self._planning_scene_client.wait_for_service(timeout_sec=timeout_sec))

    def apply_planning_scene(self, request: object, *, timeout_sec: float) -> bool:
        future = self._planning_scene_client.call_async(request)
        self._spin_until_done(future, timeout_sec=timeout_sec)
        if not future.done():
            return False
        response = future.result()
        if response is None:
            return False
        return bool(getattr(response, "success", False))

    def plan_to_pose(self, request: object, *, timeout_sec: float) -> JointTrajectory | None:
        future = self._motion_plan_client.call_async(request)
        self._spin_until_done(future, timeout_sec=timeout_sec)
        if not future.done():
            return None
        response = future.result()
        if response is None:
            return None
        error_code = int(response.motion_plan_response.error_code.val)
        if error_code != 1:
            print(f"[MoveItBridge] motion plan service returned error_code={error_code}.", flush=True)
            return None
        robot_trajectory = response.motion_plan_response.trajectory
        joint_trajectory = getattr(robot_trajectory, "joint_trajectory", None)
        if joint_trajectory is None:
            print("[MoveItBridge] motion plan response had no joint_trajectory.", flush=True)
            return None
        planned_trajectory = _joint_trajectory_from_msg(joint_trajectory)
        if planned_trajectory is not None:
            has_velocities = any(p.velocities_rad_s is not None for p in planned_trajectory.points)
            first_vel = planned_trajectory.points[0].velocities_rad_s if planned_trajectory.points else None
            last_vel = planned_trajectory.points[-1].velocities_rad_s if planned_trajectory.points else None
            print(
                "[MoveItBridge] motion plan response "
                f"points={len(planned_trajectory.points)} "
                f"has_velocities={has_velocities} "
                f"first_vel={first_vel} "
                f"last_vel={last_vel} "
                f"joint_names={planned_trajectory.joint_names}",
                flush=True,
            )
            return planned_trajectory
        print("[MoveItBridge] motion plan response had an empty joint trajectory.", flush=True)
        return None


def _joint_trajectory_from_msg(joint_trajectory_msg: object) -> JointTrajectory | None:
    joint_names = tuple(getattr(joint_trajectory_msg, "joint_names", ()))
    points_msg = getattr(joint_trajectory_msg, "points", ())
    if not joint_names or not points_msg:
        return None
    points: list[JointTrajectoryPoint] = []
    for point in points_msg:
        positions = tuple(float(value) for value in getattr(point, "positions", ()))
        if not positions:
            return None
        duration = getattr(point, "time_from_start", None)
        time_from_start_sec = 0.0
        if duration is not None:
            time_from_start_sec = float(getattr(duration, "sec", 0)) + float(getattr(duration, "nanosec", 0)) / 1_000_000_000.0
        velocities_msg = getattr(point, "velocities", ())
        velocities = tuple(float(v) for v in velocities_msg) if velocities_msg else None
        points.append(JointTrajectoryPoint(positions_rad=positions, time_from_start_sec=time_from_start_sec, velocities_rad_s=velocities))
    return JointTrajectory(joint_names=joint_names, points=tuple(points))


def _joint_trajectory_from_request_start_state(request: object) -> JointTrajectory | None:
    motion_plan_request = getattr(request, "motion_plan_request", None)
    if motion_plan_request is None:
        return None
    start_state = getattr(motion_plan_request, "start_state", None)
    if start_state is None:
        return None
    joint_state = getattr(start_state, "joint_state", None)
    if joint_state is None:
        return None
    joint_names = tuple(str(name) for name in getattr(joint_state, "name", ()))
    positions = tuple(float(value) for value in getattr(joint_state, "position", ()))
    if not joint_names or not positions:
        return None
    return JointTrajectory(
        joint_names=joint_names,
        points=(JointTrajectoryPoint(positions_rad=positions, time_from_start_sec=0.0),),
    )


def _trajectory_is_noop(
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
        for end, start in zip(end_positions, start_joint_state.positions_rad, strict=True)
    ) <= tolerance_rad


_PANDA_JOINT_BOUNDS: dict[str, tuple[float, float]] = {
    "panda_joint1": (-2.8973, 2.8973),
    "panda_joint2": (-1.7628, 1.7628),
    "panda_joint3": (-2.8973, 2.8973),
    "panda_joint4": (-3.0718, -0.069),
    "panda_joint5": (-2.8973, 2.8973),
    "panda_joint6": (-0.017, 3.7525),
    "panda_joint7": (-2.8973, 2.8973),
}


def _clamp_joint_state_to_bounds(joint_state: JointStateSnapshot) -> JointStateSnapshot:
    """MoveIt2 に送る前に関節位置を URDF 境界内にクランプする。

    Isaac Sim はロボットを全関節 0.0 rad で初期化するが、
    panda_joint4 の上限は -0.069 rad であり 0.0 は範囲外になる。
    MoveIt2 の CheckStartStateBounds がプランを拒否しないよう、
    境界違反があれば最も近い有効値へスナップする。
    """
    clamped = list(joint_state.positions_rad)
    for i, name in enumerate(joint_state.joint_names):
        if i >= len(clamped):
            break
        bounds = _PANDA_JOINT_BOUNDS.get(name)
        if bounds is None:
            continue
        lo, hi = bounds
        if clamped[i] < lo:
            clamped[i] = lo
        elif clamped[i] > hi:
            clamped[i] = hi
    return JointStateSnapshot(joint_names=joint_state.joint_names, positions_rad=tuple(clamped))


def _joint_state_from_trajectory(trajectory: JointTrajectory) -> JointStateSnapshot:
    last_point = trajectory.points[-1]
    return JointStateSnapshot(joint_names=trajectory.joint_names, positions_rad=last_point.positions_rad)


def _concatenate_trajectories(traj1: JointTrajectory, traj2: JointTrajectory) -> JointTrajectory:
    if not traj1.points:
        return traj2
    if not traj2.points:
        return traj1
    time_offset = traj1.points[-1].time_from_start_sec
    # traj2 の最初の点が t=0.0 の場合、time_offset を加算すると traj1 の最後の点と同じ
    # タイムスタンプになり JTC が拒否する。t=0.0 の点はスタート位置の重複なのでスキップする。
    traj2_points = traj2.points[1:] if traj2.points[0].time_from_start_sec == 0.0 else traj2.points
    if not traj2_points:
        return traj1
    shifted = tuple(
        JointTrajectoryPoint(
            positions_rad=p.positions_rad,
            time_from_start_sec=p.time_from_start_sec + time_offset,
        )
        for p in traj2_points
    )
    return JointTrajectory(joint_names=traj1.joint_names, points=traj1.points + shifted)


def _build_planning_scene_request(
    *,
    scene_snapshot: SceneSnapshot,
    base_frame_id: str,
    end_effector_link: str,
    tomato_ops: _TomatoPlanningSceneOps,
    tray_inner_size_m: tuple[float, float, float],
    tray_wall_thickness_m: float,
    branch_size_m: tuple[float, float, float],
    stem_size_m: tuple[float, float, float],
    attached_tomato_radius_m: float,
    attached_tomato_offset_m: tuple[float, float, float],
) -> object:
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, RobotState
    from moveit_msgs.srv import ApplyPlanningScene
    from shape_msgs.msg import SolidPrimitive

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state = RobotState()
    scene.robot_state.is_diff = True

    scene.world.collision_objects = [
        _box_collision_object(
            object_id="tomato_branch",
            frame_id=base_frame_id,
            pose=scene_snapshot.branch_pose,
            size_xyz=branch_size_m,
        ),
        _box_collision_object(
            object_id="tomato_stem",
            frame_id=base_frame_id,
            pose=scene_snapshot.stem_pose,
            size_xyz=stem_size_m,
        ),
    ]
    scene.world.collision_objects.extend(
        _tray_collision_objects(
            frame_id=base_frame_id,
            tray_pose=scene_snapshot.tray_pose,
            tray_inner_size_m=tray_inner_size_m,
            tray_wall_thickness_m=tray_wall_thickness_m,
        )
    )

    if tomato_ops.add_world_tomato:
        scene.world.collision_objects.append(
            _sphere_collision_object(
                object_id="target_tomato",
                frame_id=base_frame_id,
                pose=scene_snapshot.tomato_pose,
                radius_m=attached_tomato_radius_m,
            )
        )

    if tomato_ops.remove_world_tomato:
        scene.world.collision_objects.append(
            _remove_collision_object(
                object_id="target_tomato",
                frame_id=base_frame_id,
            )
        )

    attached_collision_objects: list[object] = []
    if tomato_ops.add_attached_tomato:
        attached = AttachedCollisionObject()
        attached.link_name = end_effector_link
        attached.object = _sphere_collision_object(
            object_id="target_tomato",
            frame_id=end_effector_link,
            pose=Pose3D(
                attached_tomato_offset_m[0],
                attached_tomato_offset_m[1],
                attached_tomato_offset_m[2],
                0.0,
                0.0,
                0.0,
            ),
            radius_m=attached_tomato_radius_m,
        )
        attached_collision_objects.append(attached)

    if tomato_ops.remove_attached_tomato:
        remove_attached = AttachedCollisionObject()
        remove_attached.link_name = end_effector_link
        remove_attached.object = CollisionObject()
        remove_attached.object.id = "target_tomato"
        remove_attached.object.header.frame_id = end_effector_link
        remove_attached.object.operation = CollisionObject.REMOVE
        attached_collision_objects.append(remove_attached)

    scene.robot_state.attached_collision_objects = attached_collision_objects

    request = ApplyPlanningScene.Request()
    request.scene = scene
    return request


def _tray_collision_objects(
    *,
    frame_id: str,
    tray_pose: Pose3D,
    tray_inner_size_m: tuple[float, float, float],
    tray_wall_thickness_m: float,
) -> tuple[object, ...]:
    inner_x, inner_y, inner_z = tray_inner_size_m
    wall = tray_wall_thickness_m
    half_inner_z = inner_z / 2.0
    wall_height = inner_z + wall
    return (
        _box_collision_object(
            object_id="place_tray_base",
            frame_id=frame_id,
            pose=Pose3D(tray_pose.x, tray_pose.y, tray_pose.z, 0.0, 0.0, 0.0),
            size_xyz=(inner_x + 2 * wall, inner_y + 2 * wall, wall),
        ),
        _box_collision_object(
            object_id="place_tray_wall_front",
            frame_id=frame_id,
            pose=Pose3D(tray_pose.x + inner_x / 2.0 + wall / 2.0, tray_pose.y, tray_pose.z + half_inner_z, 0.0, 0.0, 0.0),
            size_xyz=(wall, inner_y + 2 * wall, wall_height),
        ),
        _box_collision_object(
            object_id="place_tray_wall_back",
            frame_id=frame_id,
            pose=Pose3D(tray_pose.x - inner_x / 2.0 - wall / 2.0, tray_pose.y, tray_pose.z + half_inner_z, 0.0, 0.0, 0.0),
            size_xyz=(wall, inner_y + 2 * wall, wall_height),
        ),
        _box_collision_object(
            object_id="place_tray_wall_left",
            frame_id=frame_id,
            pose=Pose3D(tray_pose.x, tray_pose.y + inner_y / 2.0 + wall / 2.0, tray_pose.z + half_inner_z, 0.0, 0.0, 0.0),
            size_xyz=(inner_x, wall, wall_height),
        ),
        _box_collision_object(
            object_id="place_tray_wall_right",
            frame_id=frame_id,
            pose=Pose3D(tray_pose.x, tray_pose.y - inner_y / 2.0 - wall / 2.0, tray_pose.z + half_inner_z, 0.0, 0.0, 0.0),
            size_xyz=(inner_x, wall, wall_height),
        ),
    )


def _box_collision_object(*, object_id: str, frame_id: str, pose: Pose3D, size_xyz: tuple[float, float, float]) -> object:
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(size_xyz[0]), float(size_xyz[1]), float(size_xyz[2])]

    collision_object = CollisionObject()
    collision_object.id = object_id
    collision_object.header.frame_id = frame_id
    collision_object.primitives = [primitive]
    collision_object.primitive_poses = [_pose_msg_from_pose(pose)]
    collision_object.operation = CollisionObject.ADD
    return collision_object


def _sphere_collision_object(*, object_id: str, frame_id: str, pose: Pose3D, radius_m: float) -> object:
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [float(radius_m)]

    collision_object = CollisionObject()
    collision_object.id = object_id
    collision_object.header.frame_id = frame_id
    collision_object.primitives = [primitive]
    collision_object.primitive_poses = [_pose_msg_from_pose(pose)]
    collision_object.operation = CollisionObject.ADD
    return collision_object


def _remove_collision_object(*, object_id: str, frame_id: str) -> object:
    from moveit_msgs.msg import CollisionObject

    collision_object = CollisionObject()
    collision_object.id = object_id
    collision_object.header.frame_id = frame_id
    collision_object.operation = CollisionObject.REMOVE
    return collision_object


def _pose_msg_from_pose(pose: Pose3D) -> object:
    from geometry_msgs.msg import Pose

    pose_msg = Pose()
    pose_msg.position.x = float(pose.x)
    pose_msg.position.y = float(pose.y)
    pose_msg.position.z = float(pose.z)
    pose_msg.orientation = _quaternion_from_pose(pose)
    return pose_msg


def _planning_scene_object_ids() -> tuple[str, ...]:
    return (
        "tomato_branch",
        "tomato_stem",
        "place_tray_base",
        "place_tray_wall_front",
        "place_tray_wall_back",
        "place_tray_wall_left",
        "place_tray_wall_right",
        "target_tomato",
    )


def _tomato_planning_scene_ops(
    *,
    attach_tomato: bool,
    planning_scene_has_attached_tomato: bool,
) -> _TomatoPlanningSceneOps:
    if attach_tomato:
        return _TomatoPlanningSceneOps(
            add_world_tomato=False,
            remove_world_tomato=False,
            add_attached_tomato=True,
            remove_attached_tomato=False,
        )
    return _TomatoPlanningSceneOps(
        add_world_tomato=True,
        remove_world_tomato=False,
        add_attached_tomato=False,
        remove_attached_tomato=planning_scene_has_attached_tomato,
    )


def _moveit_link_target_pose_from_runtime_tool_pose(
    runtime_tool_pose: Pose3D,
    *,
    link_to_tool_offset_m: tuple[float, float, float],
) -> Pose3D:
    inverse_offset_m = tuple(-value for value in link_to_tool_offset_m)
    return _shift_pose_by_local_offset(runtime_tool_pose, inverse_offset_m)


def _shift_pose_by_local_offset(
    pose: Pose3D,
    local_offset_m: tuple[float, float, float],
) -> Pose3D:
    offset_x, offset_y, offset_z = _rotate_local_offset(local_offset_m, pose)
    return Pose3D(
        x=round(pose.x + offset_x, 6),
        y=round(pose.y + offset_y, 6),
        z=round(pose.z + offset_z, 6),
        roll=pose.roll,
        pitch=pose.pitch,
        yaw=pose.yaw,
    )


def _rotate_local_offset(
    local_offset_m: tuple[float, float, float],
    pose: Pose3D,
) -> tuple[float, float, float]:
    x, y, z = local_offset_m
    roll = math.radians(pose.roll)
    pitch = math.radians(pose.pitch)
    yaw = math.radians(pose.yaw)

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr

    return (
        r00 * x + r01 * y + r02 * z,
        r10 * x + r11 * y + r12 * z,
        r20 * x + r21 * y + r22 * z,
    )


def _quaternion_from_pose(pose: Pose3D) -> object:
    from geometry_msgs.msg import Quaternion

    roll = math.radians(pose.roll)
    pitch = math.radians(pose.pitch)
    yaw = math.radians(pose.yaw)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    quaternion = Quaternion()
    quaternion.w = cr * cp * cy + sr * sp * sy
    quaternion.x = sr * cp * cy - cr * sp * sy
    quaternion.y = cr * sp * cy + sr * cp * sy
    quaternion.z = cr * cp * sy - sr * sp * cy
    return quaternion


def build_planner(*, grasp_lateral_offset_m: float = 0.0) -> tuple[MotionPlanner, PlannerBackendInfo]:
    requested = os.environ.get("TOMATO_HARVEST_PLANNER_BACKEND", "auto").strip().lower()

    if requested == "geometric":
        planner = MoveItStylePreGraspPlanner(grasp_lateral_offset_m=grasp_lateral_offset_m)
        return planner, PlannerBackendInfo(name="geometric_fallback", moveit2_enabled=False)

    if _moveit2_python_available():
        planner = MoveIt2ServiceBridgePlanner(grasp_lateral_offset_m=grasp_lateral_offset_m)
        return planner, PlannerBackendInfo(name="moveit2_service_bridge", moveit2_enabled=True)

    planner = MoveItStylePreGraspPlanner(grasp_lateral_offset_m=grasp_lateral_offset_m)
    if requested == "moveit2":
        return planner, PlannerBackendInfo(name="geometric_fallback_moveit2_unavailable", moveit2_enabled=False)
    return planner, PlannerBackendInfo(name="geometric_fallback", moveit2_enabled=False)
