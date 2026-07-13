from __future__ import annotations

import math
import os
import time
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
from tomato_harvest_sim.msg.topics import DEFAULT_JOINT_NAMES, home_joint_state
from tomato_harvest_sim.robot.msg.planner import MotionPlanner, MoveIt2PlannerBridge, MoveIt2PlanningResult, PlannerBackendInfo
from tomato_harvest_sim.robot.motion_planner.phase_suffix_replan import (
    SUFFIX_TRAJECTORY_FIELD_BY_PHASE,
    terminal_joint_state_of_phase,
)
from tomato_harvest_sim.robot.motion_planner.planning_diagnostics import (
    PlanningFailureDiagnostic,
    StateValidityReport,
    diagnostics_directory,
    save_planning_failure_diagnostic,
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


@dataclass(frozen=True)
class _MotionPlanOutcome:
    """GetMotionPlan呼び出し1回の結果。失敗理由とerror_codeを診断へ引き継ぐ。"""

    trajectory: JointTrajectory | None
    error_code: int | None
    failure_reason: str | None


def arm_joint_goal_from_ik_solution(
    *,
    solution_joint_names: tuple[str, ...],
    solution_positions_rad: tuple[float, ...],
    arm_joint_names: tuple[str, ...],
) -> JointStateSnapshot | None:
    """seed付きIK解からarm関節のjoint-space goalを組み立てる (Issue #37)。

    /compute_ik はfinger等を含む全関節stateで解を返すため、計画groupの
    arm関節だけを指定順に射影する。欠落があればNone (pose goalへfallback)。
    """
    by_name = dict(zip(solution_joint_names, solution_positions_rad))
    if any(name not in by_name for name in arm_joint_names):
        return None
    return JointStateSnapshot(
        joint_names=arm_joint_names,
        positions_rad=tuple(float(by_name[name]) for name in arm_joint_names),
    )


def goal_joint_window(
    joint_state: JointStateSnapshot, *, window_rad: float
) -> tuple[str, float, float] | None:
    """pose goalへ併置する、base関節 (panda_joint1) の許容窓を返す (Issue #37)。

    OMPLのgoal IKサンプリングは同じ手先poseに対して遠いIK枝 (base大旋回) を
    選ぶことがあり、JTCが追従不能な軌道 (goal_tolerance_violated) を生む。
    goal制約に「joint1は現在値±窓」を併置することで近いIK枝だけを許す。
    窓内に解が無い場合に備え、呼び出し側は窓なしの再試行を持つこと。

    Args:
        joint_state: 現在の関節状態。
        window_rad: 許容半幅 [rad]。0以下で無効。

    Returns:
        (関節名, 中心値, 許容半幅)。無効時やbase関節が無い場合はNone。
    """
    if window_rad <= 0.0:
        return None
    for name, position in zip(joint_state.joint_names, joint_state.positions_rad):
        if name == "panda_joint1":
            return (name, float(position), float(window_rad))
    return None

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
    # 関節空間goal fallbackの許容誤差。採用済みplan終端と同一構成を要求する。
    JOINT_GOAL_TOLERANCE_RAD = 0.01

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
        self._state_validity_service_name = os.environ.get(
            "TOMATO_HARVEST_MOVEIT_STATE_VALIDITY_SERVICE",
            "/check_state_validity",
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
        # pose goalのIK枝を現在構成の近傍へ制限する窓 (Issue #37)。0で無効。
        self._goal_joint1_window_rad = float(os.environ.get(
            "TOMATO_HARVEST_MOVEIT_GOAL_JOINT1_WINDOW_RAD", "1.5"
        ))
        # seed付きIKで最近傍IK枝を先に確定し、joint-space goalで計画する
        # (Issue #37)。goal samplingの枝選択非決定性を排除する。
        self._seeded_ik_goal_enabled = os.environ.get(
            "TOMATO_HARVEST_MOVEIT_SEEDED_IK_GOAL", "1"
        ).strip() not in {"0", "false", "False"}
        self._ik_service_name = os.environ.get(
            "TOMATO_HARVEST_MOVEIT_IK_SERVICE", "/compute_ik"
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
        self._diagnostic_dir = diagnostics_directory(os.environ)

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
            phase_label="pregrasp",
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
            phase_label="grasp",
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
            phase_label="pull",
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
                phase_label="pre_place",
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
            phase_label="place",
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
        # 採用済みtrajectoryの終端は検証済みの有効構成。pose goalのIKサンプリングが
        # 全滅したときの関節空間goal fallbackとして使う (Issue #28 改善2)。
        fallback_joint_goal = terminal_joint_state_of_phase(plan, phase)

        # 把持前phaseは単一区間の再計画で完結する。トマトはまだworld側にある。
        if phase is HarvestTaskPhase.MOVING_TO_PREGRASP:
            pregrasp_trajectory = self._plan_phase(
                clients=clients,
                joint_state=current_joint_state,
                base_frame_id=base_frame_id,
                scene_snapshot=scene_snapshot,
                target_pose=plan.pregrasp_pose,
                attach_tomato=False,
                phase_label=phase.value,
                fallback_joint_goal=fallback_joint_goal,
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
                phase_label=phase.value,
                fallback_joint_goal=fallback_joint_goal,
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
                fallback_joint_goal=fallback_joint_goal,
            )

        # home復帰はトマトをtrayへ置いた後の自由空間移動。goalは固定のhome関節構成
        # なので、IKサンプリングを伴うpose goalを経由せず関節空間goalを一次手段に
        # 使う (Issue #32)。
        if phase is HarvestTaskPhase.RETURNING_HOME:
            if not self._apply_phase_planning_scene(
                clients=clients,
                scene_snapshot=scene_snapshot,
                base_frame_id=base_frame_id,
                attach_tomato=False,
            ):
                return self._fallback_result("planning_scene_unavailable")
            home_trajectory = self._plan_joint_goal(
                clients=clients,
                joint_state=current_joint_state,
                base_frame_id=base_frame_id,
                goal_joint_state=home_joint_state(),
                phase_label=phase.value,
            )
            if home_trajectory is None:
                return self._fallback_result("home_replan_failed")
            return MoveIt2PlanningResult(
                success=True,
                backend_name="moveit2_service_bridge",
                reason="service_ok",
                home_joint_trajectory=home_trajectory,
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
        fallback_joint_goal: JointStateSnapshot | None = None,
    ) -> MoveIt2PlanningResult:
        """approach waypoint経由でplaceまでの残区間を計画する。トマトは把持中。

        pose goalの連鎖 (pre_place → place) が失敗した場合は、現在状態から
        採用済みplan終端構成へ直行する関節空間goalで復旧を試みる。goal構成が
        既知のためOMPLのgoal sampling失敗 (error_code=99999) を回避できる。
        """
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
                phase_label="moving_to_place_pre_place",
            )
            if approach_trajectory is None:
                return self._place_joint_goal_fallback(
                    clients=clients,
                    joint_state=joint_state,
                    base_frame_id=base_frame_id,
                    fallback_joint_goal=fallback_joint_goal,
                    failed_reason="pre_place_replan_failed",
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
            phase_label="moving_to_place",
        )
        if place_trajectory is None:
            return self._place_joint_goal_fallback(
                clients=clients,
                joint_state=joint_state,
                base_frame_id=base_frame_id,
                fallback_joint_goal=fallback_joint_goal,
                failed_reason="place_replan_failed",
            )

        if approach_trajectory is not None:
            place_trajectory = _concatenate_trajectories(approach_trajectory, place_trajectory)

        return MoveIt2PlanningResult(
            success=True,
            backend_name="moveit2_service_bridge",
            reason="service_ok",
            place_joint_trajectory=place_trajectory,
        )

    def _place_joint_goal_fallback(
        self,
        *,
        clients: "_Ros2MoveIt2Clients",
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        fallback_joint_goal: JointStateSnapshot | None,
        failed_reason: str,
    ) -> MoveIt2PlanningResult:
        """place連鎖の失敗を、実robot状態から終端構成への単一計画で復旧する。

        approach waypoint経由は諦め、現在状態→採用済みplace終端の1区間だけを
        関節空間goalで計画する。planning sceneは直前の失敗attemptで
        attach_tomato=True適用済みであることを前提とする。
        """
        if fallback_joint_goal is None:
            return self._fallback_result(failed_reason)
        fallback_trajectory = self._plan_joint_goal(
            clients=clients,
            joint_state=joint_state,
            base_frame_id=base_frame_id,
            goal_joint_state=fallback_joint_goal,
            phase_label="moving_to_place",
        )
        if fallback_trajectory is None:
            return self._fallback_result(failed_reason)
        return MoveIt2PlanningResult(
            success=True,
            backend_name="moveit2_service_bridge",
            reason="joint_goal_fallback",
            place_joint_trajectory=fallback_trajectory,
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
        phase_label: str = "",
        fallback_joint_goal: JointStateSnapshot | None = None,
    ) -> JointTrajectory | None:
        """1区間をpose goalで計画し、失敗時は診断保存と関節空間goal fallbackを行う。

        Args:
            phase_label: 診断・ログでこの計画区間を識別する名前。
            fallback_joint_goal: pose goal失敗時に直行する既知の有効goal構成。
                Noneならfallbackしない (フルチェーン初期計画など)。
        """
        if not self._apply_phase_planning_scene(
            clients=clients,
            scene_snapshot=scene_snapshot,
            base_frame_id=base_frame_id,
            attach_tomato=attach_tomato,
        ):
            return None
        # 最近傍IK枝の決定的選択 (Issue #37): 現在姿勢をseedにIKを解き、
        # その関節構成へのjoint-space goalで計画する。goal samplingの
        # 枝選択非決定性 (JTCが追従できないbase大旋回) を排除する。
        if self._seeded_ik_goal_enabled:
            trajectory = self._plan_seeded_ik_goal(
                clients=clients,
                joint_state=joint_state,
                base_frame_id=base_frame_id,
                target_pose=target_pose,
                phase_label=phase_label,
            )
            if trajectory is not None:
                return trajectory

        # IKが解けない・joint計画が失敗した場合は、近いIK枝を優先する窓付き
        # pose goalで試行し、解が無ければ窓なしで再試行する。
        joint_window = goal_joint_window(
            joint_state, window_rad=self._goal_joint1_window_rad
        )
        outcome = self._plan_pose_goal(
            clients=clients,
            joint_state=joint_state,
            base_frame_id=base_frame_id,
            target_pose=target_pose,
            joint_window=joint_window,
        )
        if outcome.trajectory is None and joint_window is not None:
            print(
                f"[MoveItBridge] goal_joint_window exhausted phase={phase_label} "
                f"window_rad={joint_window[2]} — retrying without window",
                flush=True,
            )
            outcome = self._plan_pose_goal(
                clients=clients,
                joint_state=joint_state,
                base_frame_id=base_frame_id,
                target_pose=target_pose,
                joint_window=None,
            )
        trajectory = outcome.trajectory
        failure_reason = outcome.failure_reason
        if trajectory is None:
            self._debug_log(
                f"[MoveItBridge] phase planning failed: ee_link={self._end_effector_link} "
                f"target_xyz=({target_pose.x:.4f}, {target_pose.y:.4f}, {target_pose.z:.4f})"
            )
            self._record_planning_failure(
                clients=clients,
                phase_label=phase_label,
                goal_kind="pose",
                joint_state=joint_state,
                target_xyz_m=(target_pose.x, target_pose.y, target_pose.z),
                error_code=outcome.error_code,
                reason=failure_reason or "unknown",
            )
            if fallback_joint_goal is None:
                return None
            return self._plan_joint_goal(
                clients=clients,
                joint_state=joint_state,
                base_frame_id=base_frame_id,
                goal_joint_state=fallback_joint_goal,
                phase_label=phase_label,
            )
        self._debug_log(
            "[MoveItBridge] accepted trajectory "
            f"points={len(trajectory.points)} "
            f"ee_link={self._end_effector_link} "
            f"target_xyz=({target_pose.x:.4f}, {target_pose.y:.4f}, {target_pose.z:.4f}) "
            f"end_q={trajectory.points[-1].positions_rad}"
        )
        return trajectory

    def _plan_seeded_ik_goal(
        self,
        *,
        clients: "_Ros2MoveIt2Clients",
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        target_pose: Pose3D,
        phase_label: str,
    ) -> JointTrajectory | None:
        """seed付きIKで最近傍IK枝を確定し、joint-space goalで計画する (Issue #37)。

        失敗時はNoneを返し、呼び出し側が窓付きpose goalへfallbackする。
        planning sceneは適用済みであること (avoid_collisions=Trueで参照される)。
        """
        moveit_target_pose = _moveit_link_target_pose_from_runtime_tool_pose(
            target_pose,
            link_to_tool_offset_m=self.MOVEIT_LINK_TO_RUNTIME_TOOL_OFFSET_M,
        )
        quaternion = _quaternion_from_pose(moveit_target_pose)
        ik_solution = clients.compute_nearest_ik(
            seed_joint_state=joint_state,
            base_frame_id=base_frame_id,
            target_pose_xyz=(
                float(moveit_target_pose.x),
                float(moveit_target_pose.y),
                float(moveit_target_pose.z),
            ),
            target_orientation_xyzw=(
                float(quaternion.x), float(quaternion.y),
                float(quaternion.z), float(quaternion.w),
            ),
            group_name=self._group_name,
            timeout_sec=self._planning_timeout_sec,
        )
        if ik_solution is None:
            print(
                f"[MoveItBridge] seeded_ik unsolved phase={phase_label} "
                "— falling back to pose goal",
                flush=True,
            )
            return None
        goal_joint_state = arm_joint_goal_from_ik_solution(
            solution_joint_names=ik_solution.joint_names,
            solution_positions_rad=ik_solution.positions_rad,
            arm_joint_names=DEFAULT_JOINT_NAMES,
        )
        if goal_joint_state is None:
            return None
        request = self._build_joint_goal_motion_plan_request(
            joint_state=joint_state,
            base_frame_id=base_frame_id,
            goal_joint_state=goal_joint_state,
        )
        outcome = clients.plan_motion(request, timeout_sec=self._planning_timeout_sec)
        trajectory = outcome.trajectory
        if trajectory is not None and _trajectory_is_noop(
            trajectory,
            start_joint_state=joint_state,
            tolerance_rad=self.NOOP_TRAJECTORY_TOLERANCE_RAD,
        ):
            trajectory = None
        if trajectory is None:
            print(
                f"[MoveItBridge] seeded_ik goal plan failed phase={phase_label} "
                f"reason={outcome.failure_reason} error_code={outcome.error_code} "
                "— falling back to pose goal",
                flush=True,
            )
            return None
        print(
            f"[MoveItBridge] seeded_ik goal plan succeeded phase={phase_label} "
            f"points={len(trajectory.points)} goal_q={goal_joint_state.positions_rad}",
            flush=True,
        )
        return trajectory

    def _plan_pose_goal(
        self,
        *,
        clients: "_Ros2MoveIt2Clients",
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        target_pose: Pose3D,
        joint_window: tuple[str, float, float] | None,
    ) -> _MotionPlanOutcome:
        """pose goal計画1回分 (request構築→service→no-op検査) を実行する。"""
        request = self._build_motion_plan_request(
            joint_state=joint_state,
            base_frame_id=base_frame_id,
            target_pose=target_pose,
            joint_window=joint_window,
        )
        outcome = clients.plan_motion(request, timeout_sec=self._planning_timeout_sec)
        if outcome.trajectory is not None and _trajectory_is_noop(
            outcome.trajectory,
            start_joint_state=joint_state,
            tolerance_rad=self.NOOP_TRAJECTORY_TOLERANCE_RAD,
        ):
            self._debug_log(
                "[MoveItBridge] rejecting no-op trajectory and falling back to geometric execution. "
                f"ee_link={self._end_effector_link} "
                f"target_xyz=({target_pose.x:.4f}, {target_pose.y:.4f}, {target_pose.z:.4f}) "
                f"start_q={joint_state.positions_rad} "
                f"end_q={outcome.trajectory.points[-1].positions_rad}"
            )
            return _MotionPlanOutcome(None, outcome.error_code, "noop_trajectory")
        return outcome

    def _apply_phase_planning_scene(
        self,
        *,
        clients: "_Ros2MoveIt2Clients",
        scene_snapshot: SceneSnapshot,
        base_frame_id: str,
        attach_tomato: bool,
    ) -> bool:
        """phase計画の前提となるplanning scene (トマトのworld/attached切替) を適用する。"""
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
            return False
        self._planning_scene_has_attached_tomato = attach_tomato
        return True

    def _plan_joint_goal(
        self,
        *,
        clients: "_Ros2MoveIt2Clients",
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        goal_joint_state: JointStateSnapshot,
        phase_label: str,
    ) -> JointTrajectory | None:
        """既知の有効構成への関節空間goal計画 (Issue #28 改善2)。

        goal構成が確定しているためOMPLのgoal state sampling (IK) を経由せず、
        `Unable to sample any valid states for goal tree` 系の失敗を回避する。
        planning sceneは呼び出し元 (直前のpose goal attempt) が適用済みであること。
        """
        request = self._build_joint_goal_motion_plan_request(
            joint_state=joint_state,
            base_frame_id=base_frame_id,
            goal_joint_state=goal_joint_state,
        )
        outcome = clients.plan_motion(request, timeout_sec=self._planning_timeout_sec)
        trajectory = outcome.trajectory
        failure_reason = outcome.failure_reason
        if trajectory is not None and _trajectory_is_noop(
            trajectory,
            start_joint_state=joint_state,
            tolerance_rad=self.NOOP_TRAJECTORY_TOLERANCE_RAD,
        ):
            trajectory = None
            failure_reason = "noop_trajectory"
        if trajectory is None:
            print(
                f"[MoveItBridge] joint_goal_fallback failed phase={phase_label} "
                f"reason={failure_reason} error_code={outcome.error_code}",
                flush=True,
            )
            self._record_planning_failure(
                clients=clients,
                phase_label=phase_label,
                goal_kind="joint",
                joint_state=joint_state,
                target_xyz_m=None,
                error_code=outcome.error_code,
                reason=failure_reason or "unknown",
            )
            return None
        print(
            f"[MoveItBridge] joint_goal_fallback succeeded phase={phase_label} "
            f"points={len(trajectory.points)} "
            f"goal_q={goal_joint_state.positions_rad}",
            flush=True,
        )
        return trajectory

    def _record_planning_failure(
        self,
        *,
        clients: "_Ros2MoveIt2Clients",
        phase_label: str,
        goal_kind: str,
        joint_state: JointStateSnapshot,
        target_xyz_m: tuple[float, float, float] | None,
        error_code: int | None,
        reason: str,
    ) -> None:
        """planning失敗の証跡を残す (Issue #28 改善1)。

        失敗ログは常に出す。start state有効性の問い合わせとJSON保存は、
        診断ディレクトリが設定されているときだけ行い、planner本体の
        レイテンシへ影響させない。
        """
        print(
            f"[MoveItBridge] planning_failure phase={phase_label} goal_kind={goal_kind} "
            f"reason={reason} error_code={error_code}",
            flush=True,
        )
        if self._diagnostic_dir is None:
            return
        validity = clients.check_state_validity(
            joint_state=joint_state,
            group_name=self._group_name,
            timeout_sec=self._planning_timeout_sec,
        )
        diagnostic = PlanningFailureDiagnostic(
            captured_at_sec=time.time(),
            phase=phase_label or "unknown",
            goal_kind=goal_kind,
            reason=reason,
            error_code=error_code,
            target_xyz_m=target_xyz_m,
            start_joint_names=joint_state.joint_names,
            start_positions_rad=joint_state.positions_rad,
            start_state=validity,
        )
        path = save_planning_failure_diagnostic(diagnostic, self._diagnostic_dir)
        print(
            f"[MoveItBridge] planning_failure_diagnostic "
            f"saved={path is not None} path={path} "
            f"start_state_checked={validity.checked} start_state_valid={validity.valid} "
            f"contacts={','.join(validity.contacts) or 'none'}",
            flush=True,
        )

    def _build_motion_plan_request(
        self,
        *,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        target_pose: Pose3D,
        joint_window: tuple[str, float, float] | None = None,
    ) -> object:
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

        # 近いIK枝だけを許すbase関節窓 (Issue #37)。pose拘束とANDで効く。
        if joint_window is not None:
            window_name, window_center, window_half_rad = joint_window
            window_constraint = JointConstraint()
            window_constraint.joint_name = window_name
            window_constraint.position = window_center
            window_constraint.tolerance_above = window_half_rad
            window_constraint.tolerance_below = window_half_rad
            window_constraint.weight = 1.0
            goal_constraints.joint_constraints = [window_constraint]

        motion_plan_request = self._new_motion_plan_request(
            joint_state=joint_state, base_frame_id=base_frame_id
        )
        motion_plan_request.goal_constraints = [goal_constraints]
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

    def _build_joint_goal_motion_plan_request(
        self,
        *,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        goal_joint_state: JointStateSnapshot,
    ) -> object:
        """既知の関節構成をgoalとするMotionPlanRequestを作る。

        pose goalと違いgoal state samplingが不要なため、goal構成が有効である
        限りOMPLは経路探索だけに専念できる。
        """
        from moveit_msgs.msg import Constraints, JointConstraint
        from moveit_msgs.srv import GetMotionPlan

        goal_constraints = Constraints()
        for name, position in zip(
            goal_joint_state.joint_names, goal_joint_state.positions_rad
        ):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = self.JOINT_GOAL_TOLERANCE_RAD
            joint_constraint.tolerance_below = self.JOINT_GOAL_TOLERANCE_RAD
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)

        motion_plan_request = self._new_motion_plan_request(
            joint_state=joint_state, base_frame_id=base_frame_id
        )
        motion_plan_request.goal_constraints = [goal_constraints]
        self._debug_log(
            "[MoveItBridge] joint goal request "
            f"group={self._group_name} "
            f"goal_q={goal_joint_state.positions_rad} "
            f"start_q={joint_state.positions_rad}"
        )

        request = GetMotionPlan.Request()
        request.motion_plan_request = motion_plan_request
        return request

    def _new_motion_plan_request(
        self,
        *,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
    ) -> object:
        """workspace・start state・planner設定を持つgoal未設定のrequest本体を作る。"""
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
        start_joint_state.position = [float(position) for position in joint_state.positions_rad]

        start_state = RobotState()
        start_state.joint_state = start_joint_state
        start_state.is_diff = False

        motion_plan_request = MotionPlanRequest()
        motion_plan_request.workspace_parameters = workspace
        motion_plan_request.start_state = start_state
        motion_plan_request.group_name = self._group_name
        motion_plan_request.num_planning_attempts = 4
        motion_plan_request.allowed_planning_time = self._allowed_planning_time_sec
        motion_plan_request.max_velocity_scaling_factor = 0.2
        motion_plan_request.max_acceleration_scaling_factor = 0.2
        return motion_plan_request

    def _require_clients(self) -> "_Ros2MoveIt2Clients | None":
        if self._clients is not None:
            return self._clients
        try:
            self._clients = _Ros2MoveIt2Clients(
                motion_plan_service_name=self._service_name,
                planning_scene_service_name=self._scene_service_name,
                state_validity_service_name=self._state_validity_service_name,
                ik_service_name=self._ik_service_name,
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
        state_validity_service_name: str = "/check_state_validity",
        ik_service_name: str = "/compute_ik",
    ) -> None:
        import rclpy
        from moveit_msgs.srv import (
            ApplyPlanningScene, GetMotionPlan, GetPositionIK, GetStateValidity,
        )
        from rclpy.executors import SingleThreadedExecutor

        self._rclpy = rclpy
        if not self._rclpy.ok():
            self._rclpy.init(args=None)
        self._node = self._rclpy.create_node("tomato_harvest_moveit_bridge")
        self._motion_plan_client = self._node.create_client(GetMotionPlan, motion_plan_service_name)
        self._planning_scene_client = self._node.create_client(ApplyPlanningScene, planning_scene_service_name)
        self._state_validity_client = self._node.create_client(
            GetStateValidity, state_validity_service_name
        )
        self._ik_client = self._node.create_client(GetPositionIK, ik_service_name)
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

    def plan_motion(self, request: object, *, timeout_sec: float) -> _MotionPlanOutcome:
        """GetMotionPlanを1回呼び、trajectoryと失敗証跡を返す。

        Returns:
            成功時はtrajectory入りのoutcome。失敗時はtrajectory=Noneで、
            error_codeと失敗分類 (service_timeout / empty_response /
            motion_plan_error / empty_trajectory) を保持する。
        """
        future = self._motion_plan_client.call_async(request)
        self._spin_until_done(future, timeout_sec=timeout_sec)
        if not future.done():
            return _MotionPlanOutcome(None, None, "service_timeout")
        response = future.result()
        if response is None:
            return _MotionPlanOutcome(None, None, "empty_response")
        error_code = int(response.motion_plan_response.error_code.val)
        if error_code != 1:
            print(f"[MoveItBridge] motion plan service returned error_code={error_code}.", flush=True)
            return _MotionPlanOutcome(None, error_code, "motion_plan_error")
        robot_trajectory = response.motion_plan_response.trajectory
        joint_trajectory = getattr(robot_trajectory, "joint_trajectory", None)
        if joint_trajectory is None:
            print("[MoveItBridge] motion plan response had no joint_trajectory.", flush=True)
            return _MotionPlanOutcome(None, error_code, "empty_trajectory")
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
            return _MotionPlanOutcome(planned_trajectory, error_code, None)
        print("[MoveItBridge] motion plan response had an empty joint trajectory.", flush=True)
        return _MotionPlanOutcome(None, error_code, "empty_trajectory")

    def compute_nearest_ik(
        self,
        *,
        seed_joint_state: JointStateSnapshot,
        base_frame_id: str,
        target_pose_xyz: tuple[float, float, float],
        target_orientation_xyzw: tuple[float, float, float, float],
        group_name: str,
        timeout_sec: float,
    ) -> JointStateSnapshot | None:
        """現在姿勢をseedに、目標poseの最近傍IK解を求める (Issue #37)。

        反復IK (KDL) はseedから最も近い解へ収束するため、goal samplingの
        枝選択非決定性を排さずに「最小コストのIK枝」を決定的に得られる。
        avoid_collisions=Trueでplanning scene上の衝突解は棄却される。
        """
        from moveit_msgs.srv import GetPositionIK
        from sensor_msgs.msg import JointState

        if not self._ik_client.wait_for_service(timeout_sec=timeout_sec):
            return None
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.avoid_collisions = True
        request.ik_request.robot_state.joint_state = JointState()
        request.ik_request.robot_state.joint_state.name = list(seed_joint_state.joint_names)
        request.ik_request.robot_state.joint_state.position = [
            float(v) for v in seed_joint_state.positions_rad
        ]
        request.ik_request.pose_stamped.header.frame_id = base_frame_id
        request.ik_request.pose_stamped.pose.position.x = target_pose_xyz[0]
        request.ik_request.pose_stamped.pose.position.y = target_pose_xyz[1]
        request.ik_request.pose_stamped.pose.position.z = target_pose_xyz[2]
        request.ik_request.pose_stamped.pose.orientation.x = target_orientation_xyzw[0]
        request.ik_request.pose_stamped.pose.orientation.y = target_orientation_xyzw[1]
        request.ik_request.pose_stamped.pose.orientation.z = target_orientation_xyzw[2]
        request.ik_request.pose_stamped.pose.orientation.w = target_orientation_xyzw[3]
        future = self._ik_client.call_async(request)
        self._spin_until_done(future, timeout_sec=timeout_sec)
        if not future.done():
            return None
        response = future.result()
        if response is None or int(response.error_code.val) != 1:
            return None
        solution = response.solution.joint_state
        return JointStateSnapshot(
            joint_names=tuple(str(name) for name in solution.name),
            positions_rad=tuple(float(v) for v in solution.position),
        )

    def check_state_validity(
        self,
        *,
        joint_state: JointStateSnapshot,
        group_name: str,
        timeout_sec: float,
    ) -> StateValidityReport:
        """planning失敗診断用に、与えた関節状態の有効性と衝突ペアを問い合わせる。

        move_groupの`/check_state_validity`が使えない環境でも診断全体を
        壊さないよう、失敗はchecked=Falseとして返す。
        """
        from moveit_msgs.srv import GetStateValidity
        from sensor_msgs.msg import JointState

        if not self._state_validity_client.wait_for_service(timeout_sec=timeout_sec):
            return StateValidityReport(checked=False)
        request = GetStateValidity.Request()
        request.group_name = group_name
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = list(joint_state.joint_names)
        request.robot_state.joint_state.position = [
            float(position) for position in joint_state.positions_rad
        ]
        future = self._state_validity_client.call_async(request)
        self._spin_until_done(future, timeout_sec=timeout_sec)
        if not future.done():
            return StateValidityReport(checked=False)
        response = future.result()
        if response is None:
            return StateValidityReport(checked=False)
        contacts = tuple(
            f"{contact.contact_body_1}|{contact.contact_body_2}"
            for contact in getattr(response, "contacts", ())
        )
        return StateValidityReport(
            checked=True,
            valid=bool(response.valid),
            contacts=contacts,
        )


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
