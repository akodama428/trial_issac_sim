"""Compatibility facade for the responsibility-split MoveIt bridge package."""
from __future__ import annotations

from dataclasses import replace

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    SceneSnapshot,
    TargetEstimate,
)
from tomato_harvest_sim.robot.msg.planner import (
    MotionPlanner,
    MoveIt2PlannerBridge,
)
from tomato_harvest_sim.robot.motion_planner.harvest_pose_planner import (
    HarvestPoseWaypointPlanner,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.client import (
    MotionPlanOutcome as _MotionPlanOutcome,
    Ros2MoveIt2Clients as _Ros2MoveIt2Clients,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.geometry import (
    moveit_link_target_pose_from_runtime_tool_pose
    as _moveit_link_target_pose_from_runtime_tool_pose,
    quaternion_from_pose as _quaternion_from_pose,
    rotate_local_offset as _rotate_local_offset,
    shift_pose_by_local_offset as _shift_pose_by_local_offset,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.phase_planner import (
    Ros2MoveIt2PlannerBridge,
    moveit2_python_available as _moveit2_python_available,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.phase_policy import (
    PhasePlanningSpec as _PhasePlanningSpec,
    PlanningTarget,
    arm_joint_goal_from_ik_solution,
    goal_joint_window,
    ik_goal_is_near_seed,
    phase_planning_specs as _phase_planning_specs,
    should_start_via_home,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.planning_scene import (
    TomatoPlanningSceneOps as _TomatoPlanningSceneOps,
    build_planning_scene_request as _build_planning_scene_request,
    tomato_planning_scene_ops as _tomato_planning_scene_ops,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.trajectory import (
    clamp_joint_state_to_bounds as _clamp_joint_state_to_bounds,
    concatenate_trajectories as _concatenate_trajectories,
    joint_state_from_trajectory as _joint_state_from_trajectory,
    joint_trajectory_from_msg as _joint_trajectory_from_msg,
    joint_trajectory_from_request_start_state
    as _joint_trajectory_from_request_start_state,
    trajectory_is_noop as _trajectory_is_noop,
)
from tomato_harvest_sim.robot.motion_planner.phase_suffix_replan import (
    PHASE_TRAJECTORY_FIELD_BY_PHASE,
)


class MoveIt2ServiceBridgePlanner(MotionPlanner):
    """目標poseの初期計画と、各phaseの実行軌道計画を二段階で行う。"""

    def __init__(
        self,
        *,
        bridge: MoveIt2PlannerBridge | None = None,
    ) -> None:
        self._pose_planner = HarvestPoseWaypointPlanner()
        self._bridge = bridge or Ros2MoveIt2PlannerBridge()

    def plan(
        self,
        target_estimate: TargetEstimate,
        scene_snapshot: SceneSnapshot,
    ) -> HarvestMotionPlan:
        """収穫サイクル全体で使う目標pose・waypointを決定する。

        この初期計画では現在関節角を参照せず、MoveItによる軌道計画も行わない。
        そのため、返すplanの各JointTrajectory fieldは未設定のままである。
        実行用軌道は、各移動phaseの開始直前に`plan_phase_trajectory()`で生成する。

        Args:
            target_estimate: 認識した収穫対象tomatoの位置・姿勢。
            scene_snapshot: trayなど、目標poseの決定に必要なscene情報。

        Returns:
            各phaseの目標pose・waypointだけを保持する初期plan。
        """
        return self._pose_planner.plan(target_estimate, scene_snapshot)

    def plan_phase_trajectory(
        self,
        phase: HarvestTaskPhase,
        prior_plan: HarvestMotionPlan,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        scene_snapshot: SceneSnapshot,
    ) -> HarvestMotionPlan | None:
        """指定した移動phaseの実行用JointTrajectoryを直前計画する。

        `plan()`が決定済みの目標pose・waypointを利用し、phase開始時点の
        最新関節角とPlanningSceneからMoveItで軌道を生成する。成功時は
        `prior_plan`のpose・waypointを維持し、対象phaseのtrajectory field
        だけを差し替える。

        Args:
            phase: 今から実行する移動phase。
            prior_plan: `plan()`で作成した目標pose・waypointを含む既存plan。
            joint_state: MoveIt計画の開始点となる最新関節角。
            base_frame_id: MoveIt requestで使用する基準frame。
            scene_snapshot: 軌道計画時点の障害物・attach状態を含むscene情報。

        Returns:
            対象phaseの軌道を追加したplan。対象外phaseまたは計画失敗時はNone。
        """
        # phaseを、HarvestMotionPlan内の対応するtrajectory fieldへ変換する。
        # `plan()`ではこのfieldは未設定であり、ここで初めて実行軌道を格納する。
        field = PHASE_TRAJECTORY_FIELD_BY_PHASE.get(phase)
        if field is None:
            return None

        # 同じ最新状態・同じ目標に対するMoveIt計画を最大3回まで再試行する。
        for _attempt in range(3):
            result = self._bridge.plan_phase_trajectory(
                phase=phase,
                joint_state=joint_state,
                base_frame_id=base_frame_id,
                scene_snapshot=scene_snapshot,
                plan=prior_plan,
            )
            if (
                result.success
                and result.joint_trajectory is not None
                and result.joint_trajectory.points
            ):
                # 目標pose・waypointと他phaseの軌道は維持し、今回のphaseに
                # 対応するJointTrajectoryだけを更新した新しいplanを返す。
                return replace(
                    prior_plan,
                    planner_name=result.backend_name,
                    **{field: result.joint_trajectory},
                )
        return None


def build_planner() -> MoveIt2ServiceBridgePlanner:
    """Build the MoveIt planner used by the runtime."""
    return MoveIt2ServiceBridgePlanner()
