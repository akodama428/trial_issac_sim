from __future__ import annotations

import os
import unittest

from tomato_harvest_sim.msg.contracts import (
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
    Pose3D,
    ScenePhase,
    SceneSnapshot,
    TargetEstimate,
    TfTreeSnapshot,
    TomatoStatus,
)
from tomato_harvest_sim.robot.motion_planner import MoveIt2PlanningResult, MoveIt2ServiceBridgePlanner, build_planner
from tomato_harvest_sim.robot.motion_planner.moveit_service_bridge import (
    _clamp_joint_state_to_bounds,
    _moveit_link_target_pose_from_runtime_tool_pose,
    _trajectory_is_noop,
    _tomato_planning_scene_ops,
    arm_joint_goal_from_ik_solution,
    goal_joint_window,
    ik_goal_is_near_seed,
)


def _scene_snapshot() -> SceneSnapshot:
    pose = Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return SceneSnapshot(
        phase=ScenePhase.READY,
        active_camera="fixed_camera",
        tomato_attached=True,
        tomato_status=TomatoStatus.ATTACHED,
        gripper_closed=False,
        robot_home=True,
        cycle_id=0,
        robot_model="Franka Panda",
        robot_base_pose=pose,
        fixed_camera_pose=pose,
        hand_camera_pose=pose,
        branch_pose=pose,
        stem_pose=pose,
        tomato_pose=Pose3D(0.42, 0.0, 0.54, 0.0, 0.0, 0.0),
        tray_pose=Pose3D(0.35, -0.35, 0.45, 0.0, 0.0, 0.0),
        robot_tool_pose=Pose3D(0.18, 0.0, 0.65, 180.0, 0.0, 0.0),
        target_tool_pose=None,
        grasp_result_reason=None,
    )


def _target_estimate() -> TargetEstimate:
    return TargetEstimate(
        camera_name="fixed_camera",
        target_world_pose=Pose3D(0.42, 0.0, 0.54, 0.0, 0.0, 0.0),
        target_camera_pose=Pose3D(0.05, 0.0, 0.20, 0.0, 0.0, 0.0),
        confidence=1.0,
    )


def _joint_state() -> JointStateSnapshot:
    return JointStateSnapshot(
        joint_names=(
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ),
        positions_rad=(0.0, -0.4, 0.0, -2.1, 0.0, 1.7, 0.8),
    )


def _tf_tree() -> TfTreeSnapshot:
    return TfTreeSnapshot(
        robot_base_frame_id="panda_link0",
        camera_frame_id="fixed_camera_frame",
        target_frame_id="target_tomato_frame",
        robot_base_pose=Pose3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        camera_pose=Pose3D(3.0, -3.0, 2.0, 62.0, 0.0, 45.0),
        target_pose=Pose3D(0.42, 0.0, 0.54, 0.0, 0.0, 0.0),
    )


class _AcceptingValidator:
    def __init__(self) -> None:
        self.calls: list[tuple[str, ...]] = []

    def plan_phase_trajectories(
        self,
        *,
        joint_state: JointStateSnapshot,
        tf_tree: TfTreeSnapshot,
        scene_snapshot: SceneSnapshot,
        plan: object,
    ) -> MoveIt2PlanningResult:
        self.calls.append(joint_state.joint_names)
        trajectory = JointTrajectory(
            joint_names=joint_state.joint_names,
            points=(
                JointTrajectoryPoint(joint_state.positions_rad, 0.0),
                JointTrajectoryPoint((0.1, -0.3, 0.05, -2.0, 0.1, 1.75, 0.85), 1.0),
            ),
        )
        return MoveIt2PlanningResult(
            success=True,
            backend_name="moveit2_service_bridge",
            reason="service_ok",
            pregrasp_joint_trajectory=trajectory,
            grasp_joint_trajectory=trajectory,
            pull_joint_trajectory=trajectory,
            place_joint_trajectory=trajectory,
            planning_scene_object_ids=("tomato_branch", "place_tray"),
        )


class _RejectingValidator:
    def plan_phase_trajectories(
        self,
        *,
        joint_state: JointStateSnapshot,
        tf_tree: TfTreeSnapshot,
        scene_snapshot: SceneSnapshot,
        plan: object,
    ) -> MoveIt2PlanningResult:
        return MoveIt2PlanningResult(
            success=False,
            backend_name="moveit2_service_bridge_fallback",
            reason="service_unavailable",
        )


class _PartialValidator:
    """pregrasp/grasp/pull は成功するが place は失敗するバリデータ。"""

    def plan_phase_trajectories(
        self,
        *,
        joint_state: JointStateSnapshot,
        tf_tree: TfTreeSnapshot,
        scene_snapshot: SceneSnapshot,
        plan: object,
    ) -> MoveIt2PlanningResult:
        trajectory = JointTrajectory(
            joint_names=joint_state.joint_names,
            points=(
                JointTrajectoryPoint(joint_state.positions_rad, 0.0),
                JointTrajectoryPoint((0.1, -0.3, 0.05, -2.0, 0.1, 1.75, 0.85), 1.0),
            ),
        )
        return MoveIt2PlanningResult(
            success=False,
            backend_name="moveit2_service_bridge_partial",
            reason="pre_place_plan_failed",
            pregrasp_joint_trajectory=trajectory,
            grasp_joint_trajectory=trajectory,
            pull_joint_trajectory=trajectory,
            place_joint_trajectory=None,
        )


class MoveItPlannerBackendTest(unittest.TestCase):
    def test_world_tomato_is_added_before_robot_attach(self) -> None:
        ops = _tomato_planning_scene_ops(
            attach_tomato=False,
            planning_scene_has_attached_tomato=False,
        )

        self.assertTrue(ops.add_world_tomato)
        self.assertFalse(ops.remove_world_tomato)
        self.assertFalse(ops.add_attached_tomato)
        self.assertFalse(ops.remove_attached_tomato)

    def test_attached_tomato_is_removed_only_after_prior_attach(self) -> None:
        ops = _tomato_planning_scene_ops(
            attach_tomato=False,
            planning_scene_has_attached_tomato=True,
        )

        self.assertTrue(ops.add_world_tomato)
        self.assertFalse(ops.remove_world_tomato)
        self.assertFalse(ops.add_attached_tomato)
        self.assertTrue(ops.remove_attached_tomato)

    def test_world_tomato_is_removed_when_switching_to_robot_attach(self) -> None:
        ops = _tomato_planning_scene_ops(
            attach_tomato=True,
            planning_scene_has_attached_tomato=False,
        )

        self.assertFalse(ops.add_world_tomato)
        self.assertFalse(ops.remove_world_tomato)
        self.assertTrue(ops.add_attached_tomato)
        self.assertFalse(ops.remove_attached_tomato)

    def test_moveit_target_pose_is_shifted_from_runtime_tool_pose(self) -> None:
        runtime_tool_pose = Pose3D(0.42, 0.0, 0.54, 180.0, 0.0, 0.0)

        moveit_target_pose = _moveit_link_target_pose_from_runtime_tool_pose(
            runtime_tool_pose,
            link_to_tool_offset_m=(0.0, 0.0, 0.0584),
        )

        self.assertAlmostEqual(moveit_target_pose.x, 0.42, places=6)
        self.assertAlmostEqual(moveit_target_pose.y, 0.0, places=6)
        self.assertAlmostEqual(moveit_target_pose.z, 0.5984, places=6)

    def test_default_end_effector_link_is_panda_hand(self) -> None:
        planner = MoveIt2ServiceBridgePlanner()

        self.assertEqual(planner._bridge._end_effector_link, "panda_hand")

    def test_orientation_constraint_is_enabled_by_default(self) -> None:
        planner = MoveIt2ServiceBridgePlanner()

        self.assertTrue(planner._bridge._enforce_orientation_constraint)

    def test_geometric_backend_can_be_forced(self) -> None:
        previous = os.environ.get("TOMATO_HARVEST_PLANNER_BACKEND")
        os.environ["TOMATO_HARVEST_PLANNER_BACKEND"] = "geometric"
        try:
            _, info = build_planner()
        finally:
            if previous is None:
                os.environ.pop("TOMATO_HARVEST_PLANNER_BACKEND", None)
            else:
                os.environ["TOMATO_HARVEST_PLANNER_BACKEND"] = previous

        self.assertEqual(info.name, "geometric_fallback")
        self.assertFalse(info.moveit2_enabled)

    def test_moveit2_service_bridge_marks_plan_when_service_validation_succeeds(self) -> None:
        validator = _AcceptingValidator()
        planner = MoveIt2ServiceBridgePlanner(bridge=validator)

        plan = planner.plan(_target_estimate(), _joint_state(), _tf_tree(), _scene_snapshot())

        self.assertEqual(plan.planner_name, "moveit2_service_bridge")
        self.assertEqual(len(validator.calls), 1)
        self.assertIsNotNone(plan.pregrasp_joint_trajectory)
        self.assertEqual(plan.pregrasp_joint_trajectory.points[-1].positions_rad[0], 0.1)
        self.assertEqual(plan.planning_scene_object_ids, ("tomato_branch", "place_tray"))

    def test_moveit2_service_bridge_falls_back_when_service_validation_fails(self) -> None:
        planner = MoveIt2ServiceBridgePlanner(bridge=_RejectingValidator())

        plan = planner.plan(_target_estimate(), _joint_state(), _tf_tree(), _scene_snapshot())

        self.assertEqual(plan.planner_name, "moveit2_service_bridge_fallback")
        self.assertIsNone(plan.pregrasp_joint_trajectory)

    def test_partial_result_preserves_pregrasp_grasp_pull_when_place_fails(self) -> None:
        """place 計画失敗時、pregrasp/grasp/pull 軌道が保持されること。

        pre_place_plan_failed の場合でもロボットが pregrasp まで動けるよう、
        partial result が pregrasp_joint_trajectory を保持していることを検証する。
        """
        planner = MoveIt2ServiceBridgePlanner(bridge=_PartialValidator())

        plan = planner.plan(_target_estimate(), _joint_state(), _tf_tree(), _scene_snapshot())

        self.assertEqual(plan.planner_name, "moveit2_service_bridge_partial")
        self.assertIsNotNone(plan.pregrasp_joint_trajectory)
        self.assertIsNotNone(plan.grasp_joint_trajectory)
        self.assertIsNotNone(plan.pull_joint_trajectory)
        self.assertIsNone(plan.place_joint_trajectory)

    def test_joint_state_clamped_when_out_of_bounds(self) -> None:
        """Isaac Sim が 0.0 スタートの場合 panda_joint4 が上限外になるのでクランプされること。

        panda_joint4 の URDF 上限は -0.069 rad。Isaac Sim はデフォルトで全関節 0.0 rad
        で初期化するため、0.0 > -0.069 となり MoveIt2 が CheckStartStateBounds でリジェクトする。
        クランプ後は -0.069 になり、他の関節はそのままであることを確認する。
        """
        all_zero_state = JointStateSnapshot(
            joint_names=(
                "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                "panda_joint5", "panda_joint6", "panda_joint7",
            ),
            positions_rad=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

        clamped = _clamp_joint_state_to_bounds(all_zero_state)

        self.assertAlmostEqual(clamped.positions_rad[0], 0.0, places=6)   # joint1: 0.0 within bounds
        self.assertAlmostEqual(clamped.positions_rad[3], -0.069, places=6) # joint4: clamped to upper bound
        self.assertAlmostEqual(clamped.positions_rad[4], 0.0, places=6)   # joint5: 0.0 within bounds

    def test_joint_state_within_bounds_unchanged(self) -> None:
        """有効な関節状態はクランプで変化しないこと。"""
        home_state = JointStateSnapshot(
            joint_names=(
                "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                "panda_joint5", "panda_joint6", "panda_joint7",
            ),
            positions_rad=(0.0, -0.4, 0.0, -2.1, 0.0, 1.7, 0.8),
        )

        clamped = _clamp_joint_state_to_bounds(home_state)

        self.assertEqual(clamped.positions_rad, home_state.positions_rad)

    def test_goal_joint_window_centers_on_current_base_joint(self) -> None:
        """pose goalへ併置するjoint1窓は現在値中心で、遠いIK枝を刈る (Issue #37)。

        abort診断の実測で、graspのpose goalがjoint1=2.45 radの遠いIK枝を選び、
        JTCが追従不能なbase旋回 (goal_tolerance_violated) を誘発していた。
        """
        window = goal_joint_window(_joint_state(), window_rad=1.5)

        self.assertIsNotNone(window)
        assert window is not None
        joint_name, position, tolerance = window
        self.assertEqual(joint_name, "panda_joint1")
        self.assertAlmostEqual(position, 0.0)
        self.assertAlmostEqual(tolerance, 1.5)

    def test_goal_joint_window_disabled_by_zero_width(self) -> None:
        self.assertIsNone(goal_joint_window(_joint_state(), window_rad=0.0))

    def test_goal_joint_window_requires_base_joint(self) -> None:
        state = JointStateSnapshot(joint_names=("panda_joint2",), positions_rad=(0.5,))
        self.assertIsNone(goal_joint_window(state, window_rad=1.5))

    def test_ik_solution_is_projected_to_arm_joints_in_order(self) -> None:
        """seed付きIK解 (最近傍IK枝) からarm関節goalを組み立てる (Issue #37)。

        /compute_ik の解はfinger等を含む全関節で返るため、arm関節だけを
        指定順に取り出してjoint-space goalにする。
        """
        goal = arm_joint_goal_from_ik_solution(
            solution_joint_names=(
                "panda_finger_joint1", "panda_joint1", "panda_joint2",
                "panda_joint3", "panda_joint4", "panda_joint5",
                "panda_joint6", "panda_joint7", "panda_finger_joint2",
            ),
            solution_positions_rad=(0.02, 0.1, -0.4, 0.0, -2.1, 0.0, 1.7, 0.8, 0.02),
            arm_joint_names=(
                "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                "panda_joint5", "panda_joint6", "panda_joint7",
            ),
        )

        self.assertIsNotNone(goal)
        assert goal is not None
        self.assertEqual(goal.joint_names[0], "panda_joint1")
        self.assertEqual(goal.positions_rad, (0.1, -0.4, 0.0, -2.1, 0.0, 1.7, 0.8))

    def test_far_ik_solution_is_rejected_by_nearness_guard(self) -> None:
        """seedから遠いIK解 (別枝) は棄却する (Issue #37)。

        avoid_collisions付きIKはseed解が衝突するとランダムリスタートで
        任意の遠い解を返し、関節限界張り付きの異常構成 (joint3=2.897等) を
        goalにしてしまう実害が観測された。距離ガードで最近傍枝だけを許す。
        """
        seed = JointStateSnapshot(
            joint_names=("panda_joint1", "panda_joint2", "panda_joint3"),
            positions_rad=(0.35, -0.55, -0.25),
        )
        near = JointStateSnapshot(
            joint_names=("panda_joint1", "panda_joint2", "panda_joint3"),
            positions_rad=(0.10, -0.30, 0.20),
        )
        far = JointStateSnapshot(
            joint_names=("panda_joint1", "panda_joint2", "panda_joint3"),
            positions_rad=(-0.39, 1.51, 2.897),
        )

        self.assertTrue(ik_goal_is_near_seed(seed=seed, goal=near, max_joint_delta_rad=1.5))
        self.assertFalse(ik_goal_is_near_seed(seed=seed, goal=far, max_joint_delta_rad=1.5))

    def test_nearness_guard_requires_matching_joints(self) -> None:
        seed = JointStateSnapshot(joint_names=("panda_joint1",), positions_rad=(0.0,))
        goal = JointStateSnapshot(joint_names=("panda_joint2",), positions_rad=(0.0,))
        self.assertFalse(ik_goal_is_near_seed(seed=seed, goal=goal, max_joint_delta_rad=1.5))

    def test_ik_solution_missing_arm_joint_is_rejected(self) -> None:
        goal = arm_joint_goal_from_ik_solution(
            solution_joint_names=("panda_joint1",),
            solution_positions_rad=(0.1,),
            arm_joint_names=("panda_joint1", "panda_joint2"),
        )
        self.assertIsNone(goal)

    def test_noop_trajectory_is_detected(self) -> None:
        self.assertTrue(
            _trajectory_is_noop(
                JointTrajectory(
                    joint_names=_joint_state().joint_names,
                    points=(JointTrajectoryPoint(_joint_state().positions_rad, 0.0),),
                ),
                start_joint_state=_joint_state(),
                tolerance_rad=1e-3,
            )
        )

    def test_nontrivial_trajectory_is_not_noop(self) -> None:
        self.assertFalse(
            _trajectory_is_noop(
                JointTrajectory(
                    joint_names=_joint_state().joint_names,
                    points=(JointTrajectoryPoint((0.1, -0.3, 0.05, -2.0, 0.1, 1.75, 0.85), 1.0),),
                ),
                start_joint_state=_joint_state(),
                tolerance_rad=1e-3,
            )
        )


if __name__ == "__main__":
    unittest.main()
