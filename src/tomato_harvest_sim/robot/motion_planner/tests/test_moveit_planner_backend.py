from __future__ import annotations

import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from tomato_harvest_sim.msg.contracts import (
    HarvestTaskPhase,
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
    Pose3D,
    ScenePhase,
    SceneSnapshot,
    TargetEstimate,
    TomatoStatus,
)
from tomato_harvest_sim.msg.topics import home_joint_state
from tomato_harvest_sim.robot.motion_planner import MoveIt2ServiceBridgePlanner, build_planner
from tomato_harvest_sim.robot.motion_planner.moveit_service_bridge import (
    Ros2MoveIt2PlannerBridge,
    _clamp_joint_state_to_bounds,
    _moveit_link_target_pose_from_runtime_tool_pose,
    _phase_planning_specs,
    _trajectory_is_noop,
    _tomato_planning_scene_ops,
    arm_joint_goal_from_ik_solution,
    goal_joint_window,
    ik_goal_is_near_seed,
    should_start_via_home,
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


_BASE_FRAME_ID = "panda_link0"


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

    def test_build_planner_returns_the_runtime_moveit_planner(self) -> None:
        planner = build_planner()

        self.assertIsInstance(planner, MoveIt2ServiceBridgePlanner)

    def test_initial_plan_contains_only_pose_and_waypoints(self) -> None:
        planner = MoveIt2ServiceBridgePlanner()

        plan = planner.plan(_target_estimate(), _scene_snapshot())

        self.assertEqual(plan.planner_name, "harvest_pose_waypoint_planner")
        self.assertIsNotNone(plan.pregrasp_pose)
        self.assertTrue(plan.grasp_waypoints)
        self.assertTrue(plan.pull_waypoints)
        self.assertTrue(plan.place_waypoints)
        self.assertIsNone(plan.pregrasp_joint_trajectory)
        self.assertIsNone(plan.grasp_joint_trajectory)
        self.assertIsNone(plan.pull_joint_trajectory)
        self.assertIsNone(plan.place_joint_trajectory)
        self.assertIsNone(plan.home_joint_trajectory)

    def test_standard_phase_specs_hold_targets_scene_mode_and_recovery_order(
        self,
    ) -> None:
        """標準phaseの差分を設定表で表現し、PREGRASPだけ代替経路を持つこと。"""
        pose_plan = MoveIt2ServiceBridgePlanner().plan(
            _target_estimate(),
            _scene_snapshot(),
        )
        home = home_joint_state()
        far_from_home = JointStateSnapshot(
            joint_names=home.joint_names,
            positions_rad=(2.0, *home.positions_rad[1:]),
        )

        specs = _phase_planning_specs(
            plan=pose_plan,
            joint_state=far_from_home,
            home_via_threshold_rad=1.2,
        )
        by_phase = {spec.phase: spec for spec in specs}

        self.assertEqual(
            tuple(by_phase),
            (
                HarvestTaskPhase.MOVING_TO_PREGRASP,
                HarvestTaskPhase.MOVING_TO_GRASP,
                HarvestTaskPhase.DETACHING,
                HarvestTaskPhase.MOVING_TO_PLACE,
                HarvestTaskPhase.RETURNING_HOME,
            ),
        )
        self.assertEqual(
            by_phase[HarvestTaskPhase.MOVING_TO_PREGRASP].target_sequences,
            (
                (home, pose_plan.pregrasp_pose),
                (pose_plan.pregrasp_pose,),
            ),
        )
        self.assertEqual(
            by_phase[HarvestTaskPhase.MOVING_TO_GRASP].target_sequences,
            ((pose_plan.grasp_pose,),),
        )
        self.assertTrue(
            by_phase[HarvestTaskPhase.DETACHING].attach_tomato
        )
        self.assertEqual(
            by_phase[HarvestTaskPhase.MOVING_TO_PLACE].target_sequences,
            ((pose_plan.place_waypoints[0], pose_plan.place_pose),),
        )
        self.assertTrue(
            by_phase[HarvestTaskPhase.MOVING_TO_PLACE].attach_tomato
        )
        self.assertEqual(
            by_phase[HarvestTaskPhase.RETURNING_HOME].target_sequences,
            ((home,),),
        )

    def test_place_uses_common_target_sequence_and_joint_fallback_attempt(
        self,
    ) -> None:
        """PLACEも設定表の候補列を順に試し、fallback成功理由を維持すること。"""
        class _ReadyClients:
            def wait_for_services(self, *, timeout_sec: float) -> bool:
                return True

        pose_plan = MoveIt2ServiceBridgePlanner().plan(
            _target_estimate(),
            _scene_snapshot(),
        )
        prior_place_trajectory = JointTrajectory(
            joint_names=_joint_state().joint_names,
            points=(
                JointTrajectoryPoint(_joint_state().positions_rad, 0.0),
                JointTrajectoryPoint(
                    (0.2, -0.2, 0.1, -1.9, 0.1, 1.6, 0.7),
                    1.0,
                ),
            ),
        )
        pose_plan = replace(
            pose_plan,
            place_joint_trajectory=prior_place_trajectory,
        )
        fallback_trajectory = JointTrajectory(
            joint_names=_joint_state().joint_names,
            points=prior_place_trajectory.points,
        )
        bridge = Ros2MoveIt2PlannerBridge()
        bridge._clients = _ReadyClients()
        bridge._plan_phase = Mock(side_effect=(None, fallback_trajectory))
        bridge._plan_joint_goal = Mock(return_value=fallback_trajectory)

        with patch(
            "tomato_harvest_sim.robot.motion_planner.moveit_bridge."
            "phase_planner.moveit2_python_available",
            return_value=True,
        ):
            result = bridge.plan_phase_trajectory(
                phase=HarvestTaskPhase.MOVING_TO_PLACE,
                joint_state=_joint_state(),
                base_frame_id=_BASE_FRAME_ID,
                scene_snapshot=_scene_snapshot(),
                plan=pose_plan,
            )

        self.assertTrue(result.success)
        self.assertEqual(result.reason, "joint_goal_fallback")
        self.assertEqual(bridge._plan_phase.call_count, 2)
        primary, fallback = bridge._plan_phase.call_args_list
        self.assertEqual(
            primary.kwargs["planning_targets"],
            (pose_plan.place_waypoints[0], pose_plan.place_pose),
        )
        self.assertEqual(
            fallback.kwargs["planning_targets"],
            (
                JointStateSnapshot(
                    joint_names=prior_place_trajectory.joint_names,
                    positions_rad=prior_place_trajectory.points[-1].positions_rad,
                ),
            ),
        )

    def test_detaching_is_planned_from_latest_joint_state_with_attached_tomato(
        self,
    ) -> None:
        class _ReadyClients:
            def wait_for_services(self, *, timeout_sec: float) -> bool:
                return True

        pose_plan = MoveIt2ServiceBridgePlanner().plan(
            _target_estimate(),
            _scene_snapshot(),
        )
        bridge = Ros2MoveIt2PlannerBridge()
        bridge._clients = _ReadyClients()
        trajectory = JointTrajectory(
            joint_names=_joint_state().joint_names,
            points=(
                JointTrajectoryPoint(_joint_state().positions_rad, 0.0),
                JointTrajectoryPoint(
                    (0.1, -0.3, 0.05, -2.0, 0.1, 1.75, 0.85),
                    1.0,
                ),
            ),
        )
        bridge._plan_phase = Mock(return_value=trajectory)

        with patch(
            "tomato_harvest_sim.robot.motion_planner.moveit_bridge."
            "phase_planner.moveit2_python_available",
            return_value=True,
        ):
            result = bridge.plan_phase_trajectory(
                phase=HarvestTaskPhase.DETACHING,
                joint_state=_joint_state(),
                base_frame_id=_BASE_FRAME_ID,
                scene_snapshot=_scene_snapshot(),
                plan=pose_plan,
            )

        self.assertTrue(result.success)
        self.assertEqual(result.joint_trajectory, trajectory)
        call = bridge._plan_phase.call_args
        self.assertEqual(call.kwargs["joint_state"], _joint_state())
        self.assertEqual(call.kwargs["planning_targets"], (pose_plan.pull_pose,))
        self.assertTrue(call.kwargs["attach_tomato"])

    def test_phase_target_sequence_skips_ik_for_joint_goal(self) -> None:
        """joint goalはIKを通さず、終端状態から次のpose goalを計画すること。"""
        bridge = Ros2MoveIt2PlannerBridge()
        start = _joint_state()
        home = JointStateSnapshot(
            joint_names=start.joint_names,
            positions_rad=(0.1, -0.3, 0.1, -2.0, 0.1, 1.6, 0.7),
        )
        target_pose = Pose3D(0.42, 0.0, 0.58, 0.0, 0.0, 0.0)
        home_trajectory = JointTrajectory(
            joint_names=start.joint_names,
            points=(
                JointTrajectoryPoint(start.positions_rad, 0.0),
                JointTrajectoryPoint(home.positions_rad, 1.0),
            ),
        )
        pose_end = (0.2, -0.2, 0.15, -1.9, 0.15, 1.5, 0.6)
        pose_trajectory = JointTrajectory(
            joint_names=start.joint_names,
            points=(
                JointTrajectoryPoint(home.positions_rad, 0.0),
                JointTrajectoryPoint(pose_end, 1.0),
            ),
        )
        bridge._apply_phase_planning_scene = Mock(return_value=True)
        bridge._plan_joint_goal = Mock(return_value=home_trajectory)
        bridge._plan_seeded_ik_goal = Mock(return_value=pose_trajectory)

        trajectory = bridge._plan_phase(
            clients=Mock(),
            joint_state=start,
            base_frame_id=_BASE_FRAME_ID,
            scene_snapshot=_scene_snapshot(),
            planning_targets=(home, target_pose),
            attach_tomato=False,
            phase_label="moving_to_pregrasp",
        )

        self.assertIsNotNone(trajectory)
        assert trajectory is not None
        self.assertEqual(len(trajectory.points), 3)
        self.assertEqual(trajectory.points[-1].positions_rad, pose_end)
        bridge._apply_phase_planning_scene.assert_called_once()
        bridge._plan_joint_goal.assert_called_once()
        bridge._plan_seeded_ik_goal.assert_called_once()
        self.assertEqual(
            bridge._plan_seeded_ik_goal.call_args.kwargs["joint_state"],
            home,
        )

    def test_pregrasp_via_home_failure_retries_direct_target_sequence(self) -> None:
        """via-home失敗時は同じ共通plannerで直接pregraspを再試行すること。"""
        class _ReadyClients:
            def wait_for_services(self, *, timeout_sec: float) -> bool:
                return True

        pose_plan = MoveIt2ServiceBridgePlanner().plan(
            _target_estimate(),
            _scene_snapshot(),
        )
        bridge = Ros2MoveIt2PlannerBridge()
        bridge._clients = _ReadyClients()
        direct_trajectory = JointTrajectory(
            joint_names=_joint_state().joint_names,
            points=(
                JointTrajectoryPoint(_joint_state().positions_rad, 0.0),
                JointTrajectoryPoint(
                    (0.1, -0.3, 0.05, -2.0, 0.1, 1.75, 0.85),
                    1.0,
                ),
            ),
        )
        bridge._plan_phase = Mock(side_effect=(None, direct_trajectory))

        with (
            patch(
                "tomato_harvest_sim.robot.motion_planner.moveit_bridge."
                "phase_planner.moveit2_python_available",
                return_value=True,
            ),
            patch(
                "tomato_harvest_sim.robot.motion_planner.moveit_bridge."
                "phase_policy.should_start_via_home",
                return_value=True,
            ),
        ):
            result = bridge.plan_phase_trajectory(
                phase=HarvestTaskPhase.MOVING_TO_PREGRASP,
                joint_state=_joint_state(),
                base_frame_id=_BASE_FRAME_ID,
                scene_snapshot=_scene_snapshot(),
                plan=pose_plan,
            )

        self.assertTrue(result.success)
        self.assertEqual(bridge._plan_phase.call_count, 2)
        first, second = bridge._plan_phase.call_args_list
        self.assertEqual(
            first.kwargs["planning_targets"],
            (home_joint_state(), pose_plan.pregrasp_pose),
        )
        self.assertEqual(
            second.kwargs["planning_targets"],
            (pose_plan.pregrasp_pose,),
        )

    def test_direct_pregrasp_failure_is_not_retried_as_via_home_fallback(self) -> None:
        """最初から直接計画する場合は、同じ失敗計画を重複実行しないこと。"""
        class _ReadyClients:
            def wait_for_services(self, *, timeout_sec: float) -> bool:
                return True

        pose_plan = MoveIt2ServiceBridgePlanner().plan(
            _target_estimate(),
            _scene_snapshot(),
        )
        bridge = Ros2MoveIt2PlannerBridge()
        bridge._clients = _ReadyClients()
        bridge._plan_phase = Mock(return_value=None)

        with (
            patch(
                "tomato_harvest_sim.robot.motion_planner.moveit_bridge."
                "phase_planner.moveit2_python_available",
                return_value=True,
            ),
            patch(
                "tomato_harvest_sim.robot.motion_planner.moveit_bridge."
                "phase_policy.should_start_via_home",
                return_value=False,
            ),
        ):
            result = bridge.plan_phase_trajectory(
                phase=HarvestTaskPhase.MOVING_TO_PREGRASP,
                joint_state=_joint_state(),
                base_frame_id=_BASE_FRAME_ID,
                scene_snapshot=_scene_snapshot(),
                plan=pose_plan,
            )

        self.assertFalse(result.success)
        bridge._plan_phase.assert_called_once()

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

    def test_goal_joint_window_covers_all_arm_joints(self) -> None:
        """pose goalへ併置する窓は全arm関節を現在値中心で拘束する (Issue #37)。

        joint1のみの窓では、goal samplingがjoint2/3で関節限界張り付きの
        遠いIK枝を選ぶ経路が残っていた (place固着 joint2=0.82 radを実測)。
        """
        windows = goal_joint_window(_joint_state(), window_rad=1.5)

        self.assertIsNotNone(windows)
        assert windows is not None
        self.assertEqual(len(windows), 7)
        names = [name for name, _, _ in windows]
        self.assertIn("panda_joint1", names)
        self.assertIn("panda_joint3", names)
        joint3 = next(w for w in windows if w[0] == "panda_joint3")
        self.assertAlmostEqual(joint3[1], 0.0)   # 現在値中心
        self.assertAlmostEqual(joint3[2], 1.5)   # 半幅

    def test_goal_joint_window_disabled_by_zero_width(self) -> None:
        self.assertIsNone(goal_joint_window(_joint_state(), window_rad=0.0))

    def test_goal_joint_window_requires_arm_joints(self) -> None:
        state = JointStateSnapshot(joint_names=("panda_finger_joint1",), positions_rad=(0.02,))
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

    def test_near_singularity_start_goes_via_home(self) -> None:
        """home構成との差が大きい初期姿勢はhome経由で開始する (Issue #39)。

        伸展特異姿勢近傍 (joint4差2.0 rad) ではseed収束IKも窓付きsamplerも
        不安定なため、関節空間goalで確実に計画できるhomeを経由してから
        通常ケースと同じ挙動に乗せる。
        """
        near_singularity = JointStateSnapshot(
            joint_names=(
                "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                "panda_joint5", "panda_joint6", "panda_joint7",
            ),
            positions_rad=(0.0, -0.05, 0.0, -0.10, 0.0, 0.15, 0.0),
        )
        self.assertTrue(should_start_via_home(near_singularity, threshold_rad=1.2))

    def test_near_home_start_uses_direct_pregrasp(self) -> None:
        elbow_left = JointStateSnapshot(
            joint_names=(
                "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                "panda_joint5", "panda_joint6", "panda_joint7",
            ),
            positions_rad=(0.35, -0.55, -0.25, -2.0, 0.20, 1.55, 0.55),
        )
        self.assertFalse(should_start_via_home(elbow_left, threshold_rad=1.2))

    def test_via_home_is_disabled_by_zero_threshold(self) -> None:
        far = JointStateSnapshot(
            joint_names=("panda_joint4",), positions_rad=(-0.10,),
        )
        self.assertFalse(should_start_via_home(far, threshold_rad=0.0))

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
