"""motion_command_node のフェーズ別 motion_command 生成ロジックのテスト。

アーキテクチャ仕様（docs/index.html）に定義した
フェーズ別 gripper_closed + joint_trajectory 種別（移動 vs 停止）を検証する。
"""
from __future__ import annotations

import unittest

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
    Pose3D,
)


def _make_trajectory(label: str = "move") -> JointTrajectory:
    return JointTrajectory(
        joint_names=("panda_joint1",),
        points=(JointTrajectoryPoint(positions_rad=(0.1,), time_from_start_sec=1.0),),
    )


def _make_plan(
    pregrasp: JointTrajectory | None = None,
    grasp: JointTrajectory | None = None,
    pull: JointTrajectory | None = None,
    place: JointTrajectory | None = None,
) -> HarvestMotionPlan:
    pose = Pose3D(0, 0, 0, 0, 0, 0)
    return HarvestMotionPlan(
        planner_name="moveit2",
        target_pose=pose,
        pregrasp_pose=pose,
        grasp_pose=pose,
        pull_pose=pose,
        place_pose=pose,
        pregrasp_joint_trajectory=pregrasp or _make_trajectory(),
        grasp_joint_trajectory=grasp or _make_trajectory(),
        pull_joint_trajectory=pull or _make_trajectory(),
        place_joint_trajectory=place or _make_trajectory(),
    )


def _make_joint_state() -> JointStateSnapshot:
    return JointStateSnapshot(
        joint_names=("panda_joint1",),
        positions_rad=(0.5,),
    )


class TestMotionCommandNodeLogic(unittest.TestCase):
    def setUp(self) -> None:
        from tomato_harvest_sim.robot.motion_command_node import build_motion_command
        self.build = build_motion_command

    # --- gripper_closed の値が仕様通りか ---

    def test_moving_to_pregrasp_gripper_closed_true(self) -> None:
        cmd = self.build(HarvestTaskPhase.MOVING_TO_PREGRASP, _make_plan(), _make_joint_state())
        self.assertTrue(cmd.gripper_closed)

    def test_moving_to_grasp_gripper_closed_false(self) -> None:
        cmd = self.build(HarvestTaskPhase.MOVING_TO_GRASP, _make_plan(), _make_joint_state())
        self.assertFalse(cmd.gripper_closed)

    def test_at_grasp_gripper_closed_true(self) -> None:
        cmd = self.build(HarvestTaskPhase.AT_GRASP, _make_plan(), _make_joint_state())
        self.assertTrue(cmd.gripper_closed)

    def test_grasp_evaluation_gripper_closed_true(self) -> None:
        cmd = self.build(HarvestTaskPhase.GRASP_EVALUATION, _make_plan(), _make_joint_state())
        self.assertTrue(cmd.gripper_closed)

    def test_detaching_gripper_closed_true(self) -> None:
        cmd = self.build(HarvestTaskPhase.DETACHING, _make_plan(), _make_joint_state())
        self.assertTrue(cmd.gripper_closed)

    def test_moving_to_place_gripper_closed_true(self) -> None:
        cmd = self.build(HarvestTaskPhase.MOVING_TO_PLACE, _make_plan(), _make_joint_state())
        self.assertTrue(cmd.gripper_closed)

    def test_placed_gripper_closed_false(self) -> None:
        cmd = self.build(HarvestTaskPhase.PLACED, _make_plan(), _make_joint_state())
        self.assertFalse(cmd.gripper_closed)

    def test_returning_home_gripper_closed_false(self) -> None:
        cmd = self.build(HarvestTaskPhase.RETURNING_HOME, _make_plan(), _make_joint_state())
        self.assertFalse(cmd.gripper_closed)

    # --- joint_trajectory が常に非 null か ---

    def test_all_motion_phases_have_non_null_trajectory(self) -> None:
        motion_phases = [
            HarvestTaskPhase.MOVING_TO_PREGRASP,
            HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.AT_GRASP,
            HarvestTaskPhase.GRASP_EVALUATION,
            HarvestTaskPhase.DETACHING,
            HarvestTaskPhase.MOVING_TO_PLACE,
            HarvestTaskPhase.PLACED,
            HarvestTaskPhase.RETURNING_HOME,
        ]
        for phase in motion_phases:
            with self.subTest(phase=phase):
                cmd = self.build(phase, _make_plan(), _make_joint_state())
                self.assertIsNotNone(cmd.phase_motion_plan)
                self.assertIsNotNone(cmd.phase_motion_plan.joint_trajectory)

    # --- 停止軌道フェーズ: 現在位置の単一ウェイポイントか ---

    def test_at_grasp_uses_stop_trajectory(self) -> None:
        joint_state = _make_joint_state()
        cmd = self.build(HarvestTaskPhase.AT_GRASP, _make_plan(), joint_state)
        traj = cmd.phase_motion_plan.joint_trajectory
        self.assertEqual(len(traj.points), 1)
        self.assertEqual(traj.points[0].positions_rad, joint_state.positions_rad)

    def test_grasp_evaluation_uses_stop_trajectory(self) -> None:
        joint_state = _make_joint_state()
        cmd = self.build(HarvestTaskPhase.GRASP_EVALUATION, _make_plan(), joint_state)
        traj = cmd.phase_motion_plan.joint_trajectory
        self.assertEqual(len(traj.points), 1)
        self.assertEqual(traj.points[0].positions_rad, joint_state.positions_rad)

    def test_placed_uses_stop_trajectory(self) -> None:
        joint_state = _make_joint_state()
        cmd = self.build(HarvestTaskPhase.PLACED, _make_plan(), joint_state)
        traj = cmd.phase_motion_plan.joint_trajectory
        self.assertEqual(len(traj.points), 1)
        self.assertEqual(traj.points[0].positions_rad, joint_state.positions_rad)

    # --- 移動軌道フェーズ: plan の軌道を使うか ---

    def test_moving_to_pregrasp_uses_plan_trajectory(self) -> None:
        pregrasp_traj = _make_trajectory("pregrasp")
        plan = _make_plan(pregrasp=pregrasp_traj)
        cmd = self.build(HarvestTaskPhase.MOVING_TO_PREGRASP, plan, _make_joint_state())
        self.assertIs(cmd.phase_motion_plan.joint_trajectory, pregrasp_traj)

    def test_moving_to_grasp_uses_plan_trajectory(self) -> None:
        grasp_traj = _make_trajectory("grasp")
        plan = _make_plan(grasp=grasp_traj)
        cmd = self.build(HarvestTaskPhase.MOVING_TO_GRASP, plan, _make_joint_state())
        self.assertIs(cmd.phase_motion_plan.joint_trajectory, grasp_traj)

    def test_detaching_uses_pull_trajectory(self) -> None:
        pull_traj = _make_trajectory("pull")
        plan = _make_plan(pull=pull_traj)
        cmd = self.build(HarvestTaskPhase.DETACHING, plan, _make_joint_state())
        self.assertIs(cmd.phase_motion_plan.joint_trajectory, pull_traj)

    def test_moving_to_place_uses_plan_trajectory(self) -> None:
        place_traj = _make_trajectory("place")
        plan = _make_plan(place=place_traj)
        cmd = self.build(HarvestTaskPhase.MOVING_TO_PLACE, plan, _make_joint_state())
        self.assertIs(cmd.phase_motion_plan.joint_trajectory, place_traj)


if __name__ == "__main__":
    unittest.main()
