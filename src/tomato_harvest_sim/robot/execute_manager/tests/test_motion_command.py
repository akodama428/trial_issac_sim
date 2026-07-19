"""execute_manager の motion_command 生成ロジックのテスト。"""
from __future__ import annotations

import unittest
from dataclasses import replace

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
    Pose3D,
)


def _make_trajectory() -> JointTrajectory:
    return JointTrajectory(
        joint_names=("panda_joint1",),
        points=(JointTrajectoryPoint(positions_rad=(0.1,), time_from_start_sec=1.0),),
    )


def _make_arm_and_finger_trajectory() -> JointTrajectory:
    return JointTrajectory(
        joint_names=("panda_joint1", "panda_finger_joint1", "panda_joint2"),
        points=(JointTrajectoryPoint(
            positions_rad=(0.1, 0.02, 0.2),
            time_from_start_sec=1.0,
            velocities_rad_s=(0.3, 0.01, 0.4),
        ),),
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


def _make_plan_for_phase(
    phase: HarvestTaskPhase,
    plan: HarvestMotionPlan | None = None,
) -> HarvestMotionPlan:
    phase_plan = replace(plan or _make_plan(), planned_from_phase=phase)
    if phase is HarvestTaskPhase.RETURNING_HOME:
        phase_plan = replace(
            phase_plan,
            home_joint_trajectory=phase_plan.home_joint_trajectory or _make_trajectory(),
        )
    return phase_plan


def _make_joint_state() -> JointStateSnapshot:
    return JointStateSnapshot(
        joint_names=("panda_joint1",),
        positions_rad=(0.5,),
    )


def _make_arm_and_finger_joint_state() -> JointStateSnapshot:
    return JointStateSnapshot(
        joint_names=(
            "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
            "panda_joint5", "panda_joint6", "panda_joint7",
            "panda_finger_joint1", "panda_finger_joint2",
        ),
        positions_rad=(0.1, 0.2, 0.3, -1.0, 0.5, 1.2, 0.7, 0.02, 0.02),
    )


class TestMotionCommandLogic(unittest.TestCase):
    def setUp(self) -> None:
        from tomato_harvest_sim.robot.execute_manager import build_motion_command
        self.build = build_motion_command

    def test_moving_to_pregrasp_gripper_closed_true(self) -> None:
        phase = HarvestTaskPhase.MOVING_TO_PREGRASP
        cmd = self.build(phase, _make_plan_for_phase(phase), _make_joint_state())
        self.assertTrue(cmd.gripper_closed)

    def test_moving_to_grasp_gripper_closed_false(self) -> None:
        phase = HarvestTaskPhase.MOVING_TO_GRASP
        cmd = self.build(phase, _make_plan_for_phase(phase), _make_joint_state())
        self.assertFalse(cmd.gripper_closed)

    def test_at_grasp_gripper_closed_true(self) -> None:
        cmd = self.build(HarvestTaskPhase.AT_GRASP, _make_plan(), _make_joint_state())
        self.assertTrue(cmd.gripper_closed)

    def test_grasp_evaluation_gripper_closed_true(self) -> None:
        cmd = self.build(HarvestTaskPhase.GRASP_EVALUATION, _make_plan(), _make_joint_state())
        self.assertTrue(cmd.gripper_closed)

    def test_detaching_gripper_closed_true(self) -> None:
        phase = HarvestTaskPhase.DETACHING
        cmd = self.build(phase, _make_plan_for_phase(phase), _make_joint_state())
        self.assertTrue(cmd.gripper_closed)

    def test_moving_to_place_gripper_closed_true(self) -> None:
        phase = HarvestTaskPhase.MOVING_TO_PLACE
        cmd = self.build(phase, _make_plan_for_phase(phase), _make_joint_state())
        self.assertTrue(cmd.gripper_closed)

    def test_placed_gripper_closed_false(self) -> None:
        cmd = self.build(HarvestTaskPhase.PLACED, _make_plan(), _make_joint_state())
        self.assertFalse(cmd.gripper_closed)

    def test_returning_home_gripper_closed_false(self) -> None:
        phase = HarvestTaskPhase.RETURNING_HOME
        cmd = self.build(phase, _make_plan_for_phase(phase), _make_joint_state())
        self.assertFalse(cmd.gripper_closed)

    def test_grasp_phases_use_pose_tracking_by_default(self) -> None:
        for phase in (
            HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.AT_GRASP,
            HarvestTaskPhase.GRASP_EVALUATION,
        ):
            cmd = self.build(phase, _make_plan_for_phase(phase), _make_joint_state())
            self.assertTrue(cmd.terminal_pose_tracking, phase)

    def test_grasp_direct_jtc_disables_pose_tracking_for_grasp_phases(self) -> None:
        for phase in (
            HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.AT_GRASP,
            HarvestTaskPhase.GRASP_EVALUATION,
        ):
            cmd = self.build(
                phase, _make_plan_for_phase(phase), _make_joint_state(),
                grasp_direct_jtc=True,
            )
            self.assertFalse(cmd.terminal_pose_tracking, phase)

    def test_grasp_direct_jtc_keeps_other_phases_unchanged(self) -> None:
        for phase in (
            HarvestTaskPhase.MOVING_TO_PREGRASP,
            HarvestTaskPhase.DETACHING,
            HarvestTaskPhase.MOVING_TO_PLACE,
            HarvestTaskPhase.RELEASING,
            HarvestTaskPhase.RETURNING_HOME,
        ):
            baseline = self.build(
                phase, _make_plan_for_phase(phase), _make_joint_state()
            )
            cmd = self.build(
                phase, _make_plan_for_phase(phase), _make_joint_state(),
                grasp_direct_jtc=True,
            )
            self.assertEqual(
                cmd.terminal_pose_tracking, baseline.terminal_pose_tracking, phase,
            )

    def test_grasp_direct_jtc_enabled_reads_environment_flag(self) -> None:
        from tomato_harvest_sim.robot.execute_manager.motion_command import (
            grasp_direct_jtc_enabled,
        )
        self.assertFalse(grasp_direct_jtc_enabled({}))
        self.assertFalse(grasp_direct_jtc_enabled({"TOMATO_HARVEST_GRASP_DIRECT_JTC": ""}))
        self.assertFalse(grasp_direct_jtc_enabled({"TOMATO_HARVEST_GRASP_DIRECT_JTC": "0"}))
        self.assertTrue(grasp_direct_jtc_enabled({"TOMATO_HARVEST_GRASP_DIRECT_JTC": "1"}))

    def test_returning_home_trajectory_excludes_finger_positions(self) -> None:
        phase = HarvestTaskPhase.RETURNING_HOME
        plan = replace(
            _make_plan_for_phase(phase),
            home_joint_trajectory=_make_arm_and_finger_trajectory(),
        )
        cmd = self.build(
            phase,
            plan,
            _make_arm_and_finger_joint_state(),
        )
        traj = cmd.phase_motion_plan.joint_trajectory

        self.assertEqual(traj.joint_names, ("panda_joint1", "panda_joint2"))
        self.assertEqual(traj.points[0].positions_rad, (0.1, 0.2))

    def test_returning_home_prefers_planned_home_trajectory(self) -> None:
        """採用済みplanにhome区間trajectoryがあれば、直行軌道より優先する (Issue #32)。"""
        from dataclasses import replace
        planned_home = JointTrajectory(
            joint_names=("panda_joint1",),
            points=(
                JointTrajectoryPoint(positions_rad=(0.5,), time_from_start_sec=0.0),
                JointTrajectoryPoint(positions_rad=(0.25,), time_from_start_sec=1.0),
                JointTrajectoryPoint(positions_rad=(0.0,), time_from_start_sec=2.0),
            ),
        )
        plan = replace(
            _make_plan_for_phase(HarvestTaskPhase.RETURNING_HOME),
            home_joint_trajectory=planned_home,
        )

        cmd = self.build(HarvestTaskPhase.RETURNING_HOME, plan, _make_joint_state())

        self.assertEqual(cmd.phase_motion_plan.joint_trajectory, planned_home)
        self.assertEqual(cmd.planner_name, plan.planner_name)
        self.assertFalse(cmd.gripper_closed)
        self.assertEqual(cmd.phase_motion_plan.phase_id.value, "returning_home")

    def test_returning_home_without_phase_plan_is_not_executable(self) -> None:
        with self.assertRaisesRegex(ValueError, "phase trajectory plan is not ready"):
            self.build(
                HarvestTaskPhase.RETURNING_HOME,
                _make_plan(),
                _make_joint_state(),
            )

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
                cmd = self.build(
                    phase, _make_plan_for_phase(phase), _make_joint_state()
                )
                self.assertIsNotNone(cmd.phase_motion_plan)
                self.assertIsNotNone(cmd.phase_motion_plan.joint_trajectory)

    def test_execution_waits_for_current_phase_trajectory_plan(self) -> None:
        from tomato_harvest_sim.robot.execute_manager.motion_command import (
            phase_plan_is_ready_for_execution,
        )

        trajectory_field_by_phase = {
            HarvestTaskPhase.MOVING_TO_PREGRASP: "pregrasp_joint_trajectory",
            HarvestTaskPhase.MOVING_TO_GRASP: "grasp_joint_trajectory",
            HarvestTaskPhase.DETACHING: "pull_joint_trajectory",
            HarvestTaskPhase.MOVING_TO_PLACE: "place_joint_trajectory",
            HarvestTaskPhase.RETURNING_HOME: "home_joint_trajectory",
        }
        pose_only_plan = replace(
            _make_plan(),
            pregrasp_joint_trajectory=None,
            grasp_joint_trajectory=None,
            pull_joint_trajectory=None,
            place_joint_trajectory=None,
            home_joint_trajectory=None,
        )
        for phase, field in trajectory_field_by_phase.items():
            with self.subTest(phase=phase):
                self.assertFalse(
                    phase_plan_is_ready_for_execution(phase, pose_only_plan)
                )
                phase_plan = replace(
                    pose_only_plan,
                    planned_from_phase=phase,
                    **{field: _make_trajectory()},
                )
                self.assertTrue(
                    phase_plan_is_ready_for_execution(phase, phase_plan)
                )
                self.assertFalse(
                    phase_plan_is_ready_for_execution(
                        phase,
                        replace(
                            phase_plan,
                            planned_from_phase=HarvestTaskPhase.TARGET_FOUND,
                        ),
                    )
                )

    def test_hold_phase_does_not_require_new_trajectory_plan(self) -> None:
        from tomato_harvest_sim.robot.execute_manager.motion_command import (
            phase_plan_is_ready_for_execution,
        )

        pose_only_plan = replace(
            _make_plan(),
            pregrasp_joint_trajectory=None,
            grasp_joint_trajectory=None,
            pull_joint_trajectory=None,
            place_joint_trajectory=None,
            home_joint_trajectory=None,
        )

        self.assertTrue(
            phase_plan_is_ready_for_execution(
                HarvestTaskPhase.AT_GRASP,
                pose_only_plan,
            )
        )

    def test_phase_plan_arriving_before_phase_notification_is_deferred(self) -> None:
        from tomato_harvest_sim.robot.execute_manager.motion_command import (
            should_defer_phase_plan,
        )

        phase_plan = _make_plan_for_phase(HarvestTaskPhase.MOVING_TO_GRASP)

        self.assertTrue(should_defer_phase_plan(
            phase_plan,
            rejection_reason="rejected_phase_mismatch",
        ))
        self.assertTrue(should_defer_phase_plan(
            phase_plan,
            rejection_reason="rejected_current_phase_unknown",
        ))
        self.assertFalse(should_defer_phase_plan(
            phase_plan,
            rejection_reason="rejected_stale_revision",
        ))
        self.assertFalse(should_defer_phase_plan(
            replace(
                phase_plan,
                planned_from_phase=HarvestTaskPhase.AT_GRASP,
            ),
            rejection_reason="rejected_phase_mismatch",
        ))

    def test_at_grasp_uses_stop_trajectory(self) -> None:
        joint_state = _make_joint_state()
        cmd = self.build(HarvestTaskPhase.AT_GRASP, _make_plan(), joint_state)
        traj = cmd.phase_motion_plan.joint_trajectory
        self.assertEqual(len(traj.points), 1)
        self.assertEqual(traj.points[0].positions_rad, joint_state.positions_rad)

    def test_stop_trajectory_excludes_finger_joints_from_arm_controller_goal(self) -> None:
        cmd = self.build(
            HarvestTaskPhase.AT_GRASP,
            _make_plan(),
            _make_arm_and_finger_joint_state(),
        )
        traj = cmd.phase_motion_plan.joint_trajectory

        self.assertEqual(
            traj.joint_names,
            tuple(f"panda_joint{index}" for index in range(1, 8)),
        )
        self.assertEqual(traj.points[0].positions_rad, (0.1, 0.2, 0.3, -1.0, 0.5, 1.2, 0.7))

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

    def test_moving_to_pregrasp_uses_plan_trajectory(self) -> None:
        pregrasp_traj = _make_trajectory()
        phase = HarvestTaskPhase.MOVING_TO_PREGRASP
        plan = _make_plan_for_phase(phase, _make_plan(pregrasp=pregrasp_traj))
        cmd = self.build(phase, plan, _make_joint_state())
        self.assertIs(cmd.phase_motion_plan.joint_trajectory, pregrasp_traj)

    def test_every_planned_phase_excludes_finger_joints_at_command_boundary(self) -> None:
        mixed_trajectory = _make_arm_and_finger_trajectory()
        plan = _make_plan(
            pregrasp=mixed_trajectory,
            grasp=mixed_trajectory,
            pull=mixed_trajectory,
            place=mixed_trajectory,
        )
        phases = (
            HarvestTaskPhase.MOVING_TO_PREGRASP,
            HarvestTaskPhase.MOVING_TO_GRASP,
            HarvestTaskPhase.DETACHING,
            HarvestTaskPhase.MOVING_TO_PLACE,
        )

        for phase in phases:
            with self.subTest(phase=phase):
                command = self.build(
                    phase,
                    _make_plan_for_phase(phase, plan),
                    _make_arm_and_finger_joint_state(),
                )
                trajectory = command.phase_motion_plan.joint_trajectory
                self.assertEqual(trajectory.joint_names, ("panda_joint1", "panda_joint2"))
                self.assertEqual(trajectory.points[0].positions_rad, (0.1, 0.2))
                self.assertEqual(trajectory.points[0].velocities_rad_s, (0.3, 0.4))

    def test_moving_to_grasp_uses_plan_trajectory(self) -> None:
        grasp_traj = _make_trajectory()
        phase = HarvestTaskPhase.MOVING_TO_GRASP
        plan = _make_plan_for_phase(phase, _make_plan(grasp=grasp_traj))
        cmd = self.build(phase, plan, _make_joint_state())
        self.assertIs(cmd.phase_motion_plan.joint_trajectory, grasp_traj)

    def test_detaching_uses_pull_trajectory(self) -> None:
        pull_traj = _make_trajectory()
        phase = HarvestTaskPhase.DETACHING
        plan = _make_plan_for_phase(phase, _make_plan(pull=pull_traj))
        cmd = self.build(phase, plan, _make_joint_state())
        self.assertIs(cmd.phase_motion_plan.joint_trajectory, pull_traj)

    def test_moving_to_place_uses_plan_trajectory(self) -> None:
        place_traj = _make_trajectory()
        phase = HarvestTaskPhase.MOVING_TO_PLACE
        plan = _make_plan_for_phase(phase, _make_plan(place=place_traj))
        cmd = self.build(phase, plan, _make_joint_state())
        self.assertIs(cmd.phase_motion_plan.joint_trajectory, place_traj)


if __name__ == "__main__":
    unittest.main()
