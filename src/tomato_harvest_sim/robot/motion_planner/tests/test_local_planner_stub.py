"""local planner stub — producer複線化の最小受け皿のテスト (Issue #13)。"""
from __future__ import annotations

import unittest
from dataclasses import replace

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
    PlanProducerKind,
    Pose3D,
)
from tomato_harvest_sim.robot.motion_planner.local_planner_stub import (
    build_local_refinement_plan,
)


def _base_plan() -> HarvestMotionPlan:
    pose = Pose3D(0, 0, 0, 0, 0, 0)
    trajectory = JointTrajectory(
        joint_names=("joint1", "joint2"),
        points=(JointTrajectoryPoint((0.0, 0.0), 0.0),
                JointTrajectoryPoint((1.0, 1.0), 1.0)),
    )
    return HarvestMotionPlan(
        planner_name="moveit2_service_bridge",
        target_pose=pose, pregrasp_pose=pose, grasp_pose=pose,
        pull_pose=pose, place_pose=pose,
        pregrasp_joint_trajectory=trajectory,
        place_joint_trajectory=trajectory,
        plan_revision=3,
        generated_at_sec=100.0,
        planned_from_phase=HarvestTaskPhase.TARGET_FOUND,
        producer_kind=PlanProducerKind.GLOBAL_PLANNER,
        producer_instance_id="global-instance-a",
    )


class LocalRefinementPlanTest(unittest.TestCase):
    def test_refinement_is_stamped_as_independent_local_producer(self) -> None:
        candidate = build_local_refinement_plan(
            base_plan=_base_plan(),
            phase=HarvestTaskPhase.MOVING_TO_PLACE,
            current_joint_state=JointStateSnapshot(
                joint_names=("joint1", "joint2"), positions_rad=(0.4, 0.2)
            ),
            now_sec=200.0,
            instance_id="local-instance-a",
            revision=1,
        )

        self.assertIsNotNone(candidate)
        assert candidate is not None
        self.assertEqual(candidate.producer_kind, PlanProducerKind.LOCAL_PLANNER)
        self.assertEqual(candidate.producer_instance_id, "local-instance-a")
        self.assertEqual(candidate.plan_revision, 1)
        self.assertEqual(candidate.generated_at_sec, 200.0)
        self.assertEqual(
            candidate.planned_from_phase, HarvestTaskPhase.MOVING_TO_PLACE
        )
        self.assertEqual(candidate.planner_name, "joint_space_local_planner")

    def test_executor_contract_fields_are_preserved(self) -> None:
        """下流(motion_command / executor)が読むtrajectory契約を変えない。"""
        base = _base_plan()
        candidate = build_local_refinement_plan(
            base_plan=base,
            phase=HarvestTaskPhase.MOVING_TO_PLACE,
            current_joint_state=JointStateSnapshot(
                joint_names=("joint1", "joint2"), positions_rad=(0.4, 0.2)
            ),
            now_sec=200.0,
            instance_id="local-instance-a",
            revision=1,
        )

        assert candidate is not None
        self.assertEqual(
            candidate.pregrasp_joint_trajectory, base.pregrasp_joint_trajectory
        )
        self.assertEqual(candidate.place_pose, base.place_pose)

    def test_refinement_rebases_active_trajectory_without_changing_global_goal(self) -> None:
        """接続軌道は現在関節状態から始まり、採用済みplanの終端構成で終わる。"""
        candidate = build_local_refinement_plan(
            base_plan=_base_plan(),
            phase=HarvestTaskPhase.MOVING_TO_PLACE,
            current_joint_state=JointStateSnapshot(
                joint_names=("joint2", "joint1"), positions_rad=(0.2, 0.4)
            ),
            now_sec=200.0,
            instance_id="local-instance-a",
            revision=1,
        )

        assert candidate is not None
        trajectory = candidate.place_joint_trajectory
        assert trajectory is not None
        # 現在状態 (joint2=0.2, joint1=0.4) を trajectory の関節順へ並べ替えて開始する
        self.assertEqual(trajectory.joint_names, ("joint1", "joint2"))
        self.assertEqual(trajectory.points[0].positions_rad, (0.4, 0.2))
        self.assertEqual(trajectory.points[-1].positions_rad, (1.0, 1.0))
        self.assertEqual(trajectory.points[-1].velocities_rad_s, (0.0, 0.0))
        times = [point.time_from_start_sec for point in trajectory.points]
        self.assertEqual(times, sorted(times))
        self.assertGreater(
            trajectory.points[-1].time_from_start_sec,
            trajectory.points[-2].time_from_start_sec,
        )
        self.assertEqual(candidate.planner_name, "joint_space_local_planner")

    def test_connection_duration_respects_joint_velocity_limit(self) -> None:
        """最大関節差分 / 制限速度 より短い時間で終端へ到達しない。"""
        candidate = build_local_refinement_plan(
            base_plan=_base_plan(),
            phase=HarvestTaskPhase.MOVING_TO_PLACE,
            current_joint_state=JointStateSnapshot(
                joint_names=("joint1", "joint2"), positions_rad=(0.4, 0.2)
            ),
            now_sec=200.0,
            instance_id="local-instance-a",
            revision=1,
        )

        assert candidate is not None
        trajectory = candidate.place_joint_trajectory
        assert trajectory is not None
        # 最大差分 0.8 rad / 0.5 rad/s = 1.6 s (最後の静止点は除く)
        motion_end_sec = trajectory.points[-2].time_from_start_sec
        self.assertAlmostEqual(motion_end_sec, 1.6, places=6)

    def test_near_goal_state_still_produces_executable_trajectory(self) -> None:
        """既に終端近傍でも、実行可能な非ゼロ長の接続軌道を返す。"""
        candidate = build_local_refinement_plan(
            base_plan=_base_plan(),
            phase=HarvestTaskPhase.MOVING_TO_PLACE,
            current_joint_state=JointStateSnapshot(
                joint_names=("joint1", "joint2"), positions_rad=(1.0, 1.0)
            ),
            now_sec=200.0,
            instance_id="local-instance-a",
            revision=1,
        )

        assert candidate is not None
        trajectory = candidate.place_joint_trajectory
        assert trajectory is not None
        self.assertEqual(trajectory.points[-1].positions_rad, (1.0, 1.0))
        self.assertGreaterEqual(trajectory.points[-2].time_from_start_sec, 0.5)

    def test_contact_dominant_phase_is_not_refined(self) -> None:
        candidate = build_local_refinement_plan(
            base_plan=_base_plan(),
            phase=HarvestTaskPhase.DETACHING,
            current_joint_state=JointStateSnapshot(
                joint_names=("joint1", "joint2"), positions_rad=(0.0, 0.0)
            ),
            now_sec=200.0,
            instance_id="local-instance-a",
            revision=1,
        )
        self.assertIsNone(candidate)

    def test_missing_phase_trajectory_is_not_refined(self) -> None:
        base = replace(_base_plan(), grasp_joint_trajectory=None)
        candidate = build_local_refinement_plan(
            base_plan=base,
            phase=HarvestTaskPhase.MOVING_TO_GRASP,
            current_joint_state=JointStateSnapshot(
                joint_names=("joint1", "joint2"), positions_rad=(0.0, 0.0)
            ),
            now_sec=200.0,
            instance_id="local-instance-a",
            revision=1,
        )
        self.assertIsNone(candidate)


if __name__ == "__main__":
    unittest.main()
