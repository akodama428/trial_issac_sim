from __future__ import annotations

import os
import unittest

from tomato_harvest_sim.api.contracts import (
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
    _moveit_link_target_pose_from_runtime_tool_pose,
    _trajectory_is_noop,
    _tomato_planning_scene_ops,
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
