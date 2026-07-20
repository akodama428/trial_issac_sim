"""msg/ パッケージが正しく exports を提供することを確認するテスト。"""
from __future__ import annotations

import unittest


class TestMsgContracts(unittest.TestCase):
    def test_contracts_exports_core_types(self) -> None:
        from tomato_harvest_sim.msg.contracts import (
            ControlCommand,
            HarvestMotionPlan,
            HarvestTaskPhase,
            JointStateSnapshot,
            JointTrajectory,
            JointTrajectoryPoint,
            MotionCommand,
            PhaseId,
            PhaseMotionPlan,
            Pose3D,
            ScenePhase,
            SceneSnapshot,
            TargetEstimate,
            TfTreeSnapshot,
            TomatoStatus,
        )
        self.assertIsNotNone(MotionCommand)
        self.assertIsNotNone(HarvestMotionPlan)
        self.assertIsNotNone(Pose3D)

    def test_hardware_control_exports_types(self) -> None:
        from tomato_harvest_sim.msg.hardware_control import (
            HardwareCommandSample,
            HardwareControlPort,
            HardwareStateSample,
        )
        self.assertIsNotNone(HardwareStateSample)

    def test_topics_exports_constants(self) -> None:
        from tomato_harvest_sim.msg.topics import (
            CONTROL_TOPIC,
            FIXED_CAMERA_TOPIC,
            HARVEST_MOTION_PLAN_TOPIC,
            MOTION_COMMAND_TOPIC,
            PHASE_TOPIC,
            SCENE_SNAPSHOT_TOPIC,
            TARGET_ESTIMATE_TOPIC,
            TRAJECTORY_STATUS_TOPIC,
            EXECUTION_STATUS_TOPIC,
        )
        self.assertEqual(CONTROL_TOPIC, "/tomato_harvest/control")
        self.assertEqual(MOTION_COMMAND_TOPIC, "/tomato_harvest/motion_command")
        self.assertEqual(PHASE_TOPIC, "/tomato_harvest/phase")
        self.assertEqual(HARVEST_MOTION_PLAN_TOPIC, "/tomato_harvest/harvest_motion_plan")
        self.assertEqual(TRAJECTORY_STATUS_TOPIC, "/tomato_harvest/trajectory_status")
        self.assertEqual(EXECUTION_STATUS_TOPIC, "/tomato_harvest/execution_status")

    def test_serialization_roundtrip_motion_command(self) -> None:
        from tomato_harvest_sim.msg.contracts import MotionCommand, Pose3D
        from tomato_harvest_sim.msg.serialization import motion_command_to_json, motion_command_from_json

        cmd = MotionCommand(
            command_name="move_to_pregrasp",
            planner_name="moveit2",
            gripper_closed=True,
        )
        json_str = motion_command_to_json(cmd)
        restored = motion_command_from_json(json_str)

        self.assertEqual(restored.command_name, cmd.command_name)
        self.assertEqual(restored.gripper_closed, cmd.gripper_closed)

    def test_old_motion_command_json_defaults_execution_intent(self) -> None:
        from tomato_harvest_sim.msg.contracts import MotionKind
        from tomato_harvest_sim.msg.serialization import motion_command_from_json

        restored = motion_command_from_json(
            '{"command_name":"legacy","planner_name":"legacy",'
            '"target_pose":null,"gripper_closed":null,"phase_motion_plan":null}'
        )
        self.assertIs(restored.motion_kind, MotionKind.FOLLOW_TRAJECTORY)
        self.assertFalse(restored.terminal_pose_tracking)

    def test_serialization_roundtrip_target_estimate(self) -> None:
        from tomato_harvest_sim.msg.contracts import Pose3D, TargetEstimate
        from tomato_harvest_sim.msg.serialization import target_estimate_to_json, target_estimate_from_json

        pose = Pose3D(x=0.1, y=0.2, z=0.3, roll=0.0, pitch=0.0, yaw=0.0)
        estimate = TargetEstimate(
            camera_name="fixed_camera",
            target_world_pose=pose,
            target_camera_pose=pose,
            confidence=0.95,
        )
        json_str = target_estimate_to_json(estimate)
        restored = target_estimate_from_json(json_str)

        self.assertAlmostEqual(restored.target_world_pose.x, 0.1)
        self.assertAlmostEqual(restored.confidence, 0.95)


class TestHarvestMotionPlanContract(unittest.TestCase):
    """Step 1: plan 契約 (revision / generated_at / planned_from_phase / producer_kind)。"""

    def _minimal_plan_kwargs(self) -> dict:
        from tomato_harvest_sim.msg.contracts import Pose3D
        pose = Pose3D(x=0.1, y=0.2, z=0.3, roll=0.0, pitch=0.0, yaw=0.0)
        return {
            "planner_name": "moveit2_service_bridge",
            "target_pose": pose,
            "pregrasp_pose": pose,
            "grasp_pose": pose,
            "pull_pose": pose,
            "place_pose": pose,
        }

    def test_new_contract_metadata_roundtrips_via_json(self) -> None:
        from tomato_harvest_sim.msg.contracts import (
            HarvestMotionPlan,
            HarvestTaskPhase,
            PlanProducerKind,
        )
        from tomato_harvest_sim.msg.serialization import (
            harvest_motion_plan_from_json,
            harvest_motion_plan_to_json,
        )

        plan = HarvestMotionPlan(
            **self._minimal_plan_kwargs(),
            plan_revision=7,
            generated_at_sec=1234.5,
            planned_from_phase=HarvestTaskPhase.MOVING_TO_GRASP,
            producer_kind=PlanProducerKind.GLOBAL_PLANNER,
            producer_instance_id="global-instance-a",
        )
        restored = harvest_motion_plan_from_json(harvest_motion_plan_to_json(plan))

        self.assertEqual(restored.plan_revision, 7)
        self.assertAlmostEqual(restored.generated_at_sec, 1234.5)
        self.assertIs(restored.planned_from_phase, HarvestTaskPhase.MOVING_TO_GRASP)
        self.assertIs(restored.producer_kind, PlanProducerKind.GLOBAL_PLANNER)
        self.assertEqual(restored.producer_instance_id, "global-instance-a")

    def test_home_joint_trajectory_roundtrips_via_json(self) -> None:
        from tomato_harvest_sim.msg.contracts import (
            HarvestMotionPlan,
            JointTrajectory,
            JointTrajectoryPoint,
        )
        from tomato_harvest_sim.msg.serialization import (
            harvest_motion_plan_from_json,
            harvest_motion_plan_to_json,
        )

        home = JointTrajectory(
            joint_names=("panda_joint1", "panda_joint2"),
            points=(
                JointTrajectoryPoint((0.3, 0.2), 0.0),
                JointTrajectoryPoint((0.0, -0.4), 2.0),
            ),
        )
        plan = HarvestMotionPlan(
            **self._minimal_plan_kwargs(),
            home_joint_trajectory=home,
        )

        restored = harvest_motion_plan_from_json(harvest_motion_plan_to_json(plan))

        self.assertEqual(restored.home_joint_trajectory, home)

    def test_missing_home_joint_trajectory_defaults_to_none(self) -> None:
        """home区間trajectoryを持たない旧契約JSONも読める。"""
        from tomato_harvest_sim.msg.contracts import HarvestMotionPlan
        from tomato_harvest_sim.msg.serialization import (
            harvest_motion_plan_from_json,
            harvest_motion_plan_to_json,
        )

        plan = HarvestMotionPlan(**self._minimal_plan_kwargs())

        restored = harvest_motion_plan_from_json(harvest_motion_plan_to_json(plan))

        self.assertIsNone(restored.home_joint_trajectory)

    def test_old_contract_json_parses_as_unversioned(self) -> None:
        """旧契約 JSON (メタデータなし) はエラーにならず未刻印 (revision 0) として読める。

        採用可否は adoption policy が判定し、未刻印 plan は棄却される (fail-closed)。
        デシリアライズ層はクラッシュしないことだけを保証する。
        """
        import json
        from tomato_harvest_sim.msg.contracts import HarvestMotionPlan, PlanProducerKind
        from tomato_harvest_sim.msg.serialization import (
            harvest_motion_plan_from_json,
            harvest_motion_plan_to_dict,
        )

        old_json_dict = harvest_motion_plan_to_dict(
            HarvestMotionPlan(**self._minimal_plan_kwargs())
        )
        for new_field in (
            "plan_revision", "generated_at_sec", "planned_from_phase",
            "producer_kind", "producer_instance_id",
        ):
            old_json_dict.pop(new_field, None)

        restored = harvest_motion_plan_from_json(json.dumps(old_json_dict))

        self.assertEqual(restored.plan_revision, 0)
        self.assertIsNone(restored.generated_at_sec)
        self.assertIsNone(restored.planned_from_phase)
        self.assertIs(restored.producer_kind, PlanProducerKind.GLOBAL_PLANNER)
        self.assertIsNone(restored.producer_instance_id)

    def test_unknown_metadata_values_degrade_without_error(self) -> None:
        """未知の producer_kind / phase 値はエラーではなく安全側の値へ落ちる。"""
        import json
        from tomato_harvest_sim.msg.contracts import HarvestMotionPlan, PlanProducerKind
        from tomato_harvest_sim.msg.serialization import (
            harvest_motion_plan_from_json,
            harvest_motion_plan_to_dict,
        )

        data = harvest_motion_plan_to_dict(HarvestMotionPlan(**self._minimal_plan_kwargs()))
        data["producer_kind"] = "planner_from_the_future"
        data["planned_from_phase"] = "phase_from_the_future"

        restored = harvest_motion_plan_from_json(json.dumps(data))

        self.assertIs(restored.producer_kind, PlanProducerKind.UNKNOWN)
        self.assertIsNone(restored.planned_from_phase)


class TestRobotMsg(unittest.TestCase):
    def test_robot_msg_perception_exports_target_estimator(self) -> None:
        from tomato_harvest_sim.robot.msg.perception import TargetEstimator
        self.assertIsNotNone(TargetEstimator)

    def test_robot_msg_planner_exports_motion_planner(self) -> None:
        from tomato_harvest_sim.robot.msg.planner import (
            MotionPlanner,
            MoveIt2PlannerBridge,
            MoveIt2PlanningResult,
        )
        self.assertIsNotNone(MotionPlanner)


if __name__ == "__main__":
    unittest.main()
