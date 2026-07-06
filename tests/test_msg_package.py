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


class TestRobotMsg(unittest.TestCase):
    def test_robot_msg_perception_exports_target_estimator(self) -> None:
        from tomato_harvest_sim.robot.msg.perception import TargetEstimator
        self.assertIsNotNone(TargetEstimator)

    def test_robot_msg_planner_exports_motion_planner(self) -> None:
        from tomato_harvest_sim.robot.msg.planner import (
            MotionPlanner,
            MoveIt2PlannerBridge,
            MoveIt2PlanningResult,
            PlannerBackendInfo,
        )
        self.assertIsNotNone(MotionPlanner)


if __name__ == "__main__":
    unittest.main()
