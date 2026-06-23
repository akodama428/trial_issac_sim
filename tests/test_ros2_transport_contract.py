from __future__ import annotations

from pathlib import Path
import unittest

from tomato_harvest_sim.api.bridge import (
    InMemoryRos2Bridge,
    _scene_snapshot_from_dict,
    _scene_snapshot_to_dict,
    create_bridge,
)
from tomato_harvest_sim.api.contracts import ControlCommand, JointStateSnapshot
from tomato_harvest_sim.robot.moveit_service import ISAAC_PANDA_URDF, build_move_group_parameters
from tomato_harvest_sim.robot.moveit_service import write_move_group_parameters_file
from tomato_harvest_sim.simulator.scene_runtime import IsaacSceneRuntime


class Ros2TransportContractTest(unittest.TestCase):
    def test_in_memory_bridge_supports_control_and_scene_roundtrip(self) -> None:
        bridge = InMemoryRos2Bridge()
        snapshot = IsaacSceneRuntime().boot()

        bridge.publish_control(ControlCommand.START)
        bridge.publish_scene_snapshot(snapshot)

        self.assertEqual(bridge.consume_control_command(), ControlCommand.START)
        self.assertEqual(bridge.read_scene_snapshot(), snapshot)

    def test_scene_snapshot_dict_roundtrip_preserves_transport_fields(self) -> None:
        snapshot = IsaacSceneRuntime().boot()

        restored = _scene_snapshot_from_dict(_scene_snapshot_to_dict(snapshot))

        self.assertEqual(restored.phase, snapshot.phase)
        self.assertEqual(restored.active_camera, snapshot.active_camera)
        self.assertEqual(restored.tomato_pose, snapshot.tomato_pose)
        self.assertEqual(restored.motion_waypoints, snapshot.motion_waypoints)
        self.assertEqual(restored.motion_joint_trajectory, snapshot.motion_joint_trajectory)

    def test_create_bridge_honors_explicit_in_memory_override(self) -> None:
        bridge = create_bridge(transport="in_memory")

        self.assertIsInstance(bridge, InMemoryRos2Bridge)

    def test_in_memory_bridge_prefers_explicit_joint_state_over_snapshot_fallback(self) -> None:
        bridge = InMemoryRos2Bridge()
        snapshot = IsaacSceneRuntime().boot()

        bridge.publish_scene_snapshot(snapshot)
        bridge.publish_joint_state(
            JointStateSnapshot(
                joint_names=tuple(f"panda_joint{index}" for index in range(1, 8)),
                positions_rad=(0.1, -0.2, 0.3, -1.4, 0.2, 1.1, 0.7),
            )
        )

        self.assertEqual(
            bridge.read_joint_state().positions_rad,
            (0.1, -0.2, 0.3, -1.4, 0.2, 1.1, 0.7),
        )

    def test_moveit_service_parameters_include_required_sections(self) -> None:
        if not Path(ISAAC_PANDA_URDF).exists():
            self.skipTest("Isaac Sim Panda URDF is not available in this environment.")
        params = build_move_group_parameters()

        self.assertIn("robot_description", params)
        self.assertIn("robot_description_semantic", params)
        self.assertIn("robot_description_kinematics", params)
        self.assertIn("robot_description_planning", params)
        self.assertEqual(params["default_planning_pipeline"], "ompl")
        self.assertIn("ompl", params)
        self.assertIn("panda_arm", params["robot_description_kinematics"])
        self.assertIn("joint_limits", params["robot_description_planning"])
        self.assertIn("panda_joint1", params["robot_description_planning"]["joint_limits"])
        self.assertTrue(
            params["robot_description_planning"]["joint_limits"]["panda_joint1"]["has_acceleration_limits"]
        )

    def test_moveit_service_writes_ros_params_file(self) -> None:
        if not Path(ISAAC_PANDA_URDF).exists():
            self.skipTest("Isaac Sim Panda URDF is not available in this environment.")
        params_path = write_move_group_parameters_file(Path("/tmp") / "tomato-harvest-test-moveit")

        self.assertTrue(params_path.exists())
        payload = params_path.read_text(encoding="utf-8")
        self.assertIn("ros__parameters", payload)
        self.assertIn("robot_description_semantic", payload)
        self.assertIn("robot_description_planning", payload)
        self.assertIn("max_acceleration", payload)


if __name__ == "__main__":
    unittest.main()
