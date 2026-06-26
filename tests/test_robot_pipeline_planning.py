from __future__ import annotations

import unittest

from tomato_harvest_sim.api.contracts import (
    ControlCommand,
    HarvestTaskPhase,
    JointTrajectory,
    JointTrajectoryPoint,
    MotionCommand,
    Pose3D,
    TargetEstimate,
)
from tomato_harvest_sim.app.application import create_tomato_harvest_application
from tomato_harvest_sim.robot.motion import MoveItStyleMotionPublisher
from tomato_harvest_sim.robot.perception import TomatoTargetEstimator
from tomato_harvest_sim.robot.planner import MoveItStylePreGraspPlanner
from tomato_harvest_sim.simulator.scene_config import load_scene_layout_config


class RobotPipelinePlanningTest(unittest.TestCase):
    def test_bridge_exposes_camera_joint_and_tf_observations(self) -> None:
        system = create_tomato_harvest_application()
        system.boot()

        camera_frame = system.bridge.read_camera_frame("fixed_camera")
        joint_state = system.bridge.read_joint_state()
        tf_tree = system.bridge.read_tf_tree()

        self.assertEqual(camera_frame.camera_name, "fixed_camera")
        self.assertEqual(camera_frame.topic_name, "/camera/fixed/image_raw")
        self.assertEqual(joint_state.joint_names[0], "panda_joint1")
        self.assertEqual(tf_tree.camera_frame_id, "fixed_camera_frame")
        self.assertEqual(tf_tree.target_frame_id, "target_tomato_frame")

    def test_perception_estimates_target_from_camera_frame(self) -> None:
        layout = load_scene_layout_config()
        estimator = TomatoTargetEstimator()
        system = create_tomato_harvest_application()
        system.boot()

        estimate = estimator.estimate(
            system.bridge.read_camera_frame("fixed_camera"),
            system.bridge.read_tf_tree(),
        )

        self.assertEqual(estimate.camera_name, "fixed_camera")
        self.assertEqual(estimate.target_world_pose, layout.tomato_pose)
        self.assertGreater(estimate.confidence, 0.9)

    def test_moveit_style_planner_and_motion_publisher_create_pregrasp_command(self) -> None:
        layout = load_scene_layout_config()
        system = create_tomato_harvest_application()
        system.boot()
        estimator = TomatoTargetEstimator()
        planner = MoveItStylePreGraspPlanner()
        motion = MoveItStyleMotionPublisher()

        estimate = estimator.estimate(
            system.bridge.read_camera_frame("fixed_camera"),
            system.bridge.read_tf_tree(),
        )
        plan = planner.plan(
            estimate,
            system.bridge.read_joint_state(),
            system.bridge.read_tf_tree(),
            system.simulator.snapshot(),
        )
        command = motion.build_pregrasp_command(plan)
        grasp_command = motion.build_grasp_command(plan)
        pull_command = motion.build_pull_command(plan)
        expected_pregrasp = Pose3D(
            round(layout.tomato_pose.x - 0.12, 6),
            round(layout.tomato_pose.y, 6),
            round(layout.tomato_pose.z + 0.09, 6),
            180.0,
            0.0,
            0.0,
        )
        expected_grasp_hover = Pose3D(
            round(layout.tomato_pose.x, 6),
            round(layout.tomato_pose.y, 6),
            round(layout.tomato_pose.z + 0.11, 6),
            180.0,
            0.0,
            0.0,
        )
        expected_grasp_entry = Pose3D(
            round(layout.tomato_pose.x, 6),
            round(layout.tomato_pose.y, 6),
            round(layout.tomato_pose.z + 0.07, 6),
            180.0,
            0.0,
            0.0,
        )
        expected_grasp = Pose3D(
            round(layout.tomato_pose.x, 6),
            round(layout.tomato_pose.y, 6),
            round(layout.tomato_pose.z + 0.045, 6),
            180.0,
            0.0,
            0.0,
        )
        expected_pull_lift = Pose3D(
            round(layout.tomato_pose.x - 0.02, 6),
            round(layout.tomato_pose.y, 6),
            round(layout.tomato_pose.z + 0.06, 6),
            180.0,
            0.0,
            0.0,
        )
        expected_pull = Pose3D(
            round(layout.tomato_pose.x - 0.08, 6),
            round(layout.tomato_pose.y, 6),
            round(layout.tomato_pose.z + 0.08, 6),
            180.0,
            0.0,
            0.0,
        )
        expected_place = Pose3D(0.35, -0.35, 0.57, 180.0, 0.0, 0.0)
        expected_pre_place = Pose3D(0.35, -0.35, 0.67, 180.0, 0.0, 0.0)

        self.assertEqual(plan.planner_name, "moveit2_pregrasp_demo")
        self.assertEqual(plan.pregrasp_pose, expected_pregrasp)
        self.assertEqual(plan.grasp_pose, expected_grasp)
        self.assertEqual(plan.pull_pose, expected_pull)
        self.assertEqual(plan.place_pose, expected_place)
        self.assertEqual(plan.pregrasp_waypoints, (expected_pregrasp,))
        self.assertEqual(plan.grasp_waypoints, (expected_grasp_hover, expected_grasp_entry, expected_grasp))
        self.assertEqual(plan.pull_waypoints, (expected_pull_lift, expected_pull))
        self.assertEqual(plan.place_waypoints, (expected_pre_place, expected_place))
        self.assertEqual(command.command_name, "move_to_pregrasp")
        self.assertEqual(command.target_pose, plan.pregrasp_pose)
        self.assertEqual(command.waypoint_poses, plan.pregrasp_waypoints)
        self.assertEqual(grasp_command.waypoint_poses, plan.grasp_waypoints)
        self.assertEqual(pull_command.waypoint_poses, plan.pull_waypoints)
        self.assertIsNone(command.joint_trajectory)

    def test_motion_publisher_propagates_joint_trajectory_when_present(self) -> None:
        layout = load_scene_layout_config()
        system = create_tomato_harvest_application()
        system.boot()
        motion = MoveItStyleMotionPublisher()
        trajectory = JointTrajectory(
            joint_names=("panda_joint1", "panda_joint2"),
            points=(
                JointTrajectoryPoint((0.0, -0.4), 0.0),
                JointTrajectoryPoint((0.1, -0.3), 1.0),
            ),
        )
        plan = MoveItStylePreGraspPlanner().plan(
            TargetEstimate(
                camera_name="fixed_camera",
                target_world_pose=layout.tomato_pose,
                target_camera_pose=Pose3D(0.05, 0.0, 0.20, 0.0, 0.0, 0.0),
                confidence=1.0,
            ),
            system.bridge.read_joint_state(),
            system.bridge.read_tf_tree(),
            system.simulator.snapshot(),
        )
        plan = plan.__class__(
            planner_name=plan.planner_name,
            target_pose=plan.target_pose,
            pregrasp_pose=plan.pregrasp_pose,
            grasp_pose=plan.grasp_pose,
            pull_pose=plan.pull_pose,
            place_pose=plan.place_pose,
            pregrasp_waypoints=plan.pregrasp_waypoints,
            grasp_waypoints=plan.grasp_waypoints,
            pull_waypoints=plan.pull_waypoints,
            place_waypoints=plan.place_waypoints,
            pregrasp_joint_trajectory=trajectory,
            grasp_joint_trajectory=None,
            pull_joint_trajectory=None,
            place_joint_trajectory=None,
            planning_scene_object_ids=(),
        )

        command = motion.build_pregrasp_command(plan)

        self.assertEqual(command.joint_trajectory, trajectory)

    def test_start_progresses_to_pregrasp_reached_and_updates_simulator_proxy(self) -> None:
        layout = load_scene_layout_config()
        system = create_tomato_harvest_application()
        system.boot()

        system.apply_control(ControlCommand.START)
        logs: list[str] = []
        for _ in range(16):
            logs.extend(system.step())
            if system.robot.state.task_phase is HarvestTaskPhase.PREGRASP_REACHED:
                break

        self.assertEqual(system.robot.state.task_phase, HarvestTaskPhase.PREGRASP_REACHED)
        expected_pregrasp = Pose3D(
            round(layout.tomato_pose.x - 0.12, 6),
            round(layout.tomato_pose.y, 6),
            round(layout.tomato_pose.z + 0.09, 6),
            180.0,
            0.0,
            0.0,
        )
        expected_grasp_log = (
            f"Grasp world xyz: ({layout.tomato_pose.x:.4f}, {layout.tomato_pose.y:.4f}, "
            f"{layout.tomato_pose.z + 0.045:.4f})"
        )
        self.assertEqual(system.bridge.state.last_motion_command, MotionCommand(
            command_name="move_to_pregrasp",
            planner_name="moveit2_pregrasp_demo",
            target_pose=expected_pregrasp,
            waypoint_poses=(expected_pregrasp,),
        ))
        self.assertEqual(system.simulator.state.pregrasp_pose, expected_pregrasp)
        self.assertLess(abs(system.simulator.state.robot_tool_pose.x - expected_pregrasp.x), 0.03)
        self.assertLess(abs(system.simulator.state.robot_tool_pose.z - expected_pregrasp.z), 0.03)
        self.assertTrue(any("Target is Found!" in line for line in logs))
        self.assertTrue(any("[State] detecting -> target_found" in line for line in logs))
        self.assertTrue(any("[State] target_found -> planning" in line for line in logs))
        self.assertTrue(any("[State] planning -> moving_to_pregrasp" in line for line in logs))
        self.assertTrue(any("[Approach] Waiting for pre-grasp convergence." in line for line in logs))
        self.assertTrue(any("[State] moving_to_pregrasp -> pregrasp_reached" in line for line in logs))
        self.assertTrue(any("pre-grasp reached" in line for line in logs))
        self.assertTrue(any(expected_grasp_log in line for line in logs))

    def test_grasp_target_pose_is_propagated_to_motion_command_and_simulator(self) -> None:
        layout = load_scene_layout_config()
        system = create_tomato_harvest_application(transport="in_memory", autostart_moveit_service=False)
        system.boot()

        system.apply_control(ControlCommand.START)
        logs: list[str] = []
        for _ in range(12):
            logs.extend(system.step())
            if system.robot.state.task_phase is HarvestTaskPhase.MOVING_TO_GRASP:
                break

        self.assertEqual(system.robot.state.task_phase, HarvestTaskPhase.MOVING_TO_GRASP)
        self.assertIsNotNone(system.robot.state.last_pregrasp_plan)
        expected_grasp_hover = Pose3D(
            round(layout.tomato_pose.x, 6),
            round(layout.tomato_pose.y, 6),
            round(layout.tomato_pose.z + 0.11, 6),
            180.0,
            0.0,
            0.0,
        )
        expected_grasp_entry = Pose3D(
            round(layout.tomato_pose.x, 6),
            round(layout.tomato_pose.y, 6),
            round(layout.tomato_pose.z + 0.07, 6),
            180.0,
            0.0,
            0.0,
        )
        expected_grasp = Pose3D(
            round(layout.tomato_pose.x, 6),
            round(layout.tomato_pose.y, 6),
            round(layout.tomato_pose.z + 0.045, 6),
            180.0,
            0.0,
            0.0,
        )
        expected_grasp_log = (
            f"Grasp command target xyz: ({expected_grasp.x:.4f}, {expected_grasp.y:.4f}, {expected_grasp.z:.4f})"
        )
        self.assertEqual(system.robot.state.last_pregrasp_plan.grasp_pose, expected_grasp)
        self.assertEqual(
            system.bridge.state.last_motion_command,
            MotionCommand(
                command_name="move_to_grasp",
                planner_name=system.robot.state.planner_backend_name,
                target_pose=expected_grasp,
                waypoint_poses=(expected_grasp_hover, expected_grasp_entry, expected_grasp),
            ),
        )
        self.assertEqual(system.simulator.state.grasp_pose, expected_grasp)
        self.assertTrue(any(expected_grasp_log in line for line in logs))

    def test_simulator_runtime_advances_waypoints_in_order(self) -> None:
        system = create_tomato_harvest_application()
        system.boot()

        system.simulator.apply_motion_command(MotionCommand(
            command_name="pull_to_detach",
            planner_name="moveit2_pregrasp_demo",
            target_pose=Pose3D(0.34, 0.00, 0.62, 180.0, 0.0, 0.0),
            waypoint_poses=(
                Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
                Pose3D(0.42, 0.00, 0.54, 180.0, 0.0, 0.0),
                Pose3D(0.34, 0.00, 0.62, 180.0, 0.0, 0.0),
            ),
        ))

        first_snapshot = system.simulator.snapshot()
        self.assertEqual(first_snapshot.target_tool_pose, Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0))

        target_history: list[Pose3D] = [first_snapshot.target_tool_pose]
        for _ in range(24):
            snapshot = system.simulator.advance()
            if snapshot.target_tool_pose != target_history[-1]:
                target_history.append(snapshot.target_tool_pose)

        self.assertEqual(target_history, [
            Pose3D(0.30, 0.00, 0.57, 180.0, 0.0, 0.0),
            Pose3D(0.42, 0.00, 0.54, 180.0, 0.0, 0.0),
            Pose3D(0.34, 0.00, 0.62, 180.0, 0.0, 0.0),
        ])
        self.assertLess(abs(system.simulator.state.robot_tool_pose.x - 0.34), 0.03)
        self.assertLess(abs(system.simulator.state.robot_tool_pose.z - 0.62), 0.03)


if __name__ == "__main__":
    unittest.main()
