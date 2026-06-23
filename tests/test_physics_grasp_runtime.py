from __future__ import annotations

import unittest

from tomato_harvest_sim.api.contracts import MotionCommand, Pose3D, TomatoStatus
from tomato_harvest_sim.simulator.scene_config import load_scene_layout_config
from tomato_harvest_sim.simulator.physics_harvest import IsaacPhysicsHarvestBridge, PhysicsHarvestScenePaths
from tomato_harvest_sim.simulator.scene_runtime import IsaacSceneRuntime


class PhysicsGraspRuntimeTest(unittest.TestCase):
    def test_contact_batches_are_accumulated_within_one_physics_step(self) -> None:
        bridge = IsaacPhysicsHarvestBridge(
            stage=object(),
            scene_paths=PhysicsHarvestScenePaths(
                ground_prim_path="/World/GroundPlane",
                tray_prim_path="/World/PlaceTray",
                tomato_prim_path="/World/TargetTomato",
                stem_anchor_prim_path="/World/TomatoStemAnchor",
                stem_joint_prim_path="/World/TomatoStemJoint",
                grasp_joint_prim_path="/World/TomatoGraspJoint",
                hand_mount_prim_path="/World/FrankaPanda/panda_hand",
            ),
            initial_tomato_pose=Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0),
        )

        bridge.begin_physics_step()
        bridge._accumulate_pending_contacts({"left"})
        bridge._accumulate_pending_contacts({"right"})

        self.assertEqual(bridge._pending_finger_contacts, {"left", "right"})

    def test_contacts_can_be_latched_across_multiple_physics_steps_while_gripper_is_closed(self) -> None:
        bridge = IsaacPhysicsHarvestBridge(
            stage=object(),
            scene_paths=PhysicsHarvestScenePaths(
                ground_prim_path="/World/GroundPlane",
                tray_prim_path="/World/PlaceTray",
                tomato_prim_path="/World/TargetTomato",
                stem_anchor_prim_path="/World/TomatoStemAnchor",
                stem_joint_prim_path="/World/TomatoStemJoint",
                grasp_joint_prim_path="/World/TomatoGraspJoint",
                hand_mount_prim_path="/World/FrankaPanda/panda_hand",
            ),
            initial_tomato_pose=Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0),
        )

        bridge._accumulate_pending_contacts({"left"})
        bridge._promote_pending_contacts(gripper_closed=True)
        bridge.begin_physics_step()
        bridge._accumulate_pending_contacts({"right"})
        bridge._promote_pending_contacts(gripper_closed=True)

        self.assertEqual(bridge._latched_finger_contacts, {"left", "right"})

    def test_open_frame_contacts_are_carried_into_the_first_closed_frame(self) -> None:
        bridge = IsaacPhysicsHarvestBridge(
            stage=object(),
            scene_paths=PhysicsHarvestScenePaths(
                ground_prim_path="/World/GroundPlane",
                tray_prim_path="/World/PlaceTray",
                tomato_prim_path="/World/TargetTomato",
                stem_anchor_prim_path="/World/TomatoStemAnchor",
                stem_joint_prim_path="/World/TomatoStemJoint",
                grasp_joint_prim_path="/World/TomatoGraspJoint",
                hand_mount_prim_path="/World/FrankaPanda/panda_hand",
            ),
            initial_tomato_pose=Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0),
        )

        bridge._accumulate_pending_contacts({"left", "right"})
        bridge._promote_pending_contacts(gripper_closed=False)
        bridge.begin_physics_step()
        bridge._promote_pending_contacts(gripper_closed=True)

        self.assertEqual(bridge._latched_finger_contacts, {"left", "right"})

    def test_geometry_fallback_infers_both_finger_contacts_when_event_reports_are_missing(self) -> None:
        bridge = IsaacPhysicsHarvestBridge(
            stage=object(),
            scene_paths=PhysicsHarvestScenePaths(
                ground_prim_path="/World/GroundPlane",
                tray_prim_path="/World/PlaceTray",
                tomato_prim_path="/World/TargetTomato",
                stem_anchor_prim_path="/World/TomatoStemAnchor",
                stem_joint_prim_path="/World/TomatoStemJoint",
                grasp_joint_prim_path="/World/TomatoGraspJoint",
                hand_mount_prim_path="/World/FrankaPanda/panda_hand",
            ),
            initial_tomato_pose=Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0),
        )

        poses = {
            "/World/FrankaPanda/panda_hand": Pose3D(0.62, 0.0, 0.6431, 0.0, 0.0, 0.0),
            "/World/FrankaPanda/panda_leftfinger": Pose3D(0.62, -0.03, 0.5847, 0.0, 0.0, 0.0),
            "/World/FrankaPanda/panda_rightfinger": Pose3D(0.62, 0.03, 0.5847, 0.0, 0.0, 0.0),
            "/World/TargetTomato": Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0),
        }

        bridge._world_pose = lambda prim_path: poses[prim_path]  # type: ignore[method-assign]

        contacts = bridge._infer_finger_contacts_from_geometry(poses["/World/TargetTomato"])

        self.assertEqual(contacts, {"left", "right"})

    def test_geometry_fallback_does_not_create_contacts_without_recent_physical_contact(self) -> None:
        bridge = IsaacPhysicsHarvestBridge(
            stage=object(),
            scene_paths=PhysicsHarvestScenePaths(
                ground_prim_path="/World/GroundPlane",
                tray_prim_path="/World/PlaceTray",
                tomato_prim_path="/World/TargetTomato",
                stem_anchor_prim_path="/World/TomatoStemAnchor",
                stem_joint_prim_path="/World/TomatoStemJoint",
                grasp_joint_prim_path="/World/TomatoGraspJoint",
                hand_mount_prim_path="/World/FrankaPanda/panda_hand",
            ),
            initial_tomato_pose=Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0),
        )

        poses = {
            "/World/FrankaPanda/panda_hand": Pose3D(0.62, 0.0, 0.6431, 0.0, 0.0, 0.0),
            "/World/FrankaPanda/panda_leftfinger": Pose3D(0.62, -0.03, 0.5847, 0.0, 0.0, 0.0),
            "/World/FrankaPanda/panda_rightfinger": Pose3D(0.62, 0.03, 0.5847, 0.0, 0.0, 0.0),
            "/World/TargetTomato": Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0),
        }

        bridge._world_pose = lambda prim_path: poses[prim_path]  # type: ignore[method-assign]

        bridge._augment_contacts_from_grasp_geometry(
            tomato_pose=poses["/World/TargetTomato"],
            gripper_closed=True,
        )

        self.assertEqual(bridge._latched_finger_contacts, set())

    def test_geometry_fallback_can_complete_the_second_finger_after_one_real_contact(self) -> None:
        bridge = IsaacPhysicsHarvestBridge(
            stage=object(),
            scene_paths=PhysicsHarvestScenePaths(
                ground_prim_path="/World/GroundPlane",
                tray_prim_path="/World/PlaceTray",
                tomato_prim_path="/World/TargetTomato",
                stem_anchor_prim_path="/World/TomatoStemAnchor",
                stem_joint_prim_path="/World/TomatoStemJoint",
                grasp_joint_prim_path="/World/TomatoGraspJoint",
                hand_mount_prim_path="/World/FrankaPanda/panda_hand",
            ),
            initial_tomato_pose=Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0),
        )

        poses = {
            "/World/FrankaPanda/panda_hand": Pose3D(0.62, 0.0, 0.6431, 0.0, 0.0, 0.0),
            "/World/FrankaPanda/panda_leftfinger": Pose3D(0.62, -0.03, 0.5847, 0.0, 0.0, 0.0),
            "/World/FrankaPanda/panda_rightfinger": Pose3D(0.62, 0.03, 0.5847, 0.0, 0.0, 0.0),
            "/World/TargetTomato": Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0),
        }

        bridge._world_pose = lambda prim_path: poses[prim_path]  # type: ignore[method-assign]
        bridge._recent_finger_contacts = {"left"}
        bridge._recent_contact_grace_steps_remaining = 2

        bridge._augment_contacts_from_grasp_geometry(
            tomato_pose=poses["/World/TargetTomato"],
            gripper_closed=True,
        )

        self.assertEqual(bridge._latched_finger_contacts, {"left", "right"})

    def test_geometry_fallback_rejects_a_wide_gripper_even_if_the_hand_mount_is_centered(self) -> None:
        bridge = IsaacPhysicsHarvestBridge(
            stage=object(),
            scene_paths=PhysicsHarvestScenePaths(
                ground_prim_path="/World/GroundPlane",
                tray_prim_path="/World/PlaceTray",
                tomato_prim_path="/World/TargetTomato",
                stem_anchor_prim_path="/World/TomatoStemAnchor",
                stem_joint_prim_path="/World/TomatoStemJoint",
                grasp_joint_prim_path="/World/TomatoGraspJoint",
                hand_mount_prim_path="/World/FrankaPanda/panda_hand",
            ),
            initial_tomato_pose=Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0),
        )

        poses = {
            "/World/FrankaPanda/panda_hand": Pose3D(0.62, 0.0, 0.6431, 0.0, 0.0, 0.0),
            "/World/FrankaPanda/panda_leftfinger": Pose3D(0.62, -0.04, 0.5847, 0.0, 0.0, 0.0),
            "/World/FrankaPanda/panda_rightfinger": Pose3D(0.62, 0.04, 0.5847, 0.0, 0.0, 0.0),
            "/World/TargetTomato": Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0),
        }

        bridge._world_pose = lambda prim_path: poses[prim_path]  # type: ignore[method-assign]

        contacts = bridge._infer_finger_contacts_from_geometry(poses["/World/TargetTomato"])

        self.assertEqual(contacts, set())

    def test_physics_mode_waits_for_external_grasp_and_detach_updates(self) -> None:
        layout = load_scene_layout_config()
        runtime = IsaacSceneRuntime(physics_grasp_enabled=True)
        runtime.boot()

        runtime.apply_motion_command(
            MotionCommand(
                command_name="move_to_grasp",
                planner_name="moveit2_grasp_demo",
                target_pose=Pose3D(
                    layout.tomato_pose.x,
                    layout.tomato_pose.y,
                    layout.tomato_pose.z + 0.045,
                    180.0,
                    0.0,
                    0.0,
                ),
            )
        )
        for _ in range(16):
            runtime.advance()

        runtime.apply_motion_command(
            MotionCommand(
                command_name="close_gripper",
                planner_name="moveit2_grasp_demo",
            )
        )
        after_close = runtime.snapshot()
        self.assertEqual(after_close.tomato_status, TomatoStatus.ATTACHED)
        self.assertTrue(after_close.tomato_attached)

        runtime.sync_tomato_physics(
            layout.tomato_pose,
            attached=True,
            status=TomatoStatus.HELD,
            reason="stable_grasp_established_physx",
        )
        held_snapshot = runtime.snapshot()
        self.assertEqual(held_snapshot.tomato_status, TomatoStatus.HELD)

        runtime.apply_motion_command(
            MotionCommand(
                command_name="pull_to_detach",
                planner_name="moveit2_grasp_demo",
                target_pose=Pose3D(0.34, 0.00, 0.62, 180.0, 0.0, 0.0),
            )
        )
        before_sync = runtime.snapshot()
        self.assertEqual(before_sync.tomato_status, TomatoStatus.HELD)

        runtime.sync_tomato_physics(
            Pose3D(0.34, 0.00, 0.62, 0.0, 0.0, 0.0),
            attached=False,
            status=TomatoStatus.DETACHED,
            reason="tomato_detached_from_stem_physx",
        )
        detached_snapshot = runtime.snapshot()
        self.assertEqual(detached_snapshot.tomato_status, TomatoStatus.DETACHED)
        self.assertFalse(detached_snapshot.tomato_attached)

    def test_stable_grasp_then_pull_detaches_tomato(self) -> None:
        layout = load_scene_layout_config()
        runtime = IsaacSceneRuntime()
        runtime.boot()

        runtime.apply_motion_command(
            MotionCommand(
                command_name="move_to_grasp",
                planner_name="moveit2_grasp_demo",
                target_pose=Pose3D(
                    layout.tomato_pose.x,
                    layout.tomato_pose.y,
                    layout.tomato_pose.z + 0.045,
                    180.0,
                    0.0,
                    0.0,
                ),
            )
        )
        for _ in range(16):
            runtime.advance()
        runtime.apply_motion_command(
            MotionCommand(
                command_name="close_gripper",
                planner_name="moveit2_grasp_demo",
            )
        )

        held_snapshot = runtime.snapshot()
        self.assertEqual(held_snapshot.tomato_status, TomatoStatus.HELD)
        self.assertTrue(held_snapshot.gripper_closed)

        runtime.apply_motion_command(
            MotionCommand(
                command_name="pull_to_detach",
                planner_name="moveit2_grasp_demo",
                target_pose=Pose3D(0.34, 0.00, 0.62, 180.0, 0.0, 0.0),
            )
        )
        runtime.advance()

        detached_snapshot = runtime.snapshot()
        self.assertFalse(detached_snapshot.tomato_attached)
        self.assertEqual(detached_snapshot.tomato_status, TomatoStatus.DETACHED)
        self.assertLess(detached_snapshot.tomato_pose.x, layout.tomato_pose.x)
        self.assertGreater(detached_snapshot.tomato_pose.z, layout.tomato_pose.z)

        for _ in range(16):
            runtime.advance()

        final_snapshot = runtime.snapshot()
        self.assertLess(abs(final_snapshot.tomato_pose.x - 0.34), 0.03)
        self.assertLess(abs(final_snapshot.tomato_pose.z - 0.575), 0.03)

    def test_missed_grasp_makes_tomato_fall(self) -> None:
        layout = load_scene_layout_config()
        runtime = IsaacSceneRuntime()
        runtime.boot()

        runtime.apply_motion_command(
            MotionCommand(
                command_name="move_to_grasp",
                planner_name="moveit2_grasp_demo",
                target_pose=Pose3D(layout.tomato_pose.x, 0.08, layout.tomato_pose.z, 180.0, 0.0, 0.0),
            )
        )
        for _ in range(16):
            runtime.advance()
        runtime.apply_motion_command(
            MotionCommand(
                command_name="close_gripper",
                planner_name="moveit2_grasp_demo",
            )
        )

        before_fall = runtime.snapshot()
        runtime.advance()
        after_fall = runtime.snapshot()

        self.assertEqual(before_fall.tomato_status, TomatoStatus.FALLEN)
        self.assertLess(after_fall.tomato_pose.z, before_fall.tomato_pose.z)
        self.assertEqual(after_fall.grasp_result_reason, "grasp_missed_tomato")


if __name__ == "__main__":
    unittest.main()
