from __future__ import annotations

import math
import unittest
from unittest.mock import patch

from tomato_harvest_sim.msg.contracts import Pose3D, TomatoStatus
from tomato_harvest_sim.simulator.physics_harvest import IsaacPhysicsHarvestBridge, PhysicsHarvestScenePaths


class PhysicsGraspRuntimeTest(unittest.TestCase):
    def test_blank_optional_evaluation_steps_are_treated_as_disabled(self) -> None:
        with patch.dict(
            "os.environ",
            {
                "TOMATO_HARVEST_FRICTION_HOLD_EVAL_STEPS": "",
                "TOMATO_HARVEST_FRICTION_HOLD_EVAL_MIN_LIFT_M": "",
                "TOMATO_HARVEST_STEM_BREAK_NON_PULL_STEPS": "",
            },
        ):
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
                initial_tomato_pose=Pose3D(
                    0.62, 0.0, 0.54, 0.0, 0.0, 0.0
                ),
            )

        self.assertIsNone(bridge._hold_evaluator)

    def test_usd_angular_velocity_is_converted_from_degrees_to_radians(self) -> None:
        converted = IsaacPhysicsHarvestBridge._degrees_to_radians_per_second(
            (180.0, -90.0, 0.0)
        )

        self.assertAlmostEqual(converted[0], math.pi)
        self.assertAlmostEqual(converted[1], -math.pi / 2.0)
        self.assertEqual(converted[2], 0.0)

    def test_physics_mode_requires_joint_break_event_for_detached(self) -> None:
        self.assertFalse(
            IsaacPhysicsHarvestBridge._should_report_detached(
                grasp_mode="physics",
                detach_intent_active=True,
                stem_break_observed=False,
                stem_distance_m=0.1,
            )
        )

    def test_compliant_stem_joint_frames_pin_only_the_branch_side(self) -> None:
        frames = IsaacPhysicsHarvestBridge._compliant_stem_joint_frames(
            tomato_pose=Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0),
            stem_pose=Pose3D(0.62, 0.0, 0.58, 0.0, 0.0, 0.0),
            stem_length_m=0.06,
            tomato_radius_m=0.01,
        )

        self.assertEqual(frames.tomato_stem_local, (0.0, 0.0, 0.01))
        self.assertEqual(frames.stem_tomato_local, (0.0, 0.0, -0.03))
        self.assertEqual(frames.stem_pin_local, (0.0, 0.0, 0.03))
        self.assertAlmostEqual(frames.world_pin[0], 0.62)
        self.assertAlmostEqual(frames.world_pin[1], 0.0)
        self.assertAlmostEqual(frames.world_pin[2], 0.61)
        self.assertTrue(
            IsaacPhysicsHarvestBridge._should_report_detached(
                grasp_mode="physics",
                detach_intent_active=True,
                stem_break_observed=True,
                stem_distance_m=0.0,
            )
        )

    def test_success_mode_keeps_distance_compatibility(self) -> None:
        self.assertTrue(
            IsaacPhysicsHarvestBridge._should_report_detached(
                grasp_mode="success",
                detach_intent_active=True,
                stem_break_observed=False,
                stem_distance_m=IsaacPhysicsHarvestBridge.DETACH_DISTANCE_M,
            )
        )

    def test_success_mode_rejects_distance_detach_before_detaching_intent(self) -> None:
        self.assertFalse(
            IsaacPhysicsHarvestBridge._should_report_detached(
                grasp_mode="success",
                detach_intent_active=False,
                stem_break_observed=False,
                stem_distance_m=IsaacPhysicsHarvestBridge.DETACH_DISTANCE_M,
            )
        )

    def test_physics_mode_rejects_joint_break_before_detaching_intent(self) -> None:
        self.assertFalse(
            IsaacPhysicsHarvestBridge._should_report_detached(
                grasp_mode="physics",
                detach_intent_active=False,
                stem_break_observed=True,
                stem_distance_m=0.0,
            )
        )

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

    def test_geometry_fallback_can_create_contacts_without_recent_physical_contact_when_geometry_is_tight(self) -> None:
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

        self.assertEqual(bridge._latched_finger_contacts, {"left", "right"})

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

    def test_attached_tomato_pose_is_restored_when_physics_pose_runs_away(self) -> None:
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

        class _Snapshot:
            tomato_status = TomatoStatus.ATTACHED
            tomato_pose = Pose3D(0.62, 0.0, 0.54, 0.0, 0.0, 0.0)

        self.assertTrue(
            bridge._should_restore_attached_tomato_pose(
                snapshot=_Snapshot(),
                tomato_pose=Pose3D(4.70, 0.12, -3900.0, 0.0, 0.0, 0.0),
            )
        )


if __name__ == "__main__":
    unittest.main()
