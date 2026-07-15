from __future__ import annotations

import os
import time
from dataclasses import dataclass

from tomato_harvest_sim.msg.contracts import Pose3D, TomatoStatus
from tomato_harvest_sim.simulator.physics_tuning import apply_physics_tuning
from tomato_harvest_sim.simulator.scene_config import (
    PhysicsTuningConfig,
    load_physics_tuning_config,
)
from tomato_harvest_sim.simulator.physics_observation import (
    contact_forces_from_impulses,
    FingerContactImpulses,
    estimate_stem_tension_n,
    format_observation_line,
    summarize_finger_contact_impulses,
)
from tomato_harvest_sim.simulator.grasp_strategy import FrictionGraspConfig, FrictionGraspStrategy, GraspDecision


@dataclass(frozen=True)
class PhysicsHarvestScenePaths:
    ground_prim_path: str
    tray_prim_path: str
    tomato_prim_path: str
    stem_anchor_prim_path: str
    stem_joint_prim_path: str
    grasp_joint_prim_path: str
    hand_mount_prim_path: str


class IsaacPhysicsHarvestBridge:
    STEM_BREAK_FORCE_N = 50.0
    STEM_BREAK_TORQUE_NM = 50.0
    TOMATO_MASS_KG = 0.03
    DETACH_DISTANCE_M = 0.02
    # place_pose はトレイ中心の 0.12m 上空（pregrasp_planner の place_vertical_offset_m）
    # で、behavior_planner の到達許容が 0.05m あるため、正常リリース時の
    # tool-tray 距離は最大 0.17m になる。0.06 だと正常な設置でも必ず FALLEN 判定
    # になるため（run14 で実測）、リリース高さ + 到達許容 + 余裕で 0.18 とする。
    PLACE_DISTANCE_M = 0.18
    CONTACT_LATCH_GRACE_STEPS = 3
    HAND_TO_TOMATO_DISTANCE_TOLERANCE_M = 0.12
    MAX_ATTACHED_TOMATO_DEVIATION_M = 0.15
    FINGER_MIDPOINT_TO_TOMATO_TOLERANCE_M = 0.020  # MoveIt EE frame vs panda_hand prim 間の ~1.43cm X オフセットを吸収
    FINGER_CONTACT_POINT_OFFSET_Z_M = 0.0447
    FINGER_GAP_MIN_M = 0.015
    FINGER_GAP_MAX_M = 0.065
    TOMATO_COLLISION_PRIM_SUFFIX = "/Geometry"
    # 観測ログ専用の物理ステップ幅の仮定値。判定には使用しない。
    OBSERVATION_PHYSICS_DT_SEC = 1.0 / 60.0

    def __init__(
        self,
        *,
        stage: object,
        scene_paths: PhysicsHarvestScenePaths,
        initial_tomato_pose: Pose3D,
        physics_tuning: PhysicsTuningConfig | None = None,
    ) -> None:
        self._stage = stage
        self._scene_paths = scene_paths
        self._initial_tomato_pose = initial_tomato_pose
        self._physics_tuning = physics_tuning or load_physics_tuning_config()
        self._last_cycle_id = 0
        self._pending_finger_contacts: set[str] = set()
        self._active_finger_contacts: set[str] = set()
        self._latched_finger_contacts: set[str] = set()
        self._recent_finger_contacts: set[str] = set()
        self._recent_contact_grace_steps_remaining = 0
        self._grasp_joint_active = False
        self._detach_reported = False
        self._contact_subscription = None
        self._pending_contact_impulses = FingerContactImpulses(left_ns=0.0, right_ns=0.0)
        self._physics_sequence_id = 0
        self._previous_tomato_velocity = (0.0, 0.0, 0.0)
        self._debug_enabled = os.environ.get(
            "TOMATO_HARVEST_DEBUG_PHYSICS_GRASP",
            "",
        ).strip() not in {"", "0", "false", "False"}
        self._grasp_mode = "success"
        self._friction_strategy = FrictionGraspStrategy(FrictionGraspConfig(
            self._physics_tuning.friction_grasp_required_steps,
            self._physics_tuning.friction_grasp_minimum_force_n,
            self._physics_tuning.friction_grasp_maximum_relative_speed_m_s,
            self._physics_tuning.friction_grasp_maximum_slip_m,
        ))

    def set_grasp_mode(self, grasp_mode: str) -> None:
        if grasp_mode not in {"success", "physics"}:
            raise ValueError(f"unsupported physics grasp mode: {grasp_mode}")
        self._grasp_mode = grasp_mode

    def prepare_scene(self) -> None:
        self._enable_static_collision(self._scene_paths.ground_prim_path)
        self._enable_static_collision(f"{self._scene_paths.tray_prim_path}/Base")
        self._enable_static_collision(f"{self._scene_paths.tray_prim_path}/WallFront")
        self._enable_static_collision(f"{self._scene_paths.tray_prim_path}/WallBack")
        self._enable_static_collision(f"{self._scene_paths.tray_prim_path}/WallLeft")
        self._enable_static_collision(f"{self._scene_paths.tray_prim_path}/WallRight")
        self._define_stem_anchor()
        self._define_tomato_physics()
        self._apply_physics_tuning()
        self._create_stem_joint()
        self._subscribe_contact_reports()

    def _apply_physics_tuning(self) -> None:
        """scene.yaml の physics セクションを適用する（enabled=False なら無適用）。"""
        applied = apply_physics_tuning(
            stage=self._stage,
            config=self._physics_tuning,
            tomato_prim_path=self._scene_paths.tomato_prim_path,
            tomato_collision_prim_path=self._tomato_collision_prim_path(),
            finger_link_prim_paths=(
                self._left_finger_prim_path(),
                self._right_finger_prim_path(),
            ),
            container_prim_paths=(
                self._scene_paths.ground_prim_path,
                f"{self._scene_paths.tray_prim_path}/Base",
                f"{self._scene_paths.tray_prim_path}/WallFront",
                f"{self._scene_paths.tray_prim_path}/WallBack",
                f"{self._scene_paths.tray_prim_path}/WallLeft",
                f"{self._scene_paths.tray_prim_path}/WallRight",
            ),
        )
        # A/B 比較の証跡としてデバッグ無効時でも適用状態を1行残す
        print(
            f"[PhysicsTuning] enabled={self._physics_tuning.enabled} "
            f"applied_items={len(applied)}",
            flush=True,
        )
        for line in applied:
            self._debug_log(f"[PhysicsTuning] {line}")

    def begin_physics_step(self) -> None:
        self._physics_sequence_id += 1
        self._pending_finger_contacts = set()
        self._pending_contact_impulses = FingerContactImpulses(left_ns=0.0, right_ns=0.0)

    def finalize_physics_step(self, controller: object) -> None:
        snapshot = controller.current_scene_snapshot()
        forces = contact_forces_from_impulses(
            self._pending_contact_impulses, dt_sec=self.OBSERVATION_PHYSICS_DT_SEC
        )
        self._promote_pending_contacts(gripper_closed=snapshot.gripper_closed)
        controller.sync_grasp_diagnostics(
            left_contact="left" in self._active_finger_contacts,
            right_contact="right" in self._active_finger_contacts,
            left_force_n=forces.left_n,
            right_force_n=forces.right_n,
        )
        self._debug_log(
            "[PhysicsHarvest] finalize "
            f"phase={snapshot.phase.value} "
            f"tomato_status={snapshot.tomato_status.value} "
            f"gripper_closed={snapshot.gripper_closed} "
            f"contacts={sorted(self._active_finger_contacts)} "
            f"latched_contacts={sorted(self._latched_finger_contacts)} "
            f"recent_contacts={sorted(self._recent_finger_contacts)} "
            f"grace_steps={self._recent_contact_grace_steps_remaining} "
            f"grasp_joint_active={self._grasp_joint_active}"
        )

        if snapshot.cycle_id != self._last_cycle_id:
            self._last_cycle_id = snapshot.cycle_id
            if snapshot.phase.value == "ready":
                self.reset_scene(controller)
                return

        tomato_pose = self._world_pose(self._scene_paths.tomato_prim_path)
        if self._grasp_mode == "success" and self._should_restore_attached_tomato_pose(snapshot=snapshot, tomato_pose=tomato_pose):
            self._debug_log("[PhysicsHarvest] restoring unstable attached tomato pose before grasp evaluation.")
            tomato_pose = snapshot.tomato_pose
            self._set_world_pose(self._scene_paths.tomato_prim_path, tomato_pose)
            self._zero_rigid_body_velocity(self._scene_paths.tomato_prim_path)
        controller.sync_tomato_physics(tomato_pose)
        self._log_observation(snapshot=snapshot, tomato_pose=tomato_pose)
        if self._grasp_mode == "success":
            self._augment_contacts_from_grasp_geometry(tomato_pose=tomato_pose, gripper_closed=snapshot.gripper_closed)
        else:
            self._finalize_friction_grasp(controller, snapshot, tomato_pose)
            return

        if (
            snapshot.gripper_closed
            and not self._grasp_joint_active
            and self._latched_finger_contacts == {"left", "right"}
        ):
            self._debug_log("[PhysicsHarvest] both finger contacts detected. Creating grasp joint.")
            self._create_grasp_joint(tomato_pose)
            controller.sync_tomato_physics(
                tomato_pose,
                attached=True,
                status=TomatoStatus.HELD,
                reason="stable_grasp_established_physx",
            )
            return

        if self._grasp_joint_active and not self._detach_reported:
            if self._distance(self._world_pose(self._scene_paths.stem_anchor_prim_path), tomato_pose) >= self.DETACH_DISTANCE_M:
                self._debug_log("[PhysicsHarvest] detach distance reached. Reporting DETACHED.")
                self._detach_reported = True
                controller.sync_tomato_physics(
                    tomato_pose,
                    attached=False,
                    status=TomatoStatus.DETACHED,
                    reason="tomato_detached_from_stem_physx",
                )
                return

        if self._grasp_joint_active and not snapshot.gripper_closed:
            self._debug_log("[PhysicsHarvest] gripper opened while grasp joint active. Removing grasp joint.")
            self._remove_grasp_joint()
            if self._distance(snapshot.robot_tool_pose, snapshot.tray_pose) <= self.PLACE_DISTANCE_M:
                controller.sync_tomato_physics(
                    tomato_pose,
                    attached=False,
                    status=TomatoStatus.PLACED,
                    reason="tomato_placed_in_tray_physx",
                )
            else:
                controller.sync_tomato_physics(
                    tomato_pose,
                    attached=False,
                    status=TomatoStatus.FALLEN,
                    reason="released_outside_place_target_physx",
                )

    def _finalize_friction_grasp(self, controller: object, snapshot: object, tomato_pose: Pose3D) -> None:
        """人工拘束を使わず、接触観測だけでHELD・滑落・releaseを同期する。"""
        decision = self._friction_strategy.observe(
            bool(snapshot.gripper_closed),
            self._pending_contact_impulses.left_ns / self.OBSERVATION_PHYSICS_DT_SEC,
            self._pending_contact_impulses.right_ns / self.OBSERVATION_PHYSICS_DT_SEC,
            self._world_pose(self._scene_paths.hand_mount_prim_path),
            tomato_pose,
            self.OBSERVATION_PHYSICS_DT_SEC,
        )
        if decision is GraspDecision.HELD:
            controller.sync_tomato_physics(tomato_pose, attached=True, status=TomatoStatus.HELD,
                                           reason="friction_grasp_observed")
        elif decision is GraspDecision.LOST:
            controller.sync_tomato_physics(tomato_pose, attached=False, status=TomatoStatus.FALLEN,
                                           reason="friction_grasp_slipped")
        elif decision is GraspDecision.RELEASED:
            status = TomatoStatus.PLACED if self._distance(snapshot.robot_tool_pose, snapshot.tray_pose) <= self.PLACE_DISTANCE_M else TomatoStatus.FALLEN
            controller.sync_tomato_physics(tomato_pose, attached=False, status=status,
                                           reason="friction_grasp_released")
        if snapshot.tomato_status is TomatoStatus.HELD and not self._detach_reported:
            stem_distance = self._distance(self._world_pose(self._scene_paths.stem_anchor_prim_path), tomato_pose)
            if stem_distance >= self.DETACH_DISTANCE_M:
                self._detach_reported = True
                controller.sync_tomato_physics(tomato_pose, attached=False, status=TomatoStatus.DETACHED,
                                               reason="tomato_detached_from_stem_friction")

    def reset_scene(self, controller: object) -> None:
        self._active_finger_contacts = set()
        self._pending_finger_contacts = set()
        self._latched_finger_contacts = set()
        self._recent_finger_contacts = set()
        self._recent_contact_grace_steps_remaining = 0
        self._detach_reported = False
        self._friction_strategy.reset()
        self._remove_grasp_joint()
        self._set_world_pose(self._scene_paths.stem_anchor_prim_path, self._initial_tomato_pose)
        self._set_world_pose(self._scene_paths.tomato_prim_path, self._initial_tomato_pose)
        self._zero_rigid_body_velocity(self._scene_paths.tomato_prim_path)
        self._create_stem_joint()
        controller.sync_tomato_physics(
            self._initial_tomato_pose,
            attached=True,
            status=TomatoStatus.ATTACHED,
        )

    def _subscribe_contact_reports(self) -> None:
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysxSchema

        # PhysX の接触レポートは剛体アクター単位のため、collider 子 prim ではなく
        # RigidBodyAPI を持つルート prim に適用する（子 prim 適用ではイベントが
        # 一切発生しないことを Step 0 ベースライン run1 で確認済み）。
        tomato_prim = self._stage.GetPrimAtPath(self._scene_paths.tomato_prim_path)
        contact_api = PhysxSchema.PhysxContactReportAPI.Apply(tomato_prim)
        contact_api.CreateThresholdAttr().Set(0.0)
        self._contact_subscription = get_physx_simulation_interface().subscribe_contact_report_events(
            self._on_contact_report_event
        )

    def _on_contact_report_event(self, contact_headers: object, contact_data: object) -> None:
        from pxr import PhysicsSchemaTools

        active_contacts: set[str] = set()
        for header in contact_headers:
            actor0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
            actor1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            finger_name = self._match_finger_contact(actor0, actor1)
            self._debug_log(
                "[PhysicsHarvest] contact "
                f"actor0={actor0} actor1={actor1} matched={finger_name}"
            )
            if finger_name is not None:
                active_contacts.add(finger_name)
        self._accumulate_pending_contacts(active_contacts)
        self._accumulate_contact_impulses(contact_headers, contact_data)

    def _accumulate_contact_impulses(self, contact_headers: object, contact_data: object) -> None:
        """観測用に finger 別接触力積を集計する（判定へは介入しない）。"""
        from pxr import PhysicsSchemaTools

        def finger_of_pair(actor0: int, actor1: int) -> str | None:
            return self._match_finger_contact(
                str(PhysicsSchemaTools.intToSdfPath(actor0)),
                str(PhysicsSchemaTools.intToSdfPath(actor1)),
            )

        impulses = summarize_finger_contact_impulses(
            contact_headers, contact_data, finger_of_pair=finger_of_pair
        )
        self._pending_contact_impulses = self._pending_contact_impulses.merged_with(impulses)

    def _accumulate_pending_contacts(self, contacts: set[str]) -> None:
        self._pending_finger_contacts.update(contacts)

    def _promote_pending_contacts(self, *, gripper_closed: bool) -> None:
        self._active_finger_contacts = set(self._pending_finger_contacts)
        if self._active_finger_contacts:
            self._recent_finger_contacts = set(self._active_finger_contacts)
            self._recent_contact_grace_steps_remaining = self.CONTACT_LATCH_GRACE_STEPS
        elif self._recent_contact_grace_steps_remaining > 0:
            self._recent_contact_grace_steps_remaining -= 1
            if self._recent_contact_grace_steps_remaining == 0:
                self._recent_finger_contacts = set()
        if gripper_closed:
            contacts_to_latch = (
                self._active_finger_contacts
                if self._active_finger_contacts
                else self._recent_finger_contacts
            )
            self._latched_finger_contacts.update(contacts_to_latch)
            return
        self._latched_finger_contacts = set()

    def _log_observation(self, *, snapshot: object, tomato_pose: Pose3D) -> None:
        """1 ステップ分の物理観測値を機械可読形式でログ出力する。

        Step 0 の観測基盤。TOMATO_HARVEST_DEBUG_PHYSICS_GRASP 有効時のみ動作し、
        物理判定・シーン状態には一切影響しない読み取り専用処理。
        """
        if not self._debug_enabled:
            return
        velocity = self._rigid_body_velocity(self._scene_paths.tomato_prim_path)
        speed = (velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2) ** 0.5
        hand_pose = self._world_pose(self._scene_paths.hand_mount_prim_path)
        stem_pose = self._world_pose(self._scene_paths.stem_anchor_prim_path)
        finger_gap = self._distance(
            self._world_pose(self._left_finger_prim_path()),
            self._world_pose(self._right_finger_prim_path()),
        )
        stem_tension = estimate_stem_tension_n(
            mass_kg=self.TOMATO_MASS_KG,
            velocity_m_s=velocity,
            previous_velocity_m_s=self._previous_tomato_velocity,
            dt_sec=self.OBSERVATION_PHYSICS_DT_SEC,
        )
        self._previous_tomato_velocity = velocity
        print(
            format_observation_line(
                sequence_id=self._physics_sequence_id,
                timestamp_sec=time.monotonic(),
                tomato_status=snapshot.tomato_status.value,
                gripper_closed=bool(snapshot.gripper_closed),
                grasp_joint_active=self._grasp_joint_active,
                impulses=self._pending_contact_impulses,
                forces=contact_forces_from_impulses(
                    self._pending_contact_impulses,
                    dt_sec=self.OBSERVATION_PHYSICS_DT_SEC,
                ),
                tomato_speed_m_s=speed,
                hand_distance_m=self._distance(hand_pose, tomato_pose),
                stem_distance_m=self._distance(stem_pose, tomato_pose),
                stem_tension_n=stem_tension,
                finger_gap_m=finger_gap,
            ),
            flush=True,
        )

    def _rigid_body_velocity(self, prim_path: str) -> tuple[float, float, float]:
        """剛体の線速度を読み取る。属性が未生成の間は零ベクトルを返す。"""
        prim = self._stage.GetPrimAtPath(prim_path)
        attr = prim.GetAttribute("physics:velocity")
        if not attr.IsValid():
            return (0.0, 0.0, 0.0)
        value = attr.Get()
        if value is None:
            return (0.0, 0.0, 0.0)
        return (float(value[0]), float(value[1]), float(value[2]))

    def _match_finger_contact(self, actor0: str, actor1: str) -> str | None:
        pair = (actor0, actor1)
        if not any(self._is_tomato_actor(actor_path) for actor_path in pair):
            return None
        other_actor = actor1 if self._is_tomato_actor(actor0) else actor0
        if "panda_leftfinger" in other_actor:
            return "left"
        if "panda_rightfinger" in other_actor:
            return "right"
        return None

    def _augment_contacts_from_grasp_geometry(self, *, tomato_pose: Pose3D, gripper_closed: bool) -> None:
        if not gripper_closed:
            return
        if self._latched_finger_contacts == {"left", "right"}:
            return
        geometric_contacts = self._infer_finger_contacts_from_geometry(tomato_pose)
        if not geometric_contacts:
            return
        if not (self._active_finger_contacts or self._recent_finger_contacts):
            self._debug_log(
                "[PhysicsHarvest] geometry fallback inferred contacts without a contact report "
                "because the grasp geometry was tightly aligned."
            )
        self._debug_log(
            "[PhysicsHarvest] geometry fallback inferred "
            f"contacts={sorted(geometric_contacts)}"
        )
        self._active_finger_contacts.update(geometric_contacts)
        self._recent_finger_contacts.update(geometric_contacts)
        self._recent_contact_grace_steps_remaining = self.CONTACT_LATCH_GRACE_STEPS
        self._latched_finger_contacts.update(geometric_contacts)

    def _infer_finger_contacts_from_geometry(self, tomato_pose: Pose3D) -> set[str]:
        hand_pose = self._world_pose(self._scene_paths.hand_mount_prim_path)
        hand_to_tomato_distance = self._distance(hand_pose, tomato_pose)
        if hand_to_tomato_distance > self.HAND_TO_TOMATO_DISTANCE_TOLERANCE_M:
            self._debug_log_geometry_state(
                hand_pose=hand_pose,
                tomato_pose=tomato_pose,
                left_finger_pose=None,
                right_finger_pose=None,
                hand_to_tomato_distance=hand_to_tomato_distance,
                left_distance=None,
                right_distance=None,
                midpoint_distance=None,
            )
            return set()
        left_finger_pose = self._world_pose(self._left_finger_prim_path())
        right_finger_pose = self._world_pose(self._right_finger_prim_path())
        left_contact_pose = self._inferred_finger_contact_pose(left_finger_pose)
        right_contact_pose = self._inferred_finger_contact_pose(right_finger_pose)
        left_distance = self._distance(left_contact_pose, tomato_pose)
        right_distance = self._distance(right_contact_pose, tomato_pose)
        finger_gap = self._distance(left_contact_pose, right_contact_pose)
        if (
            finger_gap < self.FINGER_GAP_MIN_M
            or finger_gap > self.FINGER_GAP_MAX_M
        ):
            midpoint = Pose3D(
                x=(left_contact_pose.x + right_contact_pose.x) * 0.5,
                y=(left_contact_pose.y + right_contact_pose.y) * 0.5,
                z=(left_contact_pose.z + right_contact_pose.z) * 0.5,
                roll=0.0,
                pitch=0.0,
                yaw=0.0,
            )
            midpoint_distance = self._distance(midpoint, tomato_pose)
            self._debug_log_geometry_state(
                hand_pose=hand_pose,
                tomato_pose=tomato_pose,
                left_finger_pose=left_finger_pose,
                right_finger_pose=right_finger_pose,
                hand_to_tomato_distance=hand_to_tomato_distance,
                left_distance=left_distance,
                right_distance=right_distance,
                midpoint_distance=midpoint_distance,
            )
            return set()

        midpoint = Pose3D(
            x=(left_contact_pose.x + right_contact_pose.x) * 0.5,
            y=(left_contact_pose.y + right_contact_pose.y) * 0.5,
            z=(left_contact_pose.z + right_contact_pose.z) * 0.5,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )
        midpoint_distance = self._distance(midpoint, tomato_pose)
        if midpoint_distance > self.FINGER_MIDPOINT_TO_TOMATO_TOLERANCE_M:
            self._debug_log_geometry_state(
                hand_pose=hand_pose,
                tomato_pose=tomato_pose,
                left_finger_pose=left_contact_pose,
                right_finger_pose=right_contact_pose,
                hand_to_tomato_distance=hand_to_tomato_distance,
                left_distance=left_distance,
                right_distance=right_distance,
                midpoint_distance=midpoint_distance,
            )
            return set()
        return {"left", "right"}

    def _should_restore_attached_tomato_pose(self, *, snapshot: object, tomato_pose: Pose3D) -> bool:
        if getattr(snapshot, "tomato_status", None) != TomatoStatus.ATTACHED:
            return False
        if self._grasp_joint_active:
            return False
        reference_pose = getattr(snapshot, "tomato_pose", None)
        if reference_pose is None:
            return False
        return self._distance(reference_pose, tomato_pose) > self.MAX_ATTACHED_TOMATO_DEVIATION_M

    def _inferred_finger_contact_pose(self, finger_pose: Pose3D) -> Pose3D:
        # The finger prim pose is near the finger root, while the actual grasp contact
        # happens near the pad/tip lower along the approach axis in this top-down POC.
        return Pose3D(
            x=finger_pose.x,
            y=finger_pose.y,
            z=finger_pose.z - self.FINGER_CONTACT_POINT_OFFSET_Z_M,
            roll=finger_pose.roll,
            pitch=finger_pose.pitch,
            yaw=finger_pose.yaw,
        )

    def _debug_log_geometry_state(
        self,
        *,
        hand_pose: Pose3D,
        tomato_pose: Pose3D,
        left_finger_pose: Pose3D | None,
        right_finger_pose: Pose3D | None,
        hand_to_tomato_distance: float | None,
        left_distance: float | None,
        right_distance: float | None,
        midpoint_distance: float | None,
    ) -> None:
        if not self._debug_enabled:
            return
        finger_gap = None
        if left_finger_pose is not None and right_finger_pose is not None:
            finger_gap = self._distance(left_finger_pose, right_finger_pose)
        self._debug_log(
            "[PhysicsHarvest] geometry check "
            f"hand_xyz=({hand_pose.x:.4f}, {hand_pose.y:.4f}, {hand_pose.z:.4f}) "
            f"tomato_xyz=({tomato_pose.x:.4f}, {tomato_pose.y:.4f}, {tomato_pose.z:.4f}) "
            f"left_xyz={self._format_pose(left_finger_pose)} "
            f"right_xyz={self._format_pose(right_finger_pose)} "
            f"hand_to_tomato={self._format_distance(hand_to_tomato_distance)} "
            f"left_to_tomato={self._format_distance(left_distance)} "
            f"right_to_tomato={self._format_distance(right_distance)} "
            f"midpoint_to_tomato={self._format_distance(midpoint_distance)} "
            f"finger_gap={self._format_distance(finger_gap)}"
        )

    @staticmethod
    def _format_pose(pose: Pose3D | None) -> str:
        if pose is None:
            return "n/a"
        return f"({pose.x:.4f}, {pose.y:.4f}, {pose.z:.4f})"

    @staticmethod
    def _format_distance(distance_m: float | None) -> str:
        if distance_m is None:
            return "n/a"
        return f"{distance_m:.4f}"

    def _left_finger_prim_path(self) -> str:
        return self._scene_paths.hand_mount_prim_path.replace("panda_hand", "panda_leftfinger")

    def _right_finger_prim_path(self) -> str:
        return self._scene_paths.hand_mount_prim_path.replace("panda_hand", "panda_rightfinger")

    def _define_stem_anchor(self) -> None:
        from pxr import Gf, UsdGeom

        anchor = UsdGeom.Xform.Define(self._stage, self._scene_paths.stem_anchor_prim_path)
        anchor.AddTranslateOp().Set(
            Gf.Vec3d(self._initial_tomato_pose.x, self._initial_tomato_pose.y, self._initial_tomato_pose.z)
        )
        UsdGeom.Imageable(anchor).MakeInvisible()

    def _define_tomato_physics(self) -> None:
        from pxr import UsdPhysics

        tomato_prim = self._stage.GetPrimAtPath(self._scene_paths.tomato_prim_path)
        UsdPhysics.RigidBodyAPI.Apply(tomato_prim)
        mass_api = UsdPhysics.MassAPI.Apply(tomato_prim)
        mass_api.CreateMassAttr(self.TOMATO_MASS_KG)
        collision_prim = self._stage.GetPrimAtPath(self._tomato_collision_prim_path())
        if collision_prim.IsValid():
            UsdPhysics.CollisionAPI.Apply(collision_prim)

    def _create_stem_joint(self) -> None:
        from pxr import Gf, Sdf, UsdPhysics

        self._remove_joint(self._scene_paths.stem_joint_prim_path)
        joint = UsdPhysics.FixedJoint.Define(self._stage, self._scene_paths.stem_joint_prim_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(self._scene_paths.tomato_prim_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr().Set(
            Gf.Vec3f(
                float(self._initial_tomato_pose.x),
                float(self._initial_tomato_pose.y),
                float(self._initial_tomato_pose.z),
            )
        )
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateBreakForceAttr(self.STEM_BREAK_FORCE_N)
        joint.CreateBreakTorqueAttr(self.STEM_BREAK_TORQUE_NM)
        self._detach_reported = False

    def _create_grasp_joint(self, tomato_pose: Pose3D) -> None:
        from pxr import Gf, Gf as _Gf, Sdf, UsdPhysics

        self._remove_grasp_joint()
        joint = UsdPhysics.FixedJoint.Define(self._stage, self._scene_paths.grasp_joint_prim_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(self._scene_paths.hand_mount_prim_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(self._scene_paths.tomato_prim_path)])
        world_point = Gf.Vec3d(tomato_pose.x, tomato_pose.y, tomato_pose.z)
        hand_local = self._world_point_to_local(self._scene_paths.hand_mount_prim_path, world_point)
        tomato_local = self._world_point_to_local(self._scene_paths.tomato_prim_path, world_point)
        joint.CreateLocalPos0Attr().Set(_Gf.Vec3f(hand_local[0], hand_local[1], hand_local[2]))
        joint.CreateLocalPos1Attr().Set(_Gf.Vec3f(tomato_local[0], tomato_local[1], tomato_local[2]))
        self._grasp_joint_active = True

    def _remove_grasp_joint(self) -> None:
        self._remove_joint(self._scene_paths.grasp_joint_prim_path)
        self._grasp_joint_active = False

    def _remove_joint(self, prim_path: str) -> None:
        prim = self._stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            self._stage.RemovePrim(prim_path)

    def _debug_log(self, message: str) -> None:
        if self._debug_enabled:
            print(message, flush=True)

    def _enable_static_collision(self, prim_path: str) -> None:
        from pxr import UsdPhysics

        prim = self._stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            UsdPhysics.CollisionAPI.Apply(prim)

    def _set_world_pose(self, prim_path: str, pose: Pose3D) -> None:
        from pxr import Gf, UsdGeom

        prim = self._stage.GetPrimAtPath(prim_path)
        xformable = UsdGeom.Xformable(prim)
        translate_op = xformable.GetOrderedXformOps()[0]
        translate_op.Set(Gf.Vec3d(pose.x, pose.y, pose.z))

    def _zero_rigid_body_velocity(self, prim_path: str) -> None:
        from pxr import Gf

        prim = self._stage.GetPrimAtPath(prim_path)
        for attr_name in ("physics:velocity", "physics:angularVelocity"):
            attr = prim.GetAttribute(attr_name)
            if attr.IsValid():
                attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))

    def _world_pose(self, prim_path: str) -> Pose3D:
        from pxr import UsdGeom

        prim = self._stage.GetPrimAtPath(prim_path)
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.0)
        translation = matrix.ExtractTranslation()
        return Pose3D(
            x=float(translation[0]),
            y=float(translation[1]),
            z=float(translation[2]),
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )

    def _world_point_to_local(self, prim_path: str, point: object) -> tuple[float, float, float]:
        from pxr import Gf, UsdGeom

        prim = self._stage.GetPrimAtPath(prim_path)
        world_transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.0)
        local_point = world_transform.GetInverse().Transform(point)
        return float(local_point[0]), float(local_point[1]), float(local_point[2])

    def _tomato_collision_prim_path(self) -> str:
        return f"{self._scene_paths.tomato_prim_path}{self.TOMATO_COLLISION_PRIM_SUFFIX}"

    def _is_tomato_actor(self, actor_path: str) -> bool:
        tomato_root = self._scene_paths.tomato_prim_path
        return actor_path == tomato_root or actor_path.startswith(f"{tomato_root}/")

    @staticmethod
    def _distance(left: Pose3D, right: Pose3D) -> float:
        dx = left.x - right.x
        dy = left.y - right.y
        dz = left.z - right.z
        return (dx * dx + dy * dy + dz * dz) ** 0.5
