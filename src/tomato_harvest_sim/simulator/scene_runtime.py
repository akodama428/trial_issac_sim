from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.msg.contracts import (
    ControlCommand,
    PhaseMotionPlan,
    Pose3D,
    ScenePhase,
    SceneSnapshot,
    TomatoStatus,
)
from tomato_harvest_sim.simulator.scene_config import (
    load_placement_config,
    load_scene_layout_config,
)


@dataclass
class SceneRuntimeState:
    phase: ScenePhase
    active_camera: str
    tomato_attached: bool
    tomato_status: TomatoStatus
    gripper_closed: bool
    robot_home: bool
    cycle_id: int
    robot_model: str
    robot_base_pose: Pose3D
    fixed_camera_pose: Pose3D
    hand_camera_pose: Pose3D
    branch_pose: Pose3D
    stem_pose: Pose3D
    tomato_pose: Pose3D
    tray_pose: Pose3D
    robot_tool_pose: Pose3D
    grasp_result_reason: str | None
    active_phase_motion_plan: PhaseMotionPlan | None
    left_finger_contact: bool
    right_finger_contact: bool
    left_finger_force_n: float | None
    right_finger_force_n: float | None


class IsaacSceneRuntime:
    CAMERA_NAMES = {"fixed_camera", "hand_camera"}
    FALL_STEP_M = 0.04
    GRASP_TOMATO_OFFSET_Z_M = 0.05
    GRASP_POSITION_TOLERANCE_M = 0.03
    STABLE_GRASP_TOLERANCE_M = 0.04
    PLACE_POSITION_TOLERANCE_M = 0.05
    PLACED_TOMATO_OFFSET_Z_M = 0.03
    GRIPPER_CLOSED_THRESHOLD_RAD = 0.01

    def __init__(
        self,
        *,
        physics_grasp_enabled: bool = False,
        physics_soft_fallback_enabled: bool = False,
    ) -> None:
        self._physics_grasp_enabled = physics_grasp_enabled
        self._physics_soft_fallback_enabled = physics_soft_fallback_enabled
        self._layout = load_scene_layout_config()
        self._placement = load_placement_config()
        self._gripper_commanded_closed: bool | None = None
        self.state = self._initial_state()
        self._grasp_constraint_offset = (0.0, 0.0, -self.GRASP_TOMATO_OFFSET_Z_M)

    def boot(self) -> SceneSnapshot:
        self.state = self._initial_state()
        self.state.phase = ScenePhase.READY
        return self.snapshot()

    def apply_control(self, command: ControlCommand) -> SceneSnapshot:
        if command is ControlCommand.START:
            self.state.phase = ScenePhase.RUNNING
        elif command is ControlCommand.STOP:
            self.state.phase = ScenePhase.STOPPED
        elif command is ControlCommand.RESET:
            return self.reset_scene()
        else:
            raise ValueError(f"Unsupported control command: {command}")

        self.state.cycle_id += 1
        return self.snapshot()

    def set_active_camera(self, camera_name: str) -> SceneSnapshot:
        if camera_name not in self.CAMERA_NAMES:
            raise ValueError(f"Unsupported camera: {camera_name}")
        self.state.active_camera = camera_name
        return self.snapshot()

    def set_tomato_pose(self, pose: Pose3D) -> SceneSnapshot:
        self.state.tomato_pose = pose
        return self.snapshot()

    def detach_tomato(self) -> SceneSnapshot:
        self.state.tomato_attached = False
        self.state.tomato_status = TomatoStatus.DETACHED
        return self.snapshot()

    def sync_tomato_physics(
        self,
        pose: Pose3D,
        *,
        attached: bool | None = None,
        status: TomatoStatus | None = None,
        reason: str | None = None,
    ) -> SceneSnapshot:
        self.state.tomato_pose = pose
        if attached is not None:
            self.state.tomato_attached = attached
        if status is not None:
            self.state.tomato_status = status
        if reason is not None:
            self.state.grasp_result_reason = reason
        return self.snapshot()

    def set_robot_tool_pose(self, pose: Pose3D) -> SceneSnapshot:
        self.state.robot_tool_pose = pose
        self.state.robot_home = False
        return self.snapshot()

    def sync_robot_tool_pose(self, pose: Pose3D) -> SceneSnapshot:
        self.state.robot_tool_pose = pose
        home = self._layout.home_tool_pose
        dx = pose.x - home.x
        dy = pose.y - home.y
        dz = pose.z - home.z
        self.state.robot_home = (dx * dx + dy * dy + dz * dz) <= (
            self.GRASP_POSITION_TOLERANCE_M * self.GRASP_POSITION_TOLERANCE_M
        )
        return self.snapshot()

    def sync_grasp_diagnostics(self, *, left_contact: bool, right_contact: bool,
                               left_force_n: float | None, right_force_n: float | None) -> SceneSnapshot:
        self.state.left_finger_contact = left_contact
        self.state.right_finger_contact = right_contact
        self.state.left_finger_force_n = left_force_n
        self.state.right_finger_force_n = right_force_n
        return self.snapshot()

    def close_gripper(self) -> SceneSnapshot:
        self.state.gripper_closed = True
        if self._physics_grasp_enabled:
            if self._physics_soft_fallback_enabled:
                self.state.tomato_status = TomatoStatus.HELD
                self.state.grasp_result_reason = "stable_grasp_established_soft_fallback"
                self._grasp_constraint_offset = (
                    self.state.tomato_pose.x - self.state.robot_tool_pose.x,
                    self.state.tomato_pose.y - self.state.robot_tool_pose.y,
                    self.state.tomato_pose.z - self.state.robot_tool_pose.z,
                )
                return self.snapshot()
            self.state.grasp_result_reason = "awaiting_physics_grasp"
            return self.snapshot()
        if self._is_stable_grasp_pose():
            self.state.tomato_status = TomatoStatus.HELD
            self.state.grasp_result_reason = "stable_grasp_established"
            self._grasp_constraint_offset = (
                self.state.tomato_pose.x - self.state.robot_tool_pose.x,
                self.state.tomato_pose.y - self.state.robot_tool_pose.y,
                self.state.tomato_pose.z - self.state.robot_tool_pose.z,
            )
            return self.snapshot()

        self.state.tomato_attached = False
        self.state.tomato_status = TomatoStatus.FALLEN
        self.state.grasp_result_reason = "grasp_missed_tomato"
        return self.snapshot()

    def open_gripper(self) -> SceneSnapshot:
        self.state.gripper_closed = False
        if self._physics_grasp_enabled:
            if self._physics_soft_fallback_enabled and self.state.tomato_status is TomatoStatus.DETACHED:
                if self._is_place_release_pose():
                    self.state.tomato_status = TomatoStatus.PLACED
                    self.state.grasp_result_reason = "tomato_placed_in_tray_soft_fallback"
                    self.state.tomato_pose = self._placed_tomato_pose()
                    return self.snapshot()
                self.state.tomato_status = TomatoStatus.FALLEN
                self.state.grasp_result_reason = "released_outside_place_target_soft_fallback"
                return self.snapshot()
            self.state.grasp_result_reason = "awaiting_physics_release"
            return self.snapshot()
        if self.state.tomato_status is TomatoStatus.DETACHED:
            if self._is_place_release_pose():
                self.state.tomato_status = TomatoStatus.PLACED
                self.state.grasp_result_reason = "tomato_placed_in_tray"
                self.state.tomato_pose = self._placed_tomato_pose()
                return self.snapshot()
            self.state.tomato_status = TomatoStatus.FALLEN
            self.state.grasp_result_reason = "released_outside_place_target"
        return self.snapshot()

    def apply_gripper_command(self, closed: bool) -> SceneSnapshot:
        """gripper intentを受ける。physics modeでは実finger状態を上書きしない。"""
        if self._physics_grasp_enabled:
            self._gripper_commanded_closed = closed
            self.state.grasp_result_reason = (
                "gripper_close_commanded"
                if closed
                else "gripper_open_commanded"
            )
            return self.snapshot()
        return self.close_gripper() if closed else self.open_gripper()

    def apply_finger_positions(self, finger_left: float, finger_right: float) -> SceneSnapshot:
        was_closed = self.state.gripper_closed
        measured_gap = finger_left + finger_right
        is_closed = (
            (
                not self._physics_grasp_enabled
                and finger_left < self.GRIPPER_CLOSED_THRESHOLD_RAD
                and finger_right < self.GRIPPER_CLOSED_THRESHOLD_RAD
            )
            or (
                self._gripper_commanded_closed is True
                and measured_gap
                <= self._placement.gripper_open.measured_closed_gap_threshold_m
            )
        )
        is_open = (
            (
                not self._physics_grasp_enabled
                and not is_closed
            )
            or (
                self._gripper_commanded_closed is False
                and measured_gap
                >= self._placement.gripper_open.measured_gap_threshold_m
            )
        )
        if is_closed and not was_closed:
            return self.close_gripper()
        if is_open and was_closed:
            return self.open_gripper()
        return self.snapshot()

    def advance(self) -> SceneSnapshot:
        if self.state.tomato_status in {TomatoStatus.HELD, TomatoStatus.DETACHED} and self.state.gripper_closed:
            self.state.tomato_pose = Pose3D(
                self.state.robot_tool_pose.x + self._grasp_constraint_offset[0],
                self.state.robot_tool_pose.y + self._grasp_constraint_offset[1],
                self.state.robot_tool_pose.z + self._grasp_constraint_offset[2],
                0.0,
                0.0,
                0.0,
            )
            return self.snapshot()

        if self._physics_grasp_enabled:
            return self.snapshot()

        if self.state.tomato_status is TomatoStatus.FALLEN:
            next_z = max(0.01, round(self.state.tomato_pose.z - self.FALL_STEP_M, 6))
            self.state.tomato_pose = Pose3D(
                self.state.tomato_pose.x,
                self.state.tomato_pose.y,
                next_z,
                self.state.tomato_pose.roll,
                self.state.tomato_pose.pitch,
                self.state.tomato_pose.yaw,
            )
        return self.snapshot()

    def reset_scene(self) -> SceneSnapshot:
        next_cycle_id = self.state.cycle_id + 1
        self.state = self._initial_state()
        self.state.phase = ScenePhase.READY
        self.state.cycle_id = next_cycle_id
        return self.snapshot()

    def describe_scene(self) -> str:
        return (
            f"robot={self.state.robot_model} "
            f"active_camera={self.state.active_camera} "
            f"tomato_attached={self.state.tomato_attached} "
            f"tomato_status={self.state.tomato_status.value} "
            f"tomato_xyz=({self.state.tomato_pose.x:.2f},"
            f"{self.state.tomato_pose.y:.2f},"
            f"{self.state.tomato_pose.z:.2f}) "
            f"tray_xyz=({self.state.tray_pose.x:.2f},"
            f"{self.state.tray_pose.y:.2f},"
            f"{self.state.tray_pose.z:.2f})"
        )

    def snapshot(self) -> SceneSnapshot:
        return SceneSnapshot(
            phase=self.state.phase,
            active_camera=self.state.active_camera,
            tomato_attached=self.state.tomato_attached,
            tomato_status=self.state.tomato_status,
            gripper_closed=self.state.gripper_closed,
            robot_home=self.state.robot_home,
            cycle_id=self.state.cycle_id,
            robot_model=self.state.robot_model,
            robot_base_pose=self.state.robot_base_pose,
            fixed_camera_pose=self.state.fixed_camera_pose,
            hand_camera_pose=self.state.hand_camera_pose,
            branch_pose=self.state.branch_pose,
            stem_pose=self.state.stem_pose,
            tomato_pose=self.state.tomato_pose,
            tray_pose=self.state.tray_pose,
            robot_tool_pose=self.state.robot_tool_pose,
            target_tool_pose=Pose3D(
                self.state.tomato_pose.x, self.state.tomato_pose.y,
                self.state.tomato_pose.z + self.GRASP_TOMATO_OFFSET_Z_M,
                self.state.robot_tool_pose.roll, self.state.robot_tool_pose.pitch,
                self.state.robot_tool_pose.yaw,
            ),
            grasp_result_reason=self.state.grasp_result_reason,
            active_phase_motion_plan=None,
            left_finger_contact=self.state.left_finger_contact,
            right_finger_contact=self.state.right_finger_contact,
            left_finger_force_n=self.state.left_finger_force_n,
            right_finger_force_n=self.state.right_finger_force_n,
            gripper_commanded_closed=self._gripper_commanded_closed,
        )

    def _is_stable_grasp_pose(self) -> bool:
        expected_tool_pose = Pose3D(
            self.state.tomato_pose.x,
            self.state.tomato_pose.y,
            self.state.tomato_pose.z + self.GRASP_TOMATO_OFFSET_Z_M,
            self.state.robot_tool_pose.roll,
            self.state.robot_tool_pose.pitch,
            self.state.robot_tool_pose.yaw,
        )
        return self._tool_pose_distance_sq(expected_tool_pose) <= (
            self.STABLE_GRASP_TOLERANCE_M * self.STABLE_GRASP_TOLERANCE_M
        )

    def _tool_pose_distance_sq(self, pose: Pose3D) -> float:
        dx = self.state.robot_tool_pose.x - pose.x
        dy = self.state.robot_tool_pose.y - pose.y
        dz = self.state.robot_tool_pose.z - pose.z
        return dx * dx + dy * dy + dz * dz

    def _is_place_release_pose(self) -> bool:
        dx = self.state.robot_tool_pose.x - self.state.tray_pose.x
        dy = self.state.robot_tool_pose.y - self.state.tray_pose.y
        dz = self.state.robot_tool_pose.z - self.state.tray_pose.z
        distance_sq = dx * dx + dy * dy + dz * dz
        return distance_sq <= self.PLACE_POSITION_TOLERANCE_M * self.PLACE_POSITION_TOLERANCE_M

    def _placed_tomato_pose(self) -> Pose3D:
        return Pose3D(
            self.state.tray_pose.x,
            self.state.tray_pose.y,
            round(self.state.tray_pose.z + self.PLACED_TOMATO_OFFSET_Z_M, 6),
            0.0,
            0.0,
            0.0,
        )

    @staticmethod
    def _initial_state() -> SceneRuntimeState:
        layout = load_scene_layout_config()
        return SceneRuntimeState(
            phase=ScenePhase.BOOTING,
            active_camera="fixed_camera",
            tomato_attached=True,
            tomato_status=TomatoStatus.ATTACHED,
            gripper_closed=False,
            robot_home=True,
            cycle_id=0,
            robot_model="Franka Panda",
            robot_base_pose=layout.robot_base_pose,
            fixed_camera_pose=layout.fixed_camera_pose,
            hand_camera_pose=layout.hand_camera_pose,
            branch_pose=layout.branch_pose,
            stem_pose=layout.stem_pose,
            tomato_pose=layout.tomato_pose,
            tray_pose=layout.tray_pose,
            robot_tool_pose=layout.home_tool_pose,
            grasp_result_reason=None,
            active_phase_motion_plan=None,
            left_finger_contact=False,
            right_finger_contact=False,
            left_finger_force_n=None,
            right_finger_force_n=None,
        )
