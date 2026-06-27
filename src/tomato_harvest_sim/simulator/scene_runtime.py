from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.api.contracts import (
    ControlCommand,
    JointTrajectory,
    MotionCommand,
    PhaseMotionPlan,
    Pose3D,
    ScenePhase,
    SceneSnapshot,
    TomatoStatus,
)
from tomato_harvest_sim.simulator.scene_config import load_scene_layout_config


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
    target_tool_pose: Pose3D | None
    pregrasp_pose: Pose3D | None
    grasp_pose: Pose3D | None
    pull_pose: Pose3D | None
    place_pose: Pose3D | None
    grasp_result_reason: str | None
    motion_waypoints: tuple[Pose3D, ...]
    active_waypoint_index: int | None
    motion_joint_trajectory: JointTrajectory | None
    active_phase_motion_plan: PhaseMotionPlan | None


class IsaacSceneRuntime:
    CAMERA_NAMES = {"fixed_camera", "hand_camera"}
    FALL_STEP_M = 0.04
    TOOL_STEP_M = 0.04
    MOTION_TARGET_TOLERANCE_M = 0.005
    GRASP_TOMATO_OFFSET_Z_M = 0.05
    GRASP_POSITION_TOLERANCE_M = 0.03
    STABLE_GRASP_TOLERANCE_M = 0.04
    PLACE_POSITION_TOLERANCE_M = 0.05
    PLACED_TOMATO_OFFSET_Z_M = 0.03

    def __init__(
        self,
        *,
        physics_grasp_enabled: bool = False,
        physics_soft_fallback_enabled: bool = False,
    ) -> None:
        self._physics_grasp_enabled = physics_grasp_enabled
        self._physics_soft_fallback_enabled = physics_soft_fallback_enabled
        self._layout = load_scene_layout_config()
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
        if self.state.target_tool_pose == self._layout.home_tool_pose:
            self.state.robot_home = self._tool_pose_distance_sq(self._layout.home_tool_pose) <= (
                self.GRASP_POSITION_TOLERANCE_M * self.GRASP_POSITION_TOLERANCE_M
            )
        return self.snapshot()

    def set_pregrasp_pose(
        self,
        pose: Pose3D,
        *,
        waypoint_poses: tuple[Pose3D, ...] = (),
        joint_trajectory: JointTrajectory | None = None,
        phase_motion_plan: PhaseMotionPlan | None = None,
    ) -> SceneSnapshot:
        self.state.pregrasp_pose = pose
        return self._set_motion_path(
            pose,
            waypoint_poses=waypoint_poses,
            joint_trajectory=joint_trajectory,
            phase_motion_plan=phase_motion_plan,
        )

    def set_grasp_pose(
        self,
        pose: Pose3D,
        *,
        waypoint_poses: tuple[Pose3D, ...] = (),
        joint_trajectory: JointTrajectory | None = None,
        phase_motion_plan: PhaseMotionPlan | None = None,
    ) -> SceneSnapshot:
        self.state.grasp_pose = pose
        return self._set_motion_path(
            pose,
            waypoint_poses=waypoint_poses,
            joint_trajectory=joint_trajectory,
            phase_motion_plan=phase_motion_plan,
        )

    def set_pull_pose(
        self,
        pose: Pose3D,
        *,
        waypoint_poses: tuple[Pose3D, ...] = (),
        joint_trajectory: JointTrajectory | None = None,
        phase_motion_plan: PhaseMotionPlan | None = None,
    ) -> SceneSnapshot:
        self.state.pull_pose = pose
        return self._set_motion_path(
            pose,
            waypoint_poses=waypoint_poses,
            joint_trajectory=joint_trajectory,
            phase_motion_plan=phase_motion_plan,
        )

    def set_place_pose(
        self,
        pose: Pose3D,
        *,
        waypoint_poses: tuple[Pose3D, ...] = (),
        joint_trajectory: JointTrajectory | None = None,
        phase_motion_plan: PhaseMotionPlan | None = None,
    ) -> SceneSnapshot:
        self.state.place_pose = pose
        return self._set_motion_path(
            pose,
            waypoint_poses=waypoint_poses,
            joint_trajectory=joint_trajectory,
            phase_motion_plan=phase_motion_plan,
        )

    def move_robot_home(
        self,
        is_home: bool = True,
        *,
        phase_motion_plan: PhaseMotionPlan | None = None,
    ) -> SceneSnapshot:
        if not is_home:
            self.state.robot_home = False
            return self.snapshot()
        return self._set_motion_path(
            self._layout.home_tool_pose,
            waypoint_poses=(self._layout.home_tool_pose,),
            phase_motion_plan=phase_motion_plan,
        )

    def close_gripper(self) -> SceneSnapshot:
        self.state.gripper_closed = True
        self._clear_motion_target()
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
        self._clear_motion_target()
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

    def apply_motion_command(self, command: MotionCommand) -> SceneSnapshot:
        if command.command_name == "move_to_pregrasp":
            return self.set_pregrasp_pose(
                command.target_pose,
                waypoint_poses=command.waypoint_poses,
                joint_trajectory=command.joint_trajectory,
                phase_motion_plan=command.phase_motion_plan,
            )
        if command.command_name == "move_to_grasp":
            return self.set_grasp_pose(
                command.target_pose,
                waypoint_poses=command.waypoint_poses,
                joint_trajectory=command.joint_trajectory,
                phase_motion_plan=command.phase_motion_plan,
            )
        if command.command_name == "pull_to_detach":
            snapshot = self.set_pull_pose(
                command.target_pose,
                waypoint_poses=command.waypoint_poses,
                joint_trajectory=command.joint_trajectory,
                phase_motion_plan=command.phase_motion_plan,
            )
            if self._physics_grasp_enabled:
                if self._physics_soft_fallback_enabled and self.state.tomato_status is TomatoStatus.HELD:
                    self.state.tomato_attached = False
                    self.state.tomato_status = TomatoStatus.DETACHED
                    self.state.grasp_result_reason = "tomato_detached_from_stem_soft_fallback"
                    return self.snapshot()
                self.state.grasp_result_reason = "awaiting_physics_detach"
                return snapshot
            if self.state.tomato_status is TomatoStatus.HELD:
                self.state.tomato_attached = False
                self.state.tomato_status = TomatoStatus.DETACHED
                self.state.grasp_result_reason = "tomato_detached_from_stem"
            return snapshot
        if command.command_name == "move_to_place":
            return self.set_place_pose(
                command.target_pose,
                waypoint_poses=command.waypoint_poses,
                joint_trajectory=command.joint_trajectory,
                phase_motion_plan=command.phase_motion_plan,
            )
        if command.command_name == "close_gripper":
            return self.close_gripper()
        if command.command_name == "open_gripper":
            return self.open_gripper()
        if command.command_name == "move_home":
            return self.move_robot_home(phase_motion_plan=command.phase_motion_plan)
        raise ValueError(f"Unsupported motion command: {command.command_name}")

    def advance(self) -> SceneSnapshot:
        if self.state.target_tool_pose is not None:
            if self._target_pose_reached(self.state.target_tool_pose):
                if self._advance_motion_waypoint_if_needed():
                    return self.snapshot()
                if self.state.target_tool_pose == self._layout.home_tool_pose:
                    self.state.robot_home = True
            else:
                self.state.robot_tool_pose = self._step_tool_toward_target(
                    self.state.robot_tool_pose,
                    self.state.target_tool_pose,
                )
            if (
                self._target_pose_reached(self.state.target_tool_pose)
                and self.state.target_tool_pose == self._layout.home_tool_pose
            ):
                self.state.robot_home = True

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
            target_tool_pose=self.state.target_tool_pose,
            pregrasp_pose=self.state.pregrasp_pose,
            grasp_pose=self.state.grasp_pose,
            pull_pose=self.state.pull_pose,
            place_pose=self.state.place_pose,
            grasp_result_reason=self.state.grasp_result_reason,
            motion_waypoints=self.state.motion_waypoints,
            active_waypoint_index=self.state.active_waypoint_index,
            motion_joint_trajectory=self.state.motion_joint_trajectory,
            active_phase_motion_plan=self.state.active_phase_motion_plan,
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

    def _target_pose_reached(self, pose: Pose3D | None) -> bool:
        if pose is None:
            return False
        return self._tool_pose_distance_sq(pose) <= (
            self.MOTION_TARGET_TOLERANCE_M * self.MOTION_TARGET_TOLERANCE_M
        )

    def _set_motion_path(
        self,
        final_pose: Pose3D,
        *,
        waypoint_poses: tuple[Pose3D, ...],
        joint_trajectory: JointTrajectory | None = None,
        phase_motion_plan: PhaseMotionPlan | None = None,
    ) -> SceneSnapshot:
        path = waypoint_poses or (final_pose,)
        active_index = self._first_unreached_waypoint_index(path)
        self.state.motion_waypoints = path
        self.state.active_waypoint_index = active_index
        self.state.target_tool_pose = path[active_index]
        self.state.motion_joint_trajectory = joint_trajectory
        self.state.active_phase_motion_plan = phase_motion_plan
        self.state.robot_home = False
        return self.snapshot()

    def _first_unreached_waypoint_index(self, waypoint_poses: tuple[Pose3D, ...]) -> int:
        for index, waypoint_pose in enumerate(waypoint_poses):
            if not self._target_pose_reached(waypoint_pose):
                return index
        return len(waypoint_poses) - 1

    def _advance_motion_waypoint_if_needed(self) -> bool:
        if self.state.active_waypoint_index is None:
            return False
        next_index = self.state.active_waypoint_index + 1
        if next_index >= len(self.state.motion_waypoints):
            return False
        self.state.active_waypoint_index = next_index
        self.state.target_tool_pose = self.state.motion_waypoints[next_index]
        return True

    def _is_place_release_pose(self) -> bool:
        if self.state.place_pose is None:
            return False
        dx = self.state.robot_tool_pose.x - self.state.place_pose.x
        dy = self.state.robot_tool_pose.y - self.state.place_pose.y
        dz = self.state.robot_tool_pose.z - self.state.place_pose.z
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

    def _clear_motion_target(self) -> None:
        self.state.target_tool_pose = None
        self.state.motion_waypoints = ()
        self.state.active_waypoint_index = None
        self.state.motion_joint_trajectory = None
        self.state.active_phase_motion_plan = None

    def _step_tool_toward_target(self, current_pose: Pose3D, target_pose: Pose3D) -> Pose3D:
        dx = target_pose.x - current_pose.x
        dy = target_pose.y - current_pose.y
        dz = target_pose.z - current_pose.z
        distance_sq = dx * dx + dy * dy + dz * dz
        if distance_sq == 0.0:
            return target_pose

        distance = distance_sq ** 0.5
        if distance <= self.TOOL_STEP_M:
            return target_pose

        scale = self.TOOL_STEP_M / distance
        return Pose3D(
            round(current_pose.x + dx * scale, 6),
            round(current_pose.y + dy * scale, 6),
            round(current_pose.z + dz * scale, 6),
            target_pose.roll,
            target_pose.pitch,
            target_pose.yaw,
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
            target_tool_pose=None,
            pregrasp_pose=None,
            grasp_pose=None,
            pull_pose=None,
            place_pose=None,
            grasp_result_reason=None,
            motion_waypoints=(),
            active_waypoint_index=None,
            motion_joint_trajectory=None,
            active_phase_motion_plan=None,
        )
