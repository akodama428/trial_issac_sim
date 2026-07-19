from __future__ import annotations

import os
import time

from tomato_harvest_sim.msg.contracts import (
    JointStateSnapshot,
    JointTrajectory,
    Pose3D,
    SceneSnapshot,
)
from tomato_harvest_sim.msg.topics import DEFAULT_JOINT_NAMES, home_joint_state
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.client import (
    MotionPlanOutcome,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.config import (
    MoveItPlannerConfig,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.geometry import (
    moveit_link_target_pose_from_runtime_tool_pose,
    quaternion_from_pose,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.phase_policy import (
    PlanningTarget,
    arm_joint_goal_from_ik_solution,
    goal_joint_window,
    ik_goal_is_near_seed,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.planning_scene import (
    PlanningSceneManager,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.request_builder import (
    build_joint_goal_request,
    build_pose_goal_request,
    new_motion_plan_request,
)
from tomato_harvest_sim.robot.motion_planner.moveit_bridge.trajectory import (
    concatenate_trajectories,
    joint_state_from_trajectory,
    trajectory_is_noop,
)
from tomato_harvest_sim.robot.motion_planner.planning_diagnostics import (
    PlanningFailureDiagnostic,
    diagnostics_directory,
    save_planning_failure_diagnostic,
)


class MoveItGoalPlanner:
    """Plan and combine targets after phase policy has selected them."""

    MOVEIT_LINK_TO_RUNTIME_TOOL_OFFSET_M = (0.0, 0.0, 0.0584)
    TRAY_INNER_SIZE_M = (0.22, 0.16, 0.05)
    TRAY_WALL_THICKNESS_M = 0.012
    TRAY_COLLISION_MARGIN_M = 0.015
    BRANCH_SIZE_M = (0.18, 0.02, 0.02)
    STEM_SIZE_M = (0.008, 0.008, 0.06)
    ATTACHED_TOMATO_RADIUS_M = 0.01
    ATTACHED_TOMATO_OFFSET_M = (0.0, 0.0, 0.1034)
    NOOP_TRAJECTORY_TOLERANCE_RAD = 1e-3
    JOINT_GOAL_TOLERANCE_RAD = 0.01

    def __init__(self, config: MoveItPlannerConfig) -> None:
        self._config = config
        # Keep these attributes during the compatibility transition.
        self._group_name = config.group_name
        self._end_effector_link = config.end_effector_link
        self._planning_timeout_sec = config.planning_timeout_sec
        self._goal_joint1_window_rad = config.goal_joint_window_rad
        self._seeded_ik_goal_enabled = config.seeded_ik_goal_enabled
        self._enforce_orientation_constraint = (
            config.enforce_orientation_constraint
        )
        self._debug_enabled = config.debug_enabled
        self._scene_manager = PlanningSceneManager(config)
        self._diagnostic_dir = diagnostics_directory(os.environ)

    def _plan_phase(
        self,
        *,
        clients: object,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        scene_snapshot: SceneSnapshot,
        planning_targets: tuple[PlanningTarget, ...],
        attach_tomato: bool,
        allow_gripper_target_contact: bool = False,
        phase_label: str = "",
        fallback_joint_goal: JointStateSnapshot | None = None,
    ) -> JointTrajectory | None:
        """Plan an ordered target sequence and concatenate its segments."""
        if not planning_targets:
            return None
        if not self._apply_phase_planning_scene(
            clients=clients,
            scene_snapshot=scene_snapshot,
            base_frame_id=base_frame_id,
            attach_tomato=attach_tomato,
            allow_gripper_target_contact=allow_gripper_target_contact,
        ):
            return None
        current = joint_state
        combined: JointTrajectory | None = None
        final_index = len(planning_targets) - 1
        for index, target in enumerate(planning_targets):
            segment = self._plan_target(
                clients=clients,
                joint_state=current,
                base_frame_id=base_frame_id,
                target=target,
                phase_label=phase_label,
                fallback_joint_goal=(
                    fallback_joint_goal if index == final_index else None
                ),
            )
            if segment is None:
                return None
            combined = (
                segment
                if combined is None
                else concatenate_trajectories(combined, segment)
            )
            current = joint_state_from_trajectory(segment)
        return combined

    def _plan_target(
        self,
        *,
        clients: object,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        target: PlanningTarget,
        phase_label: str,
        fallback_joint_goal: JointStateSnapshot | None,
    ) -> JointTrajectory | None:
        # 関節角指定のtargetは、MoveItのIK計算を経ずに直接planする。
        if isinstance(target, JointStateSnapshot):
            return self._plan_joint_goal(
                clients=clients,
                joint_state=joint_state,
                base_frame_id=base_frame_id,
                goal_joint_state=target,
                phase_label=phase_label,
            )
        return self._plan_pose_target(
            clients=clients,
            joint_state=joint_state,
            base_frame_id=base_frame_id,
            target_pose=target,
            phase_label=phase_label,
            fallback_joint_goal=fallback_joint_goal,
        )

    def _plan_pose_target(
        self,
        *,
        clients: object,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        target_pose: Pose3D,
        phase_label: str,
        fallback_joint_goal: JointStateSnapshot | None,
    ) -> JointTrajectory | None:
        # MoveItのIK計算をseeded_ik_goalで試み、失敗した場合はpose_goalで計画する。
        if self._seeded_ik_goal_enabled:
            trajectory = self._plan_seeded_ik_goal(
                clients=clients,
                joint_state=joint_state,
                base_frame_id=base_frame_id,
                target_pose=target_pose,
                phase_label=phase_label,
            )
            if trajectory is not None:
                return trajectory
        return self._plan_pose_goal_with_recovery(
            clients=clients,
            joint_state=joint_state,
            base_frame_id=base_frame_id,
            target_pose=target_pose,
            phase_label=phase_label,
            fallback_joint_goal=fallback_joint_goal,
        )

    def _plan_pose_goal_with_recovery(
        self,
        *,
        clients: object,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        target_pose: Pose3D,
        phase_label: str,
        fallback_joint_goal: JointStateSnapshot | None,
    ) -> JointTrajectory | None:
        # MoveItのpose_goal計画を、goal_joint_windowを使って再試行する。
        joint_window = goal_joint_window(
            joint_state, window_rad=self._goal_joint1_window_rad
        )
        outcome = self._plan_pose_goal(
            clients=clients,
            joint_state=joint_state,
            base_frame_id=base_frame_id,
            target_pose=target_pose,
            joint_window=joint_window,
        )
        # 関節角windowを使った計画が失敗した場合、windowを外して再試行する。
        if outcome.trajectory is None and joint_window is not None:
            print(
                f"[MoveItBridge] goal_joint_window exhausted "
                f"phase={phase_label} window_rad={joint_window[0][2]} "
                "— retrying without window",
                flush=True,
            )
            outcome = self._plan_pose_goal(
                clients=clients,
                joint_state=joint_state,
                base_frame_id=base_frame_id,
                target_pose=target_pose,
                joint_window=None,
            )
        if outcome.trajectory is not None:
            trajectory = outcome.trajectory
            self._debug_log(
                "[MoveItBridge] accepted trajectory "
                f"points={len(trajectory.points)} "
                f"ee_link={self._end_effector_link} "
                f"target_xyz=({target_pose.x:.4f}, {target_pose.y:.4f}, "
                f"{target_pose.z:.4f}) "
                f"end_q={trajectory.points[-1].positions_rad}"
            )
            return trajectory

        self._debug_log(
            f"[MoveItBridge] phase planning failed: "
            f"ee_link={self._end_effector_link} "
            f"target_xyz=({target_pose.x:.4f}, {target_pose.y:.4f}, "
            f"{target_pose.z:.4f})"
        )
        self._record_planning_failure(
            clients=clients,
            phase_label=phase_label,
            goal_kind="pose",
            joint_state=joint_state,
            target_xyz_m=(target_pose.x, target_pose.y, target_pose.z),
            error_code=outcome.error_code,
            reason=outcome.failure_reason or "unknown",
        )
        if fallback_joint_goal is None:
            return None
        # MoveItのpose_goal計画が失敗した場合、fallback_joint_goalを使って再試行する。
        return self._plan_joint_goal(
            clients=clients,
            joint_state=joint_state,
            base_frame_id=base_frame_id,
            goal_joint_state=fallback_joint_goal,
            phase_label=phase_label,
        )

    def _plan_seeded_ik_goal(
        self,
        *,
        clients: object,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        target_pose: Pose3D,
        phase_label: str,
    ) -> JointTrajectory | None:
        # MoveItのIK計算をseeded_ik_goalで試み、失敗した場合はpose_goalで計画する。
        moveit_pose = moveit_link_target_pose_from_runtime_tool_pose(
            target_pose,
            link_to_tool_offset_m=(
                self._config.moveit_link_to_runtime_tool_offset_m
            ),
        )
        quaternion = quaternion_from_pose(moveit_pose)
        target_xyz = (moveit_pose.x, moveit_pose.y, moveit_pose.z)
        target_xyzw = (
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        goal: JointStateSnapshot | None = None
        for seed in (joint_state, joint_state, home_joint_state()):
            solution = clients.compute_nearest_ik(
                seed_joint_state=seed,
                base_frame_id=base_frame_id,
                target_pose_xyz=target_xyz,
                target_orientation_xyzw=target_xyzw,
                group_name=self._group_name,
                timeout_sec=self._planning_timeout_sec,
            )
            if solution is None:
                continue
            candidate = arm_joint_goal_from_ik_solution(
                solution_joint_names=solution.joint_names,
                solution_positions_rad=solution.positions_rad,
                arm_joint_names=DEFAULT_JOINT_NAMES,
            )
            if candidate is not None and ik_goal_is_near_seed(
                seed=joint_state,
                goal=candidate,
                max_joint_delta_rad=self._goal_joint1_window_rad,
            ):
                goal = candidate
                break
            if candidate is not None:
                print(
                    "[MoveItBridge] seeded_ik solution rejected as far "
                    f"branch phase={phase_label} "
                    f"goal_q={candidate.positions_rad}",
                    flush=True,
                )
        if goal is None:
            print(
                f"[MoveItBridge] seeded_ik unsolved or far "
                f"phase={phase_label} — falling back to pose goal",
                flush=True,
            )
            return None
        outcome = clients.plan_motion(
            self._build_joint_goal_motion_plan_request(
                joint_state=joint_state,
                base_frame_id=base_frame_id,
                goal_joint_state=goal,
            ),
            timeout_sec=self._planning_timeout_sec,
        )
        trajectory = outcome.trajectory
        if trajectory is not None and trajectory_is_noop(
            trajectory,
            start_joint_state=joint_state,
            tolerance_rad=self._config.noop_trajectory_tolerance_rad,
        ):
            trajectory = None
        if trajectory is None:
            print(
                f"[MoveItBridge] seeded_ik goal plan failed "
                f"phase={phase_label} reason={outcome.failure_reason} "
                f"error_code={outcome.error_code} "
                "— falling back to pose goal",
                flush=True,
            )
            return None
        print(
            f"[MoveItBridge] seeded_ik goal plan succeeded "
            f"phase={phase_label} points={len(trajectory.points)} "
            f"goal_q={goal.positions_rad}",
            flush=True,
        )
        return trajectory

    def _plan_pose_goal(
        self,
        *,
        clients: object,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        target_pose: Pose3D,
        joint_window: tuple[tuple[str, float, float], ...] | None,
    ) -> MotionPlanOutcome:
        outcome = clients.plan_motion(
            self._build_motion_plan_request(
                joint_state=joint_state,
                base_frame_id=base_frame_id,
                target_pose=target_pose,
                joint_window=joint_window,
            ),
            timeout_sec=self._planning_timeout_sec,
        )
        if outcome.trajectory is not None and trajectory_is_noop(
            outcome.trajectory,
            start_joint_state=joint_state,
            tolerance_rad=self._config.noop_trajectory_tolerance_rad,
        ):
            return MotionPlanOutcome(
                None, outcome.error_code, "noop_trajectory"
            )
        return outcome

    def _apply_phase_planning_scene(
        self,
        *,
        clients: object,
        scene_snapshot: SceneSnapshot,
        base_frame_id: str,
        attach_tomato: bool,
        allow_gripper_target_contact: bool,
    ) -> bool:
        return self._scene_manager.apply(
            clients=clients,
            scene_snapshot=scene_snapshot,
            base_frame_id=base_frame_id,
            attach_tomato=attach_tomato,
            allow_gripper_target_contact=allow_gripper_target_contact,
        )

    def _plan_joint_goal(
        self,
        *,
        clients: object,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        goal_joint_state: JointStateSnapshot,
        phase_label: str,
    ) -> JointTrajectory | None:
        outcome = clients.plan_motion(
            self._build_joint_goal_motion_plan_request(
                joint_state=joint_state,
                base_frame_id=base_frame_id,
                goal_joint_state=goal_joint_state,
            ),
            timeout_sec=self._planning_timeout_sec,
        )
        trajectory = outcome.trajectory
        reason = outcome.failure_reason
        if trajectory is not None and trajectory_is_noop(
            trajectory,
            start_joint_state=joint_state,
            tolerance_rad=self._config.noop_trajectory_tolerance_rad,
        ):
            trajectory, reason = None, "noop_trajectory"
        if trajectory is not None:
            print(
                f"[MoveItBridge] joint_goal_fallback succeeded "
                f"phase={phase_label} points={len(trajectory.points)} "
                f"goal_q={goal_joint_state.positions_rad}",
                flush=True,
            )
            return trajectory
        print(
            f"[MoveItBridge] joint_goal_fallback failed phase={phase_label} "
            f"reason={reason} error_code={outcome.error_code}",
            flush=True,
        )
        self._record_planning_failure(
            clients=clients,
            phase_label=phase_label,
            goal_kind="joint",
            joint_state=joint_state,
            target_xyz_m=None,
            error_code=outcome.error_code,
            reason=reason or "unknown",
        )
        return None

    def _record_planning_failure(
        self,
        *,
        clients: object,
        phase_label: str,
        goal_kind: str,
        joint_state: JointStateSnapshot,
        target_xyz_m: tuple[float, float, float] | None,
        error_code: int | None,
        reason: str,
    ) -> None:
        print(
            f"[MoveItBridge] planning_failure phase={phase_label} "
            f"goal_kind={goal_kind} reason={reason} "
            f"error_code={error_code}",
            flush=True,
        )
        if self._diagnostic_dir is None:
            return
        validity = clients.check_state_validity(
            joint_state=joint_state,
            group_name=self._group_name,
            timeout_sec=self._planning_timeout_sec,
        )
        diagnostic = PlanningFailureDiagnostic(
            captured_at_sec=time.time(),
            phase=phase_label or "unknown",
            goal_kind=goal_kind,
            reason=reason,
            error_code=error_code,
            target_xyz_m=target_xyz_m,
            start_joint_names=joint_state.joint_names,
            start_positions_rad=joint_state.positions_rad,
            start_state=validity,
        )
        path = save_planning_failure_diagnostic(
            diagnostic, self._diagnostic_dir
        )
        print(
            "[MoveItBridge] planning_failure_diagnostic "
            f"saved={path is not None} path={path} "
            f"start_state_checked={validity.checked} "
            f"start_state_valid={validity.valid} "
            f"contacts={','.join(validity.contacts) or 'none'}",
            flush=True,
        )

    def _build_motion_plan_request(
        self,
        *,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        target_pose: Pose3D,
        joint_window: tuple[tuple[str, float, float], ...] | None = None,
    ) -> object:
        return build_pose_goal_request(
            config=self._config,
            joint_state=joint_state,
            base_frame_id=base_frame_id,
            target_pose=target_pose,
            joint_window=joint_window,
            debug_log=self._debug_log,
        )

    def _build_joint_goal_motion_plan_request(
        self,
        *,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
        goal_joint_state: JointStateSnapshot,
    ) -> object:
        return build_joint_goal_request(
            config=self._config,
            joint_state=joint_state,
            base_frame_id=base_frame_id,
            goal_joint_state=goal_joint_state,
            debug_log=self._debug_log,
        )

    def _new_motion_plan_request(
        self,
        *,
        joint_state: JointStateSnapshot,
        base_frame_id: str,
    ) -> object:
        return new_motion_plan_request(
            config=self._config,
            joint_state=joint_state,
            base_frame_id=base_frame_id,
        )

    def _debug_log(self, message: str) -> None:
        if self._debug_enabled:
            print(message, flush=True)
