from __future__ import annotations

import json

import pytest

from tomato_harvest_sim.msg.contracts import (
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
    MotionCommand,
    PhaseId,
    PhaseMotionPlan,
    Pose3D,
)
from tomato_harvest_sim.robot.execute_manager.servo_execution_adapter import (
    SERVO_JOINT_GAIN,
    SERVO_MAX_VELOCITY_RAD_S,
    CommandLifecycle,
    StallDetector,
    TrajectoryReference,
    decide_joint_jog,
    decide_time_synchronized_joint_jog,
    decide_pose_tracking,
    execution_status_payload,
    gripper_state_for_tracking,
    progress_scale,
    servo_target_from_command,
    trajectory_reference_at,
)


def _command(*, phase: PhaseId = PhaseId.MOVING_TO_PREGRASP) -> MotionCommand:
    return MotionCommand(
        command_name="move_to_grasp",
        planner_name="moveit2",
        target_pose=Pose3D(0.4, 0.1, 0.5, 180.0, 0.0, 90.0),
        gripper_closed=True,
        phase_motion_plan=PhaseMotionPlan(
            phase_id=phase,
            phase_goal_pose=Pose3D(0.4, 0.1, 0.5, 180.0, 0.0, 90.0),
            active_waypoints=(),
            joint_trajectory=JointTrajectory(
                joint_names=("j1", "j2"),
                points=(JointTrajectoryPoint((0.8, -0.4), 2.0),),
            ),
        ),
        terminal_pose_tracking=phase is PhaseId.MOVING_TO_GRASP,
    )


def test_target_uses_trajectory_endpoint_and_timeout_budget() -> None:
    target = servo_target_from_command(_command(), started_at_sec=10.0)

    assert target is not None
    assert target.joint_names == ("j1", "j2")
    assert target.positions_rad == (0.8, -0.4)
    assert tuple(point.positions_rad for point in target.trajectory_points) == ((0.8, -0.4),)
    assert target.deadline_sec == 19.0
    assert target.pose_tracking_goal is None


def test_target_supports_short_deadline_fault_injection() -> None:
    target = servo_target_from_command(
        _command(), started_at_sec=10.0,
        deadline_stretch_factor=0.0, timeout_margin_sec=0.2,
    )

    assert target is not None
    assert target.deadline_sec == 10.2


def test_lifecycle_advances_waypoints_in_order() -> None:
    command = _command()
    trajectory = JointTrajectory(
        joint_names=("j1", "j2"),
        points=(
            JointTrajectoryPoint((0.2, -0.1), 1.0),
            JointTrajectoryPoint((0.8, -0.4), 2.0),
        ),
    )
    command = MotionCommand(
        command.command_name,
        command.planner_name,
        command.target_pose,
        command.gripper_closed,
        PhaseMotionPlan(
            command.phase_motion_plan.phase_id,
            command.phase_motion_plan.phase_goal_pose,
            (),
            trajectory,
        ),
    )
    target = servo_target_from_command(command, started_at_sec=10.0)
    lifecycle = CommandLifecycle()
    assert target is not None
    lifecycle.start(target, 10.0)

    assert lifecycle.reference_elapsed_sec == 0.0
    lifecycle.update_reference_clock(now_sec=10.2, progress_scale=1.0)
    assert lifecycle.reference_elapsed_sec == 0.2
    lifecycle.update_reference_clock(now_sec=10.4, progress_scale=0.5)
    assert lifecycle.reference_elapsed_sec == 0.3
    lifecycle.update_reference_clock(now_sec=10.6, progress_scale=0.0)
    assert lifecycle.reference_elapsed_sec == 0.3


@pytest.mark.parametrize(
    ("error_rad", "expected"),
    ((0.05, 1.0), (0.10, 1.0), (0.15, 0.5), (0.20, 0.0), (0.25, 0.0)),
)
def test_progress_scale_is_linear_in_tracking_error_band(
    error_rad: float, expected: float
) -> None:
    assert progress_scale(error_rad) == pytest.approx(expected)


def test_trajectory_reference_interpolates_boundaries_and_segment_velocity() -> None:
    command = _command()
    trajectory = JointTrajectory(
        joint_names=("j1", "j2"),
        points=(
            JointTrajectoryPoint((0.0, 0.0), 0.0),
            JointTrajectoryPoint((1.0, -0.5), 2.0),
        ),
    )
    command = MotionCommand(
        command.command_name, command.planner_name, command.target_pose,
        command.gripper_closed,
        PhaseMotionPlan(command.phase_motion_plan.phase_id,
                        command.phase_motion_plan.phase_goal_pose, (), trajectory),
    )
    target = servo_target_from_command(command, started_at_sec=10.0)
    assert target is not None

    assert trajectory_reference_at(target, -1.0) == TrajectoryReference(
        (0.0, 0.0), (0.5, -0.25), False
    )
    assert trajectory_reference_at(target, 1.0) == TrajectoryReference(
        (0.5, -0.25), (0.5, -0.25), False
    )
    assert trajectory_reference_at(target, 3.0) == TrajectoryReference(
        (1.0, -0.5), (0.0, 0.0), True
    )


def test_trajectory_reference_prefers_planned_velocities() -> None:
    command = _command()
    trajectory = JointTrajectory(
        joint_names=("j1", "j2"),
        points=(
            JointTrajectoryPoint((0.0, 0.0), 0.0, (0.1, -0.1)),
            JointTrajectoryPoint((1.0, -0.5), 2.0, (0.3, -0.2)),
        ),
    )
    command = MotionCommand(
        command.command_name, command.planner_name, command.target_pose,
        command.gripper_closed,
        PhaseMotionPlan(command.phase_motion_plan.phase_id,
                        command.phase_motion_plan.phase_goal_pose, (), trajectory),
    )
    target = servo_target_from_command(command, started_at_sec=10.0)
    assert target is not None

    assert trajectory_reference_at(target, 1.0).velocities_rad_s == pytest.approx((0.2, -0.15))


def test_time_synchronized_jog_combines_feed_forward_and_feedback() -> None:
    target = servo_target_from_command(_command(), started_at_sec=10.0)
    assert target is not None
    reference = TrajectoryReference((0.4, -0.2), (0.1, -0.1), False)
    state = JointStateSnapshot(("j1", "j2"), (0.3, -0.25))

    decision = decide_time_synchronized_joint_jog(
        target, state, reference, gain=2.0, max_velocity_rad_s=0.8
    )

    assert decision is not None
    assert decision.velocities_rad_s == pytest.approx((0.3, 0.0))
    assert decision.max_error_rad == 0.1


def test_time_stretch_preserves_feed_forward_while_reference_clock_is_paused() -> None:
    target = servo_target_from_command(_command(), started_at_sec=10.0)
    assert target is not None
    reference = TrajectoryReference((0.4, -0.2), (0.8, -0.8), False)
    state = JointStateSnapshot(("j1", "j2"), (0.2, -0.2))

    decision = decide_time_synchronized_joint_jog(
        target, state, reference, gain=3.0, max_velocity_rad_s=0.8
    )

    assert decision is not None
    assert decision.max_error_rad == 0.2
    assert decision.progress_scale == 0.0
    assert decision.velocities_rad_s == pytest.approx((0.6, 0.0))


def test_progress_scaling_scales_feed_forward_and_preserves_feedback() -> None:
    target = servo_target_from_command(_command(), started_at_sec=10.0)
    assert target is not None
    reference = TrajectoryReference((0.4, -0.2), (0.4, -0.4), False)
    state = JointStateSnapshot(("j1", "j2"), (0.25, -0.2))

    decision = decide_time_synchronized_joint_jog(
        target, state, reference, gain=2.0, max_velocity_rad_s=0.8
    )

    assert decision is not None
    assert decision.progress_scale == pytest.approx(0.5)
    assert decision.velocities_rad_s == pytest.approx((0.5, -0.2))


def test_stall_detector_requires_stationary_feedback_only_for_half_second() -> None:
    detector = StallDetector()

    assert detector.update(
        now_sec=1.0, progress_scale=0.0, velocities_rad_s=(0.01, -0.02)
    ) is False
    assert detector.update(
        now_sec=1.49, progress_scale=0.0, velocities_rad_s=(0.01, -0.02)
    ) is False
    assert detector.update(
        now_sec=1.5, progress_scale=0.0, velocities_rad_s=(0.01, -0.02)
    ) is True
    assert detector.elapsed_sec == pytest.approx(0.5)


def test_stall_detector_resets_for_motion_scaling_or_missing_velocity() -> None:
    detector = StallDetector()
    detector.update(now_sec=1.0, progress_scale=0.0, velocities_rad_s=(0.0,))

    assert detector.update(
        now_sec=1.4, progress_scale=0.0, velocities_rad_s=(0.05,)
    ) is False
    assert detector.elapsed_sec == 0.0
    detector.update(now_sec=2.0, progress_scale=0.0, velocities_rad_s=(0.0,))
    assert detector.update(
        now_sec=2.4, progress_scale=0.1, velocities_rad_s=(0.0,)
    ) is False
    assert detector.update(
        now_sec=3.0, progress_scale=0.0, velocities_rad_s=()
    ) is False
    assert detector.elapsed_sec == 0.0


def test_execution_status_adds_progress_and_stall_fields_compatibly() -> None:
    payload = execution_status_payload(
        "running",
        max_error_rad=0.2,
        progress_scale=0.0,
        stall_elapsed_sec=0.5,
        stalled=True,
    )

    assert json.loads(payload) == {
        "status": "running",
        "tracking_error_rad": 0.2,
        "max_joint_error_rad": 0.2,
        "scale": 0.0,
        "stall_elapsed_sec": 0.5,
        "stalled": True,
    }


def test_grasp_target_uses_terminal_pose_tracking_goal() -> None:
    target = servo_target_from_command(
        _command(phase=PhaseId.MOVING_TO_GRASP), started_at_sec=10.0
    )

    assert target is not None
    assert target.pose_tracking_goal == Pose3D(0.4, 0.1, 0.5584, 180.0, 0.0, 45.0)
    assert gripper_state_for_tracking(target) is True


def test_phase_id_does_not_implicitly_enable_pose_tracking() -> None:
    command = _command(phase=PhaseId.MOVING_TO_GRASP)
    command = MotionCommand(
        command_name=command.command_name,
        planner_name=command.planner_name,
        target_pose=command.target_pose,
        gripper_closed=command.gripper_closed,
        phase_motion_plan=command.phase_motion_plan,
        terminal_pose_tracking=False,
    )

    target = servo_target_from_command(command, started_at_sec=10.0)

    assert target is not None
    assert target.pose_tracking_goal is None


def test_closed_hold_command_never_reopens_gripper_during_pose_tracking() -> None:
    target = servo_target_from_command(
        _command(phase=PhaseId.MOVING_TO_GRASP), started_at_sec=10.0
    )

    assert target is not None
    assert gripper_state_for_tracking(target) is True


def test_pose_tracking_success_does_not_override_open_gripper_command() -> None:
    command = _command(phase=PhaseId.MOVING_TO_GRASP)
    command = MotionCommand(
        command.command_name,
        command.planner_name,
        command.target_pose,
        False,
        command.phase_motion_plan,
    )
    target = servo_target_from_command(command, started_at_sec=10.0)

    assert target is not None
    assert gripper_state_for_tracking(target) is False


def test_pose_tracking_requires_stable_position_and_orientation_tolerance() -> None:
    target = servo_target_from_command(
        _command(phase=PhaseId.MOVING_TO_GRASP), started_at_sec=10.0
    )
    assert target is not None

    reached = decide_pose_tracking(
        target, Pose3D(0.402, 0.1, 0.5584, 180.0, 0.0, 45.5)
    )
    outside = decide_pose_tracking(
        target, Pose3D(0.42, 0.1, 0.5584, 180.0, 0.0, 45.5)
    )

    assert reached is not None and reached.reached is True
    assert reached.position_error_m == 0.002
    assert outside is not None and outside.reached is False


def test_pose_tracking_accepts_observed_sub_tenth_millimeter_boundary_jitter() -> None:
    target = servo_target_from_command(
        _command(phase=PhaseId.MOVING_TO_GRASP), started_at_sec=10.0
    )
    assert target is not None and target.pose_tracking_goal is not None
    goal = target.pose_tracking_goal

    decision = decide_pose_tracking(
        target, Pose3D(goal.x + 0.00505, goal.y, goal.z, goal.roll, goal.pitch, goal.yaw)
    )

    assert decision is not None and decision.reached is True


def test_joint_jog_reorders_feedback_and_clamps_velocity() -> None:
    target = servo_target_from_command(_command(), started_at_sec=10.0)
    assert target is not None
    state = JointStateSnapshot(("j2", "j1"), (-0.2, 0.0))

    decision = decide_joint_jog(target, state, gain=2.0, max_velocity_rad_s=0.5)

    assert decision.reached is False
    assert decision.joint_names == ("j1", "j2")
    assert decision.velocities_rad_s == (0.5, -0.4)
    assert decision.max_error_rad == 0.8


def test_joint_jog_marks_target_reached_with_zero_velocity() -> None:
    target = servo_target_from_command(_command(), started_at_sec=10.0)
    assert target is not None
    state = JointStateSnapshot(("j1", "j2"), (0.79, -0.42))

    decision = decide_joint_jog(target, state, tolerance_rad=0.03)

    assert decision.reached is True
    assert decision.velocities_rad_s == (0.0, 0.0)
    assert decision.max_error_rad == 0.02


def test_joint_jog_rejects_incomplete_feedback() -> None:
    target = servo_target_from_command(_command(), started_at_sec=10.0)
    assert target is not None

    assert decide_joint_jog(target, JointStateSnapshot(("j1",), (0.0,))) is None


def test_command_without_trajectory_has_no_servo_target() -> None:
    command = MotionCommand("stop", "stop", None, False, None)

    assert servo_target_from_command(command, started_at_sec=10.0) is None


def test_tuned_profile_stays_below_slowest_panda_joint_velocity_limit() -> None:
    """Panda joint 2の公称上限1 rad/sに安全余裕を残す。"""
    assert SERVO_MAX_VELOCITY_RAD_S == 0.8
    assert SERVO_MAX_VELOCITY_RAD_S < 1.0
    assert SERVO_JOINT_GAIN == 3.0
