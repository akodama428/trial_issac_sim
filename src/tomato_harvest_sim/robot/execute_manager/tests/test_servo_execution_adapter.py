from __future__ import annotations

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
    decide_joint_jog,
    decide_pose_tracking,
    gripper_state_at_tracking_start,
    servo_target_from_command,
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
    )


def test_target_uses_trajectory_endpoint_and_timeout_budget() -> None:
    target = servo_target_from_command(_command(), started_at_sec=10.0)

    assert target is not None
    assert target.joint_names == ("j1", "j2")
    assert target.positions_rad == (0.8, -0.4)
    assert target.deadline_sec == 17.0
    assert target.pose_tracking_goal is None


def test_grasp_target_uses_terminal_pose_tracking_goal() -> None:
    target = servo_target_from_command(
        _command(phase=PhaseId.MOVING_TO_GRASP), started_at_sec=10.0
    )

    assert target is not None
    assert target.pose_tracking_goal == Pose3D(0.4, 0.1, 0.5584, 180.0, 0.0, 45.0)
    assert gripper_state_at_tracking_start(target) is False


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
