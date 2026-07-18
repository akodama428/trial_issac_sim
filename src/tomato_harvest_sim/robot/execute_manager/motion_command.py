"""motion_command_node — フェーズ・計画・現在関節状態から motion_command を生成し publish する。

アーキテクチャ仕様: docs/index.html §motion_command_node
"""
from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass, replace

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
    MotionCommand,
    MotionKind,
    PhaseId,
    PhaseMotionPlan,
    Pose3D,
)
from tomato_harvest_sim.msg.topics import DEFAULT_JOINT_NAMES, DEFAULT_JOINT_POSITIONS_RAD


@dataclass(frozen=True)
class PhaseCommandSpec:
    command_name: str
    phase_id: PhaseId
    motion_kind: MotionKind
    terminal_pose_tracking: bool
    gripper_closed: bool
    pose_field: str | None
    trajectory_field: str | None


PHASE_COMMAND_TABLE = {
    HarvestTaskPhase.MOVING_TO_PREGRASP: PhaseCommandSpec("move_to_pregrasp", PhaseId.MOVING_TO_PREGRASP, MotionKind.FOLLOW_TRAJECTORY, False, True, "pregrasp_pose", "pregrasp_joint_trajectory"),
    HarvestTaskPhase.MOVING_TO_GRASP: PhaseCommandSpec("move_to_grasp", PhaseId.MOVING_TO_GRASP, MotionKind.FOLLOW_TRAJECTORY, True, False, "grasp_pose", "grasp_joint_trajectory"),
    HarvestTaskPhase.AT_GRASP: PhaseCommandSpec("hold_at_grasp", PhaseId.MOVING_TO_GRASP, MotionKind.HOLD, True, True, "grasp_pose", None),
    HarvestTaskPhase.GRASP_EVALUATION: PhaseCommandSpec("hold_grasp_eval", PhaseId.MOVING_TO_GRASP, MotionKind.HOLD, True, True, "grasp_pose", None),
    HarvestTaskPhase.DETACHING: PhaseCommandSpec("pull_to_detach", PhaseId.PULL_TO_DETACH, MotionKind.FOLLOW_TRAJECTORY, False, True, "pull_pose", "pull_joint_trajectory"),
    HarvestTaskPhase.MOVING_TO_PLACE: PhaseCommandSpec("move_to_place", PhaseId.MOVING_TO_PLACE, MotionKind.FOLLOW_TRAJECTORY, False, True, "place_pose", "place_joint_trajectory"),
    HarvestTaskPhase.RELEASING: PhaseCommandSpec("release_in_tray", PhaseId.MOVING_TO_PLACE, MotionKind.HOLD, False, False, "place_pose", None),
    HarvestTaskPhase.PLACED: PhaseCommandSpec("hold_placed", PhaseId.MOVING_TO_PLACE, MotionKind.HOLD, False, False, "place_pose", None),
    HarvestTaskPhase.RETURNING_HOME: PhaseCommandSpec("move_home", PhaseId.RETURNING_HOME, MotionKind.FOLLOW_TRAJECTORY, False, False, None, "home_joint_trajectory"),
}

# Issue #59 A/B実験: 静止物体把持でTF直接追従(Servo pose tracking)が本当に必要かを
# 検証するため、grasp系phaseのterminal_pose_trackingをdirect JTC実行へ切り替える対象。
GRASP_SERVO_PHASES = frozenset({
    HarvestTaskPhase.MOVING_TO_GRASP,
    HarvestTaskPhase.AT_GRASP,
    HarvestTaskPhase.GRASP_EVALUATION,
})

GRASP_DIRECT_JTC_ENV = "TOMATO_HARVEST_GRASP_DIRECT_JTC"


def grasp_direct_jtc_enabled(environ: Mapping[str, str] | None = None) -> bool:
    """grasp系phaseをdirect JTC実行へ切り替えるA/B実験フラグを環境変数から読む。

    Args:
        environ: 参照する環境変数マップ。Noneならos.environ。

    Returns:
        `TOMATO_HARVEST_GRASP_DIRECT_JTC` が "1"/"true"/"yes" のときTrue。
    """
    source = os.environ if environ is None else environ
    return source.get(GRASP_DIRECT_JTC_ENV, "").strip().lower() in {"1", "true", "yes"}


def _arm_only_trajectory(trajectory: JointTrajectory) -> JointTrajectory:
    """arm JTCとの共通境界でgripper関節をtrajectoryから除外する。

    Args:
        trajectory: planner種別やphaseを問わない入力trajectory。

    Returns:
        arm controller順に関節と各pointの値を射影したtrajectory。

    Raises:
        ValueError: trajectoryの配列長が不正、またはarm関節がない場合。
    """
    index_by_name = {name: index for index, name in enumerate(trajectory.joint_names)}
    arm_joint_names = tuple(name for name in DEFAULT_JOINT_NAMES if name in index_by_name)
    if not arm_joint_names:
        raise ValueError("trajectory has no arm controller joints")
    arm_indices = tuple(index_by_name[name] for name in arm_joint_names)
    points: list[JointTrajectoryPoint] = []
    for point in trajectory.points:
        if len(point.positions_rad) != len(trajectory.joint_names):
            raise ValueError("trajectory names and positions must have the same length")
        velocities = point.velocities_rad_s
        if velocities is not None and len(velocities) != len(trajectory.joint_names):
            raise ValueError("trajectory names and velocities must have the same length")
        points.append(replace(
            point,
            positions_rad=tuple(point.positions_rad[index] for index in arm_indices),
            velocities_rad_s=(
                tuple(velocities[index] for index in arm_indices)
                if velocities is not None else None
            ),
        ))
    if arm_joint_names == trajectory.joint_names:
        return trajectory
    return replace(trajectory, joint_names=arm_joint_names, points=tuple(points))


def _arm_only_command(command: MotionCommand) -> MotionCommand:
    """すべてのphaseに同一のarm/gripper契約境界を適用する。"""
    phase_plan = command.phase_motion_plan
    if phase_plan is None or phase_plan.joint_trajectory is None:
        return command
    return replace(
        command,
        phase_motion_plan=replace(
            phase_plan,
            joint_trajectory=_arm_only_trajectory(phase_plan.joint_trajectory),
        ),
    )


def _stop_trajectory(joint_state: JointStateSnapshot) -> JointTrajectory:
    """現在関節位置を単一ウェイポイントとする停止軌道を返す。"""
    return JointTrajectory(
        joint_names=joint_state.joint_names,
        points=(JointTrajectoryPoint(
            positions_rad=joint_state.positions_rad,
            time_from_start_sec=0.0,
        ),),
    )


def build_motion_command(
    phase: HarvestTaskPhase,
    plan: HarvestMotionPlan,
    current_joints: JointStateSnapshot,
    *,
    grasp_direct_jtc: bool = False,
) -> MotionCommand:
    """フェーズ・計画・現在関節状態から MotionCommand を生成する。

    アーキテクチャ仕様のフェーズ別出力仕様に従い、joint_trajectory と
    gripper_closed を決定する。joint_trajectory は常に非 null。

    Args:
        phase: 現在のharvestフェーズ。
        plan: 採択済みのmotion plan。
        current_joints: 現在の関節状態。
        grasp_direct_jtc: Trueならgrasp系phaseのpose trackingを無効化し
            direct JTC実行へ切り替える (Issue #59 A/B実験のB条件)。
    """
    return _arm_only_command(_build_phase_motion_command(
        phase, plan, current_joints, grasp_direct_jtc=grasp_direct_jtc,
    ))


def _build_phase_motion_command(
    phase: HarvestTaskPhase,
    plan: HarvestMotionPlan,
    current_joints: JointStateSnapshot,
    *,
    grasp_direct_jtc: bool = False,
) -> MotionCommand:
    """宣言テーブルの実行意図からcommandを組み立てる。"""
    spec = PHASE_COMMAND_TABLE.get(phase)
    if spec is None:
        raise ValueError(f"build_motion_command: unsupported phase {phase!r}")
    if grasp_direct_jtc and phase in GRASP_SERVO_PHASES:
        spec = replace(spec, terminal_pose_tracking=False)
    if phase is HarvestTaskPhase.RETURNING_HOME:
        # abort復旧 (suffix replan) が刻んだ衝突考慮済みのhome区間trajectoryが
        # あれば優先する (Issue #32)。無ければ従来どおりhome定数への直行軌道を使う。
        if plan.home_joint_trajectory is not None:
            return _make_command(spec, None, plan.home_joint_trajectory, plan)
        home_positions_by_name = dict(zip(
            DEFAULT_JOINT_NAMES, DEFAULT_JOINT_POSITIONS_RAD, strict=True
        ))
        home_trajectory = JointTrajectory(
            joint_names=current_joints.joint_names,
            points=(
                JointTrajectoryPoint(
                    positions_rad=current_joints.positions_rad,
                    time_from_start_sec=0.0,
                ),
                JointTrajectoryPoint(
                    positions_rad=tuple(
                        home_positions_by_name.get(name, position)
                        for name, position in zip(
                            current_joints.joint_names,
                            current_joints.positions_rad,
                            strict=True,
                        )
                    ),
                    time_from_start_sec=10.0,
                ),
            ),
        )
        return MotionCommand(
            command_name=spec.command_name,
            planner_name="direct",
            target_pose=None,
            gripper_closed=False,
            phase_motion_plan=PhaseMotionPlan(
                phase_id=spec.phase_id,
                phase_goal_pose=None,
                active_waypoints=(),
                joint_trajectory=home_trajectory,
            ),
            motion_kind=spec.motion_kind,
            terminal_pose_tracking=spec.terminal_pose_tracking,
        )

    goal_pose = getattr(plan, spec.pose_field) if spec.pose_field is not None else None
    if spec.motion_kind is MotionKind.HOLD:
        return _make_stop_command(spec, goal_pose, current_joints)
    trajectory = getattr(plan, spec.trajectory_field) if spec.trajectory_field is not None else None
    return _make_command(spec, goal_pose, trajectory, plan)


def _make_command(
    spec: PhaseCommandSpec,
    goal_pose: Pose3D | None,
    trajectory: JointTrajectory | None,
    plan: HarvestMotionPlan,
) -> MotionCommand:
    return MotionCommand(
        command_name=spec.command_name,
        planner_name=plan.planner_name,
        target_pose=goal_pose,
        gripper_closed=spec.gripper_closed,
        phase_motion_plan=PhaseMotionPlan(
            phase_id=spec.phase_id,
            phase_goal_pose=goal_pose,
            active_waypoints=(),
            joint_trajectory=trajectory,
        ),
        motion_kind=spec.motion_kind,
        terminal_pose_tracking=spec.terminal_pose_tracking,
    )


def _make_stop_command(
    spec: PhaseCommandSpec,
    goal_pose: Pose3D | None,
    current_joints: JointStateSnapshot,
) -> MotionCommand:
    return MotionCommand(
        command_name=spec.command_name,
        planner_name="stop",
        target_pose=goal_pose,
        gripper_closed=spec.gripper_closed,
        phase_motion_plan=PhaseMotionPlan(
            phase_id=spec.phase_id,
            phase_goal_pose=goal_pose,
            active_waypoints=(),
            joint_trajectory=_stop_trajectory(current_joints),
        ),
        motion_kind=spec.motion_kind,
        terminal_pose_tracking=spec.terminal_pose_tracking,
    )


def main() -> None:
    import time

    import rclpy
    from std_msgs.msg import String
    from rclpy.node import Node
    from tomato_harvest_sim.msg.topics import (
        PHASE_TOPIC, HARVEST_MOTION_PLAN_TOPIC,
        MOTION_COMMAND_TOPIC, JOINT_STATES_TOPIC,
    )
    from tomato_harvest_sim.msg.serialization import motion_command_to_json
    from tomato_harvest_sim.robot.execute_manager.plan_arbitration import (
        evaluate_plan_arbitration,
    )
    from tomato_harvest_sim.robot.motion_planner.observability import metric_line

    rclpy.init()

    class MotionCommandNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("motion_command_node")
            self._phase: HarvestTaskPhase | None = None
            self._plan: HarvestMotionPlan | None = None
            self._joint_state: JointStateSnapshot | None = None
            self._grasp_direct_jtc = grasp_direct_jtc_enabled()
            self.get_logger().info(metric_line(
                "grasp_direct_jtc_flag", enabled=self._grasp_direct_jtc,
            ))

            self.create_subscription(String, PHASE_TOPIC, self._on_phase, 10)
            self.create_subscription(String, HARVEST_MOTION_PLAN_TOPIC, self._on_plan, 10)
            self.create_subscription(
                __import__("sensor_msgs.msg", fromlist=["JointState"]).JointState,
                JOINT_STATES_TOPIC, self._on_joint_state, 10,
            )
            self._pub = self.create_publisher(String, MOTION_COMMAND_TOPIC, 10)

        def _on_phase(self, msg: String) -> None:
            try:
                self._phase = HarvestTaskPhase(msg.data)
            except ValueError:
                return
            self._try_publish()

        def _on_plan(self, msg: String) -> None:
            from tomato_harvest_sim.msg.serialization import harvest_motion_plan_from_json
            candidate = harvest_motion_plan_from_json(msg.data)
            decision = evaluate_plan_arbitration(
                candidate=candidate,
                current_plan=self._plan,
                current_phase=self._phase,
            )
            self.get_logger().info(metric_line(
                "plan_adopted" if decision.adopted else "plan_rejected",
                reason=decision.reason,
                plan_revision=candidate.plan_revision,
                current_revision=self._plan.plan_revision if self._plan is not None else None,
                planned_from_phase=(
                    candidate.planned_from_phase.value
                    if candidate.planned_from_phase is not None else None
                ),
                phase=self._phase.value if self._phase is not None else None,
                producer_kind=candidate.producer_kind.value,
                producer_instance_id=candidate.producer_instance_id,
                plan_age_sec=(
                    round(time.time() - candidate.generated_at_sec, 3)
                    if candidate.generated_at_sec is not None else None
                ),
            ))
            if not decision.adopted:
                return
            self._plan = candidate
            self._try_publish()

        def _on_joint_state(self, msg: object) -> None:
            self._joint_state = JointStateSnapshot(
                joint_names=tuple(str(n) for n in getattr(msg, "name", ())),
                positions_rad=tuple(float(v) for v in getattr(msg, "position", ())),
            )

        def _try_publish(self) -> None:
            if self._phase is None or self._plan is None or self._joint_state is None:
                return
            try:
                cmd = build_motion_command(
                    self._phase, self._plan, self._joint_state,
                    grasp_direct_jtc=self._grasp_direct_jtc,
                )
            except ValueError:
                return
            out = String()
            out.data = motion_command_to_json(cmd)
            self._pub.publish(out)

    node = MotionCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
