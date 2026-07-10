"""motion_command_node — フェーズ・計画・現在関節状態から motion_command を生成し publish する。

アーキテクチャ仕様: docs/index.html §motion_command_node
"""
from __future__ import annotations

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    JointTrajectory,
    JointTrajectoryPoint,
    MotionCommand,
    PhaseId,
    PhaseMotionPlan,
)
from tomato_harvest_sim.msg.topics import DEFAULT_JOINT_NAMES, DEFAULT_JOINT_POSITIONS_RAD


def _select_arm_joint_positions(joint_state: JointStateSnapshot) -> tuple[tuple[str, ...], tuple[float, ...]]:
    """arm JTC が管理する関節だけを名前で選択する。

    Args:
        joint_state: arm と gripper を含み得る最新関節状態。

    Returns:
        controller 順に並べた関節名と位置。

    Raises:
        ValueError: 関節名と位置の数が不一致、または arm 関節がない場合。
    """
    if len(joint_state.joint_names) != len(joint_state.positions_rad):
        raise ValueError("joint state names and positions must have the same length")
    positions_by_name = dict(zip(joint_state.joint_names, joint_state.positions_rad, strict=True))
    arm_joint_names = tuple(name for name in DEFAULT_JOINT_NAMES if name in positions_by_name)
    if not arm_joint_names:
        raise ValueError("joint state has no arm controller joints")
    return arm_joint_names, tuple(positions_by_name[name] for name in arm_joint_names)


def _stop_trajectory(joint_state: JointStateSnapshot) -> JointTrajectory:
    """現在関節位置を単一ウェイポイントとする停止軌道を返す。"""
    joint_names, positions_rad = _select_arm_joint_positions(joint_state)
    return JointTrajectory(
        joint_names=joint_names,
        points=(JointTrajectoryPoint(
            positions_rad=positions_rad,
            time_from_start_sec=0.0,
        ),),
    )


def build_motion_command(
    phase: HarvestTaskPhase,
    plan: HarvestMotionPlan,
    current_joints: JointStateSnapshot,
) -> MotionCommand:
    """フェーズ・計画・現在関節状態から MotionCommand を生成する。

    アーキテクチャ仕様のフェーズ別出力仕様に従い、joint_trajectory と
    gripper_closed を決定する。joint_trajectory は常に非 null。
    """
    if phase is HarvestTaskPhase.MOVING_TO_PREGRASP:
        return _make_command("move_to_pregrasp", PhaseId.MOVING_TO_PREGRASP,
                             plan.pregrasp_pose, plan.pregrasp_joint_trajectory, True, plan)

    if phase is HarvestTaskPhase.MOVING_TO_GRASP:
        return _make_command("move_to_grasp", PhaseId.MOVING_TO_GRASP,
                             plan.grasp_pose, plan.grasp_joint_trajectory, False, plan)

    if phase is HarvestTaskPhase.AT_GRASP:
        return _make_stop_command("hold_at_grasp", PhaseId.MOVING_TO_GRASP,
                                  plan.grasp_pose, True, current_joints)

    if phase is HarvestTaskPhase.GRASP_EVALUATION:
        return _make_stop_command("hold_grasp_eval", PhaseId.MOVING_TO_GRASP,
                                  plan.grasp_pose, True, current_joints)

    if phase is HarvestTaskPhase.DETACHING:
        return _make_command("pull_to_detach", PhaseId.PULL_TO_DETACH,
                             plan.pull_pose, plan.pull_joint_trajectory, True, plan)

    if phase is HarvestTaskPhase.MOVING_TO_PLACE:
        return _make_command("move_to_place", PhaseId.MOVING_TO_PLACE,
                             plan.place_pose, plan.place_joint_trajectory, True, plan)

    if phase is HarvestTaskPhase.PLACED:
        return _make_stop_command("hold_placed", PhaseId.MOVING_TO_PLACE,
                                  plan.place_pose, False, current_joints)

    if phase is HarvestTaskPhase.RETURNING_HOME:
        joint_names, current_positions = _select_arm_joint_positions(current_joints)
        home_positions_by_name = dict(zip(
            DEFAULT_JOINT_NAMES, DEFAULT_JOINT_POSITIONS_RAD, strict=True
        ))
        home_trajectory = JointTrajectory(
            joint_names=joint_names,
            points=(
                JointTrajectoryPoint(
                    positions_rad=current_positions,
                    time_from_start_sec=0.0,
                ),
                JointTrajectoryPoint(
                    positions_rad=tuple(home_positions_by_name[name] for name in joint_names),
                    time_from_start_sec=10.0,
                ),
            ),
        )
        return MotionCommand(
            command_name="move_home",
            planner_name="direct",
            target_pose=None,
            gripper_closed=False,
            phase_motion_plan=PhaseMotionPlan(
                phase_id=PhaseId.RETURNING_HOME,
                phase_goal_pose=None,
                active_waypoints=(),
                joint_trajectory=home_trajectory,
            ),
        )

    raise ValueError(f"build_motion_command: unsupported phase {phase!r}")


def _make_command(
    command_name: str,
    phase_id: PhaseId,
    goal_pose: Pose3D | None,
    trajectory: JointTrajectory | None,
    gripper_closed: bool,
    plan: HarvestMotionPlan,
) -> MotionCommand:
    return MotionCommand(
        command_name=command_name,
        planner_name=plan.planner_name,
        target_pose=goal_pose,
        gripper_closed=gripper_closed,
        phase_motion_plan=PhaseMotionPlan(
            phase_id=phase_id,
            phase_goal_pose=goal_pose,
            active_waypoints=(),
            joint_trajectory=trajectory,
        ),
    )


def _make_stop_command(
    command_name: str,
    phase_id: PhaseId,
    goal_pose: Pose3D | None,
    gripper_closed: bool,
    current_joints: JointStateSnapshot,
) -> MotionCommand:
    return MotionCommand(
        command_name=command_name,
        planner_name="stop",
        target_pose=goal_pose,
        gripper_closed=gripper_closed,
        phase_motion_plan=PhaseMotionPlan(
            phase_id=phase_id,
            phase_goal_pose=goal_pose,
            active_waypoints=(),
            joint_trajectory=_stop_trajectory(current_joints),
        ),
    )


def main() -> None:
    import rclpy
    from std_msgs.msg import String
    from rclpy.node import Node
    from tomato_harvest_sim.msg.topics import (
        PHASE_TOPIC, HARVEST_MOTION_PLAN_TOPIC,
        MOTION_COMMAND_TOPIC, JOINT_STATES_TOPIC,
    )
    from tomato_harvest_sim.msg.serialization import motion_command_to_json

    rclpy.init()

    class MotionCommandNode(Node):  # type: ignore[misc]
        def __init__(self) -> None:
            super().__init__("motion_command_node")
            self._phase: HarvestTaskPhase | None = None
            self._plan: HarvestMotionPlan | None = None
            self._joint_state: JointStateSnapshot | None = None

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
            self._plan = harvest_motion_plan_from_json(msg.data)
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
                cmd = build_motion_command(self._phase, self._plan, self._joint_state)
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
