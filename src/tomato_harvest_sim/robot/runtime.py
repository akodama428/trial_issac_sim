from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

from tomato_harvest_sim.api.bridge import BridgeProtocol
from tomato_harvest_sim.api.contracts import (
    ControlCommand,
    HarvestMotionPlan,
    HarvestTaskPhase,
    MotionCommand,
    Pose3D,
    PhaseMotionPlan,
    RobotRuntimeState,
    ScenePhase,
    SceneSnapshot,
    TargetEstimate,
)
from tomato_harvest_sim.robot.behavior_planner import BehaviorPlanner
from tomato_harvest_sim.robot.motion import MoveItStyleMotionPublisher
from tomato_harvest_sim.robot.perception import TomatoTargetEstimator
from tomato_harvest_sim.robot.motion_planner import build_planner

if TYPE_CHECKING:
    from tomato_harvest_sim.robot.trajectory_tracking import TrajectoryTrackingCoordinator


@dataclass
class RobotState:
    runtime_state: RobotRuntimeState
    task_phase: HarvestTaskPhase
    last_seen_phase: ScenePhase
    last_scene_snapshot: SceneSnapshot | None
    last_target_estimate: TargetEstimate | None
    last_harvest_motion_plan: HarvestMotionPlan | None
    last_motion_command: MotionCommand | None
    last_phase_motion_plan: PhaseMotionPlan | None
    planner_backend_name: str
    grasp_evaluation_wait_steps: int
    grasp_settle_wait_steps: int
    phase_success_stable_steps: int


class HarvestRuntime:
    GRASP_SETTLE_STEPS = BehaviorPlanner.GRASP_SETTLE_STEPS

    def __init__(
        self,
        *,
        grasp_lateral_offset_m: float = 0.0,
        executor: TrajectoryTrackingCoordinator | None = None,
    ) -> None:
        planner, planner_info = build_planner(grasp_lateral_offset_m=grasp_lateral_offset_m)
        self.state = RobotState(
            runtime_state=RobotRuntimeState.BOOTING,
            task_phase=HarvestTaskPhase.IDLE,
            last_seen_phase=ScenePhase.BOOTING,
            last_scene_snapshot=None,
            last_target_estimate=None,
            last_harvest_motion_plan=None,
            last_motion_command=None,
            last_phase_motion_plan=None,
            planner_backend_name=planner_info.name,
            grasp_evaluation_wait_steps=0,
            grasp_settle_wait_steps=0,
            phase_success_stable_steps=0,
        )
        self._estimator = TomatoTargetEstimator()
        self._planner = planner
        self._behavior_planner = BehaviorPlanner(
            state=self.state,
            motion_publisher=MoveItStyleMotionPublisher(),
        )
        self._executor = executor

    @property
    def has_executor(self) -> bool:
        return self._executor is not None

    def consume_end_effector_pose(self) -> Pose3D | None:
        if self._executor is None:
            return None
        return self._executor.current_end_effector_pose()

    def boot(self) -> None:
        self.state.runtime_state = RobotRuntimeState.READY
        self.state.task_phase = HarvestTaskPhase.IDLE
        self.state.last_seen_phase = ScenePhase.READY
        self._reset_phase_counters()

    def observe_scene(self, snapshot: SceneSnapshot) -> None:
        self.state.last_scene_snapshot = snapshot
        self.state.last_seen_phase = snapshot.phase

    def apply_control(self, command: ControlCommand) -> None:
        if command is ControlCommand.START:
            self.state.runtime_state = RobotRuntimeState.RUNNING
            self.state.task_phase = HarvestTaskPhase.DETECTING
            self._reset_phase_counters()
            return

        if command is ControlCommand.STOP:
            self.state.runtime_state = RobotRuntimeState.STOPPED
            self.state.task_phase = HarvestTaskPhase.STOPPED
            self._reset_phase_counters()
            return

        if command is ControlCommand.RESET:
            self.state.runtime_state = RobotRuntimeState.READY
            self.state.task_phase = HarvestTaskPhase.IDLE
            self.state.last_target_estimate = None
            self.state.last_harvest_motion_plan = None
            self.state.last_motion_command = None
            self.state.last_phase_motion_plan = None
            self._reset_phase_counters()
            return

        raise ValueError(f"Unsupported control command: {command}")

    def step(self, bridge: BridgeProtocol) -> tuple[str, ...]:
        snapshot = self.state.last_scene_snapshot

        if self._executor is not None:
            joint_state = self._executor.current_joint_state_snapshot()
            if joint_state is not None:
                bridge.publish_joint_state(joint_state)

        if self.state.runtime_state is not RobotRuntimeState.RUNNING:
            # Hold current position via executor even before Start (warmup / drift prevention)
            if self._executor is not None and snapshot is not None:
                self._executor.run_cycle(replace(snapshot, active_phase_motion_plan=None))
            return ()

        if snapshot is None:
            return ()

        estimate = None
        if self.state.task_phase is HarvestTaskPhase.DETECTING:
            estimate = self._estimator.estimate(
                bridge.read_camera_frame("fixed_camera"),
                bridge.read_tf_tree(),
            )

        # 上位計画 (BehaviorPlanner) を先に実行し、今ティックの current_phase を確定する。
        # その後、下位計画 (MotionPlanner) が current_phase を参照して軌道を生成する。
        logs = self._behavior_planner.step(snapshot, bridge, estimate=estimate)

        if self.state.task_phase is HarvestTaskPhase.TARGET_FOUND and self.state.last_target_estimate is not None:
            motion_plan = self._planner.plan(
                self.state.last_target_estimate,
                bridge.read_joint_state(),
                bridge.read_tf_tree(),
                snapshot,
            )
            if motion_plan is not None:
                self.state.last_harvest_motion_plan = motion_plan
                self.state.planner_backend_name = motion_plan.planner_name
                self.state.task_phase = HarvestTaskPhase.PLANNING
                logs = logs + (
                    "[State] target_found -> planning",
                    f"[Planning] Planner backend={motion_plan.planner_name}",
                    "[Planning] MoveIt2-ready pre-grasp plan was created.",
                    (
                        "Pre-grasp world xyz: "
                        f"({motion_plan.pregrasp_pose.x:.4f}, {motion_plan.pregrasp_pose.y:.4f}, {motion_plan.pregrasp_pose.z:.4f})"
                        if motion_plan.pregrasp_pose is not None else "Pre-grasp world xyz: n/a"
                    ),
                    (
                        "Grasp world xyz: "
                        f"({motion_plan.grasp_pose.x:.4f}, {motion_plan.grasp_pose.y:.4f}, {motion_plan.grasp_pose.z:.4f})"
                        if motion_plan.grasp_pose is not None else "Grasp world xyz: n/a"
                    ),
                    (
                        "Pull world xyz: "
                        f"({motion_plan.pull_pose.x:.4f}, {motion_plan.pull_pose.y:.4f}, {motion_plan.pull_pose.z:.4f})"
                        if motion_plan.pull_pose is not None else "Pull world xyz: n/a"
                    ),
                )

        if self._executor is not None:
            effective_snapshot = replace(snapshot, active_phase_motion_plan=self.state.last_phase_motion_plan)
            executor_log = self._executor.run_cycle(effective_snapshot)
            if executor_log:
                logs = logs + (executor_log,)
            ctrl_state = self._executor.current_controller_state()
            if ctrl_state is not None:
                bridge.publish_controller_state(ctrl_state)
            reason = self._executor.consume_replan_request()
            if reason is not None:
                logs = logs + self.replan_active_motion(bridge, reason=reason)
            self._executor.log_post_update_debug_snapshot()

        return logs

    def replan_active_motion(self, bridge: BridgeProtocol, *, reason: str) -> tuple[str, ...]:
        if self.state.runtime_state is not RobotRuntimeState.RUNNING:
            return ()
        snapshot = self.state.last_scene_snapshot
        if snapshot is None:
            return ()
        if self.state.last_target_estimate is None:
            return ()

        current_joint_state = bridge.read_joint_state()
        tf_tree = bridge.read_tf_tree()

        # MOVING_TO_PLACE 中のリプランは place 軌道のみを再計画する。
        # フルチェーン（pregrasp→grasp→pull→place）を経由すると place 軌道の
        # 開始位置が end-of-pull になり、実際のロボット位置と乖離して即座に
        # path_tolerance_violation を引き起こすため。
        if (
            self.state.task_phase is HarvestTaskPhase.MOVING_TO_PLACE
            and self.state.last_harvest_motion_plan is not None
        ):
            plan_place_fn = getattr(self._planner, "plan_place_from_joint_state", None)
            if plan_place_fn is not None:
                motion_plan = plan_place_fn(
                    self.state.last_harvest_motion_plan,
                    current_joint_state,
                    tf_tree,
                    snapshot,
                )
                if motion_plan is not None:
                    return self._behavior_planner.replan(snapshot, bridge, reason=reason, motion_plan=motion_plan)

        motion_plan = self._planner.plan(
            self.state.last_target_estimate,
            current_joint_state,
            tf_tree,
            snapshot,
        )
        return self._behavior_planner.replan(snapshot, bridge, reason=reason, motion_plan=motion_plan)

    def _reset_phase_counters(self) -> None:
        self.state.grasp_evaluation_wait_steps = 0
        self.state.grasp_settle_wait_steps = 0
        self.state.phase_success_stable_steps = 0


RobotRuntime = HarvestRuntime
