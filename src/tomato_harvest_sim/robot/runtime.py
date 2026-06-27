from __future__ import annotations

from dataclasses import dataclass

from tomato_harvest_sim.api.bridge import BridgeProtocol
from tomato_harvest_sim.api.contracts import (
    ControlCommand,
    HarvestMotionPlan,
    HarvestTaskPhase,
    MotionCommand,
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

    def __init__(self, *, grasp_lateral_offset_m: float = 0.0) -> None:
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
        self._behavior_planner = BehaviorPlanner(
            state=self.state,
            estimator=TomatoTargetEstimator(),
            planner=planner,
            motion_publisher=MoveItStyleMotionPublisher(),
        )

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
        if self.state.runtime_state is not RobotRuntimeState.RUNNING:
            return ()
        snapshot = self.state.last_scene_snapshot
        if snapshot is None:
            return ()
        return self._behavior_planner.step(snapshot, bridge)

    def replan_active_motion(self, bridge: BridgeProtocol, *, reason: str) -> tuple[str, ...]:
        if self.state.runtime_state is not RobotRuntimeState.RUNNING:
            return ()
        snapshot = self.state.last_scene_snapshot
        if snapshot is None:
            return ()
        return self._behavior_planner.replan(snapshot, bridge, reason=reason)

    def _reset_phase_counters(self) -> None:
        self.state.grasp_evaluation_wait_steps = 0
        self.state.grasp_settle_wait_steps = 0
        self.state.phase_success_stable_steps = 0


RobotRuntime = HarvestRuntime
