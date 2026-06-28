from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from tomato_harvest_sim.api.bridge import BridgeProtocol, Ros2LoopbackBridge, create_bridge
from tomato_harvest_sim.api.contracts import (
    ControlCommand,
    ControlResult,
    JointStateSnapshot,
    Pose3D,
    SceneSnapshot,
    TomatoStatus,
)
from tomato_harvest_sim.robot.motion_planner import MoveItServiceManager
from tomato_harvest_sim.robot.runtime import RobotRuntime
from tomato_harvest_sim.simulator.scene_runtime import IsaacSceneRuntime

if TYPE_CHECKING:
    from tomato_harvest_sim.robot.trajectory_tracking import TrajectoryTrackingCoordinator


@dataclass
class TomatoHarvestApplication:
    scene_runtime: IsaacSceneRuntime
    robot: RobotRuntime
    bridge: BridgeProtocol
    moveit_service: MoveItServiceManager | None = None
    executor: TrajectoryTrackingCoordinator | None = None

    @property
    def simulator(self) -> IsaacSceneRuntime:
        return self.scene_runtime

    def boot(self) -> None:
        snapshot = self.scene_runtime.boot()
        self.robot.boot()
        self.bridge.publish_scene_snapshot(snapshot)
        self.robot.observe_scene(self.bridge.read_scene_snapshot())

    def set_active_camera(self, camera_name: str) -> SceneSnapshot:
        snapshot = self.scene_runtime.set_active_camera(camera_name)
        self.bridge.publish_scene_snapshot(snapshot)
        transported = self.bridge.read_scene_snapshot()
        self.robot.observe_scene(transported)
        return transported

    def sync_robot_tool_pose(self, pose: object) -> SceneSnapshot:
        snapshot = self.simulator.sync_robot_tool_pose(pose)
        self.bridge.publish_scene_snapshot(snapshot)
        transported = self.bridge.read_scene_snapshot()
        self.robot.observe_scene(transported)
        return transported

    def sync_robot_joint_state(self, joint_state: JointStateSnapshot) -> None:
        self.bridge.publish_joint_state(joint_state)

    def sync_tomato_physics(
        self,
        pose: Pose3D,
        *,
        attached: bool | None = None,
        status: TomatoStatus | None = None,
        reason: str | None = None,
    ) -> SceneSnapshot:
        snapshot = self.scene_runtime.sync_tomato_physics(
            pose,
            attached=attached,
            status=status,
            reason=reason,
        )
        self.bridge.publish_scene_snapshot(snapshot)
        transported = self.bridge.read_scene_snapshot()
        self.robot.observe_scene(transported)
        return transported

    def apply_control(self, command: ControlCommand) -> ControlResult:
        self.bridge.publish_control(command)
        transport_command = self.bridge.consume_control_command() or command
        snapshot = self.scene_runtime.apply_control(transport_command)
        self.bridge.publish_scene_snapshot(snapshot)
        self.robot.apply_control(transport_command)
        self.robot.observe_scene(self.bridge.read_scene_snapshot())
        return ControlResult(
            command=transport_command,
            accepted=True,
            scene_phase=self.scene_runtime.state.phase,
            robot_state=self.robot.state.runtime_state,
        )

    def step(self) -> tuple[str, ...]:
        self.bridge.spin_once()

        # ① joint state sync: executor → bridge
        if self.executor is not None:
            joint_state = self.executor.current_joint_state_snapshot()
            if joint_state is not None:
                self.bridge.publish_joint_state(joint_state)

        # ② perception / behavior / motion planning
        logs = self.robot.step(self.bridge)

        motion_command = self.bridge.consume_motion_command()
        if motion_command is not None:
            snapshot = self.scene_runtime.apply_motion_command(motion_command)
            self.bridge.publish_scene_snapshot(snapshot)
            self.robot.observe_scene(self.bridge.read_scene_snapshot())

        snapshot = self.scene_runtime.advance()
        self.bridge.publish_scene_snapshot(snapshot)
        self.robot.observe_scene(self.bridge.read_scene_snapshot())

        # ④ trajectory tracking + ros2_control (snapshot is post-advance)
        if self.executor is not None:
            executor_log = self.executor.run_cycle(self.scene_runtime.snapshot())
            if executor_log:
                logs = logs + (executor_log,)
            reason = self.executor.consume_replan_request()
            if reason is not None:
                logs = logs + self.replan_motion(reason)

            # ⑤ end effector sync: executor → scene
            pose = self.executor.current_end_effector_pose()
            if pose is not None:
                self.sync_robot_tool_pose(pose)

        return logs

    def replan_motion(self, reason: str) -> tuple[str, ...]:
        logs = self.robot.replan_active_motion(self.bridge, reason=reason)
        motion_command = self.bridge.consume_motion_command()
        if motion_command is not None:
            snapshot = self.scene_runtime.apply_motion_command(motion_command)
            self.bridge.publish_scene_snapshot(snapshot)
            self.robot.observe_scene(self.bridge.read_scene_snapshot())
        return logs

    def close(self) -> None:
        self.bridge.close()
        if self.moveit_service is not None:
            self.moveit_service.shutdown()


def create_tomato_harvest_application(
    *,
    grasp_mode: str = "success",
    physics_grasp_enabled: bool = False,
    physics_soft_fallback_enabled: bool = False,
    transport: str | None = None,
    autostart_moveit_service: bool = True,
    executor: TrajectoryTrackingCoordinator | None = None,
) -> TomatoHarvestApplication:
    grasp_lateral_offset_m = 0.0 if grasp_mode == "success" else 0.08
    bridge = create_bridge(transport=transport)
    moveit_service = None
    if autostart_moveit_service and isinstance(bridge, Ros2LoopbackBridge):
        moveit_service = MoveItServiceManager.start_if_needed()
    return TomatoHarvestApplication(
        scene_runtime=IsaacSceneRuntime(
            physics_grasp_enabled=physics_grasp_enabled,
            physics_soft_fallback_enabled=physics_soft_fallback_enabled,
        ),
        robot=RobotRuntime(grasp_lateral_offset_m=grasp_lateral_offset_m),
        bridge=bridge,
        moveit_service=moveit_service,
        executor=executor,
    )
