from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol

from tomato_harvest_sim.msg.contracts import ControlCommand, JointStateSnapshot, Pose3D, SceneSnapshot, TomatoStatus


class ControlPanelSystem(Protocol):
    def boot(self) -> None: ...

    def apply_control(self, command: ControlCommand) -> object: ...

    def step(self) -> tuple[str, ...]: ...

    def set_active_camera(self, camera_name: str) -> SceneSnapshot: ...

    def sync_robot_tool_pose(self, pose: Pose3D) -> SceneSnapshot: ...

    def sync_robot_joint_state(self, joint_state: JointStateSnapshot) -> None: ...

    def sync_tomato_physics(
        self,
        pose: Pose3D,
        *,
        attached: bool | None = None,
        status: TomatoStatus | None = None,
        reason: str | None = None,
    ) -> SceneSnapshot: ...

    def replan_motion(self, reason: str) -> tuple[str, ...]: ...

    @property
    def simulator(self) -> object: ...

    @property
    def robot(self) -> object: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class ControlPanelStatus:
    scene_phase: str
    robot_state: str
    task_phase: str
    active_camera: str
    tomato_status: str

    def summary_text(self) -> str:
        return (
            f"Scene: {self.scene_phase}\n"
            f"Robot: {self.robot_state}\n"
            f"Task: {self.task_phase}\n"
            f"Camera: {self.active_camera}\n"
            f"Tomato: {self.tomato_status}"
        )


@dataclass(frozen=True)
class ControlPanelLayoutSettings:
    title: str
    width: int
    height: int
    visible: bool
    dock_preference: str
    dock_target: str
    dock_policy: str


def load_control_panel_layout_settings() -> ControlPanelLayoutSettings:
    settings_path = Path(__file__).with_name("control_panel.layout.json")
    data = json.loads(settings_path.read_text(encoding="utf-8"))
    return ControlPanelLayoutSettings(
        title=str(data["title"]),
        width=int(data["width"]),
        height=int(data["height"]),
        visible=bool(data["visible"]),
        dock_preference=str(data["dock_preference"]),
        dock_target=str(data["dock_target"]),
        dock_policy=str(data["dock_policy"]),
    )


class ControlPanelController:
    PHYSICS_START_DELAY_FRAMES = 10

    def __init__(
        self,
        *,
        system: ControlPanelSystem,
        set_viewport_camera: Callable[[str], None],
        log_fn: Callable[[str], None],
    ) -> None:
        self._system = system
        self._set_viewport_camera = set_viewport_camera
        self._log = log_fn
        self._physics_grasp_enabled = bool(getattr(self._system.simulator, "_physics_grasp_enabled", False))
        self._pending_start_after_reset = False
        self._pending_start_delay_frames = 0
        self._pending_start_camera_name = "fixed_camera"

    def boot(self, *, initial_camera_name: str) -> ControlPanelStatus:
        self._system.boot()
        if initial_camera_name != "fixed_camera":
            self._system.set_active_camera(initial_camera_name)
        self._set_viewport_camera(initial_camera_name)
        status = self.status()
        self._log("[Ready] Ready. Use Start / Stop / Reset from Tomato Harvest Controls.")
        return status

    def start(self) -> ControlPanelStatus:
        if self._physics_grasp_enabled:
            current_status = self.status()
            self._pending_start_camera_name = current_status.active_camera
            reset_result = self._system.apply_control(ControlCommand.RESET)
            self._pending_start_after_reset = True
            self._pending_start_delay_frames = self.PHYSICS_START_DELAY_FRAMES
            if self._pending_start_camera_name != "fixed_camera":
                self._system.set_active_camera(self._pending_start_camera_name)
                self._set_viewport_camera(self._pending_start_camera_name)
            status = self.status()
            self._log(
                f"[StartPrep] accepted={getattr(reset_result, 'accepted', True)} "
                f"scene={status.scene_phase} robot={status.robot_state} "
                "mode=reset_then_start"
            )
            return status
        result = self._system.apply_control(ControlCommand.START)
        status = self.status()
        self._log(
            f"[Start] accepted={getattr(result, 'accepted', True)} "
            f"scene={status.scene_phase} robot={status.robot_state}"
        )
        return status

    def stop(self) -> ControlPanelStatus:
        self._pending_start_after_reset = False
        self._pending_start_delay_frames = 0
        result = self._system.apply_control(ControlCommand.STOP)
        status = self.status()
        self._log(
            f"[Stop] accepted={getattr(result, 'accepted', True)} "
            f"scene={status.scene_phase} robot={status.robot_state}"
        )
        return status

    def reset(self) -> ControlPanelStatus:
        self._pending_start_after_reset = False
        self._pending_start_delay_frames = 0
        result = self._system.apply_control(ControlCommand.RESET)
        self._set_viewport_camera("fixed_camera")
        status = self.status()
        self._log(
            f"[Reset] accepted={getattr(result, 'accepted', True)} "
            f"scene={status.scene_phase} robot={status.robot_state}"
        )
        return status

    def select_camera(self, camera_name: str) -> ControlPanelStatus:
        snapshot = self._system.set_active_camera(camera_name)
        self._set_viewport_camera(camera_name)
        status = self.status()
        self._log(f"[Camera] active={snapshot.active_camera}")
        return status

    def status(self) -> ControlPanelStatus:
        simulator_state = self._system.simulator.state
        robot_state = self._system.robot.state
        return ControlPanelStatus(
            scene_phase=simulator_state.phase.value,
            robot_state=robot_state.runtime_state.value,
            task_phase=robot_state.task_phase.value,
            active_camera=simulator_state.active_camera,
            tomato_status=simulator_state.tomato_status.value,
        )

    def step_runtime(self) -> ControlPanelStatus:
        if self._pending_start_after_reset:
            if self._pending_start_delay_frames > 0:
                self._pending_start_delay_frames -= 1
                return self.status()
            result = self._system.apply_control(ControlCommand.START)
            self._pending_start_after_reset = False
            status = self.status()
            self._log(
                f"[Start] accepted={getattr(result, 'accepted', True)} "
                f"scene={status.scene_phase} robot={status.robot_state}"
            )
        for message in self._system.step():
            self._log(message)
        return self.status()

    def current_scene_snapshot(self) -> SceneSnapshot:
        return self._system.simulator.snapshot()

    def current_robot_state(self) -> object:
        return self._system.robot.state

    def sync_robot_tool_pose(self, pose: Pose3D) -> ControlPanelStatus:
        self._system.sync_robot_tool_pose(pose)
        return self.status()

    def sync_robot_joint_state(self, joint_state: JointStateSnapshot) -> ControlPanelStatus:
        self._system.sync_robot_joint_state(joint_state)
        return self.status()

    def sync_tomato_physics(
        self,
        pose: Pose3D,
        *,
        attached: bool | None = None,
        status: TomatoStatus | None = None,
        reason: str | None = None,
    ) -> ControlPanelStatus:
        self._system.sync_tomato_physics(
            pose,
            attached=attached,
            status=status,
            reason=reason,
        )
        return self.status()

    def close(self) -> None:
        self._system.close()

    def request_motion_replan(self, reason: str) -> None:
        for message in self._system.replan_motion(reason):
            self._log(message)


class IsaacControlPanelWindow:
    def __init__(self, controller: ControlPanelController) -> None:
        import omni.ui as ui

        self._controller = controller
        self._ui = ui
        self._layout = load_control_panel_layout_settings()
        self._window = ui.Window(
            self._layout.title,
            width=self._layout.width,
            height=self._layout.height,
            visible=self._layout.visible,
            dockPreference=getattr(ui.DockPreference, self._layout.dock_preference),
        )
        self._window.deferred_dock_in(
            self._layout.dock_target,
            getattr(ui.DockPolicy, self._layout.dock_policy),
        )
        self._status_label: object | None = None
        self._build()
        self._refresh(controller.status())

    def _build(self) -> None:
        ui = self._ui

        with self._window.frame:
            with ui.VStack(spacing=8, height=0):
                ui.Label("Scenario Controls", height=24)
                self._status_label = ui.Label("", height=88)
                with ui.HStack(height=32, spacing=6):
                    ui.Button("Start", clicked_fn=lambda: self._run_and_refresh(self._controller.start))
                    ui.Button("Stop", clicked_fn=lambda: self._run_and_refresh(self._controller.stop))
                    ui.Button("Reset", clicked_fn=lambda: self._run_and_refresh(self._controller.reset))
                ui.Spacer(height=6)
                ui.Label("Camera", height=20)
                with ui.HStack(height=32, spacing=6):
                    ui.Button(
                        "Fixed Camera",
                        clicked_fn=lambda: self._run_and_refresh(
                            lambda: self._controller.select_camera("fixed_camera")
                        ),
                    )
                    ui.Button(
                        "Hand Camera",
                        clicked_fn=lambda: self._run_and_refresh(
                            lambda: self._controller.select_camera("hand_camera")
                        ),
                    )

    def _run_and_refresh(self, action: Callable[[], ControlPanelStatus]) -> None:
        status = action()
        self._refresh(status)

    def _refresh(self, status: ControlPanelStatus) -> None:
        if self._status_label is not None:
            self._status_label.text = status.summary_text()

    def refresh_status(self, status: ControlPanelStatus) -> None:
        self._refresh(status)
