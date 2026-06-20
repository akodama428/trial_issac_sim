from __future__ import annotations

import threading
import time
from dataclasses import replace

from .config import RuntimeConfig
from .model import FailureReason, LogEntry, ScenarioMode, SimulationStatus, Snapshot, VisualState


class HarvestSimulationService:
    def __init__(
        self,
        config: RuntimeConfig,
        scenario: ScenarioMode = ScenarioMode.SUCCESS,
    ) -> None:
        self._config = config
        self._scenario = scenario
        self._lock = threading.Lock()
        self._busy = False
        self._snapshot = Snapshot(
            status=SimulationStatus.IDLE,
            result_message="Simulator not started",
            target_label=config.scene.target_label,
            stage_items=config.scene.stage_items,
        )

    def boot(self) -> None:
        with self._lock:
            self._snapshot = self._replace_snapshot(
                status=SimulationStatus.LOADING,
                result_message="Loading assets...",
                logs=(LogEntry("boot", "Initializing ROS 2 bridge..."),),
                visual=VisualState(),
            )
        worker = threading.Thread(target=self._complete_boot, daemon=True)
        worker.start()

    def get_snapshot(self) -> Snapshot:
        with self._lock:
            return self._snapshot

    def start_harvest(self) -> bool:
        with self._lock:
            if self._busy or self._snapshot.status != SimulationStatus.READY:
                return False
            self._busy = True
        worker = threading.Thread(target=self._run_harvest_flow, daemon=True)
        worker.start()
        return True

    def reset_scene(self) -> bool:
        with self._lock:
            if self._busy:
                return False
            attempts = self._snapshot.visual.attempts_completed
            self._snapshot = self._replace_snapshot(
                status=SimulationStatus.READY,
                result_message="Simulation ready",
                failure_reason=None,
                logs=(
                    LogEntry("reset", "Scene reset complete"),
                ),
                visual=VisualState(attempts_completed=attempts),
            )
        return True

    def _complete_boot(self) -> None:
        time.sleep(self._config.durations.loading_s)
        with self._lock:
            self._snapshot = self._replace_snapshot(
                status=SimulationStatus.READY,
                result_message="Simulation ready",
                logs=(
                    LogEntry("boot", "Loading assets..."),
                    LogEntry("boot", "Initializing ROS 2 bridge..."),
                    LogEntry("boot", "Simulation ready"),
                ),
            )

    def _run_harvest_flow(self) -> None:
        self._transition(
            SimulationStatus.APPROACHING,
            "Harvest request accepted",
            arm_progress=0.35,
            log_message="Approaching target",
        )
        time.sleep(self._config.durations.approach_s)
        if self._scenario == ScenarioMode.TARGET_LOST:
            self._fail(FailureReason.TARGET_LOST)
            return

        self._transition(
            SimulationStatus.GRASPING,
            "Closing gripper",
            arm_progress=0.65,
            gripper_closed=True,
            log_message="Closing gripper",
        )
        time.sleep(self._config.durations.grasp_s)
        if self._scenario == ScenarioMode.GRASP_FAILED:
            self._fail(FailureReason.GRASP_FAILED)
            return

        self._transition(
            SimulationStatus.PULLING,
            "Pulling",
            arm_progress=1.0,
            gripper_closed=True,
            log_message="Pulling",
        )
        time.sleep(self._config.durations.pull_s)

        if self._scenario == ScenarioMode.DETACH_FAILED or (
            self._config.pull_force < self._config.detach_break_force
        ):
            self._fail(FailureReason.DETACH_FAILED)
            return

        with self._lock:
            attempts = self._snapshot.visual.attempts_completed + 1
            self._snapshot = self._replace_snapshot(
                status=SimulationStatus.DETACHED,
                result_message="Harvest Succeeded",
                failure_reason=None,
                logs=self._snapshot.logs + (LogEntry("result", "Detached"),),
                visual=VisualState(
                    target_highlighted=True,
                    tomato_detached=True,
                    gripper_closed=True,
                    arm_progress=1.0,
                    attempts_completed=attempts,
                ),
            )
            self._busy = False

    def _fail(self, reason: FailureReason) -> None:
        with self._lock:
            attempts = self._snapshot.visual.attempts_completed + 1
            self._snapshot = self._replace_snapshot(
                status=SimulationStatus.FAILED,
                result_message=reason.value,
                failure_reason=reason,
                logs=self._snapshot.logs + (LogEntry("result", reason.value),),
                visual=VisualState(
                    target_highlighted=True,
                    tomato_detached=False,
                    gripper_closed=self._snapshot.visual.gripper_closed,
                    arm_progress=self._snapshot.visual.arm_progress,
                    attempts_completed=attempts,
                ),
            )
            self._busy = False

    def _transition(
        self,
        status: SimulationStatus,
        message: str,
        *,
        arm_progress: float,
        gripper_closed: bool = False,
        log_message: str,
    ) -> None:
        with self._lock:
            self._snapshot = self._replace_snapshot(
                status=status,
                result_message=message,
                logs=self._snapshot.logs + (LogEntry(status.value.lower(), log_message),),
                visual=VisualState(
                    target_highlighted=True,
                    tomato_detached=False,
                    gripper_closed=gripper_closed,
                    arm_progress=arm_progress,
                    attempts_completed=self._snapshot.visual.attempts_completed,
                ),
            )

    def _replace_snapshot(self, **kwargs: object) -> Snapshot:
        return replace(self._snapshot, **kwargs)
