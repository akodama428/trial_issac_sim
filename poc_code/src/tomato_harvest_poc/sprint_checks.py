from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

from .config import MotionDurations, RuntimeConfig
from .model import ScenarioMode, SimulationStatus
from .render import render_camera_svg, render_viewport_svg
from .service import HarvestSimulationService


@dataclass(frozen=True)
class SprintCheckResult:
    sprint: str
    passed: bool
    details: tuple[str, ...]


def _wait_for_status(service: HarvestSimulationService, expected: SimulationStatus, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if service.get_snapshot().status == expected:
            return True
        time.sleep(0.01)
    return False


def _runtime_config_for_checks() -> RuntimeConfig:
    return RuntimeConfig(durations=MotionDurations(loading_s=0.01, approach_s=0.01, grasp_s=0.01, pull_s=0.01))


def run_sprint_checks(repo_root: Path) -> list[SprintCheckResult]:
    config = _runtime_config_for_checks()

    sprint1_files = (
        repo_root / "docker" / "Dockerfile",
        repo_root / "docker" / "entrypoint.sh",
        repo_root / "scripts" / "run_poc.py",
    )
    service = HarvestSimulationService(config)
    service.boot()
    ready = _wait_for_status(service, SimulationStatus.READY, timeout_s=1.0)
    sprint1 = SprintCheckResult(
        sprint="Sprint 1",
        passed=all(path.exists() for path in sprint1_files) and ready,
        details=(
            "docker build/run artifacts exist",
            "mock runtime reaches Ready",
        ),
    )

    snapshot = service.get_snapshot()
    sprint2 = SprintCheckResult(
        sprint="Sprint 2",
        passed=(
            "FrankaPanda" in "".join(snapshot.stage_items)
            and "Target Tomato" in render_camera_svg(snapshot)
            and "Target Tomato" in render_viewport_svg(snapshot, config)
        ),
        details=(
            "stage contains Franka, camera, and tomato items",
            "camera and viewport render target highlight",
        ),
    )

    service3 = HarvestSimulationService(config)
    service3.boot()
    ready3 = _wait_for_status(service3, SimulationStatus.READY, timeout_s=1.0)
    started3 = service3.start_harvest()
    detached3 = _wait_for_status(service3, SimulationStatus.DETACHED, timeout_s=1.0)
    sprint3 = SprintCheckResult(
        sprint="Sprint 3",
        passed=ready3 and started3 and detached3,
        details=(
            "Harvest Start accepted",
            "state machine reaches approach/grasp/pull path",
        ),
    )

    success_service = HarvestSimulationService(config, scenario=ScenarioMode.SUCCESS)
    success_service.boot()
    _wait_for_status(success_service, SimulationStatus.READY, timeout_s=1.0)
    success_service.start_harvest()
    success = _wait_for_status(success_service, SimulationStatus.DETACHED, timeout_s=1.0)

    failure_service = HarvestSimulationService(config, scenario=ScenarioMode.DETACH_FAILED)
    failure_service.boot()
    _wait_for_status(failure_service, SimulationStatus.READY, timeout_s=1.0)
    failure_service.start_harvest()
    failed = _wait_for_status(failure_service, SimulationStatus.FAILED, timeout_s=1.0)
    sprint4 = SprintCheckResult(
        sprint="Sprint 4",
        passed=success and failed,
        details=(
            "success and failure scenarios are reproducible",
            "detach result is exposed as final state",
        ),
    )

    service5 = HarvestSimulationService(config)
    service5.boot()
    _wait_for_status(service5, SimulationStatus.READY, timeout_s=1.0)
    attempts = 0
    for _ in range(3):
        if not service5.start_harvest():
            break
        if not _wait_for_status(service5, SimulationStatus.DETACHED, timeout_s=1.0):
            break
        if not service5.reset_scene():
            break
        attempts += 1
    help_text_present = "Confirm target" in service5.get_snapshot().instructions[0]
    sprint5 = SprintCheckResult(
        sprint="Sprint 5",
        passed=attempts == 3 and help_text_present,
        details=(
            "reset supports three repeated attempts",
            "beginner guidance text is present",
        ),
    )
    return [sprint1, sprint2, sprint3, sprint4, sprint5]
