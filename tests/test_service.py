from __future__ import annotations

import time
import unittest

from tomato_harvest_poc.config import MotionDurations, RuntimeConfig
from tomato_harvest_poc.model import FailureReason, ScenarioMode, SimulationStatus
from tomato_harvest_poc.service import HarvestSimulationService


def make_config() -> RuntimeConfig:
    return RuntimeConfig(durations=MotionDurations(loading_s=0.01, approach_s=0.01, grasp_s=0.01, pull_s=0.01))


def wait_until(service: HarvestSimulationService, status: SimulationStatus) -> bool:
    deadline = time.time() + 1.0
    while time.time() < deadline:
        if service.get_snapshot().status == status:
            return True
        time.sleep(0.01)
    return False


class HarvestSimulationServiceTest(unittest.TestCase):
    def test_boot_reaches_ready(self) -> None:
        service = HarvestSimulationService(make_config())
        service.boot()
        self.assertTrue(wait_until(service, SimulationStatus.READY))

    def test_successful_harvest_reaches_detached(self) -> None:
        service = HarvestSimulationService(make_config(), scenario=ScenarioMode.SUCCESS)
        service.boot()
        self.assertTrue(wait_until(service, SimulationStatus.READY))
        self.assertTrue(service.start_harvest())
        self.assertTrue(wait_until(service, SimulationStatus.DETACHED))
        snapshot = service.get_snapshot()
        self.assertEqual(snapshot.result_message, "Harvest Succeeded")
        self.assertTrue(snapshot.visual.tomato_detached)

    def test_failed_harvest_returns_reason(self) -> None:
        service = HarvestSimulationService(make_config(), scenario=ScenarioMode.DETACH_FAILED)
        service.boot()
        self.assertTrue(wait_until(service, SimulationStatus.READY))
        self.assertTrue(service.start_harvest())
        self.assertTrue(wait_until(service, SimulationStatus.FAILED))
        snapshot = service.get_snapshot()
        self.assertEqual(snapshot.failure_reason, FailureReason.DETACH_FAILED)

    def test_reset_returns_to_ready(self) -> None:
        service = HarvestSimulationService(make_config(), scenario=ScenarioMode.SUCCESS)
        service.boot()
        self.assertTrue(wait_until(service, SimulationStatus.READY))
        self.assertTrue(service.start_harvest())
        self.assertTrue(wait_until(service, SimulationStatus.DETACHED))
        self.assertTrue(service.reset_scene())
        self.assertEqual(service.get_snapshot().status, SimulationStatus.READY)

