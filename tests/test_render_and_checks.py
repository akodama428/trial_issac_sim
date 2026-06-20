from __future__ import annotations

import unittest
from pathlib import Path

from tomato_harvest_poc.config import RuntimeConfig
from tomato_harvest_poc.render import render_camera_svg, render_viewport_svg
from tomato_harvest_poc.service import HarvestSimulationService
from tomato_harvest_poc.sprint_checks import run_sprint_checks


class RenderAndChecksTest(unittest.TestCase):
    def test_render_contains_target_label(self) -> None:
        config = RuntimeConfig()
        service = HarvestSimulationService(config)
        service.boot()
        snapshot = service.get_snapshot()
        self.assertIn("Target Tomato", render_camera_svg(snapshot))
        self.assertIn("Target Tomato", render_viewport_svg(snapshot, config))

    def test_sprint_checks_all_pass(self) -> None:
        repo_root = Path(__file__).resolve().parents[1]
        results = run_sprint_checks(repo_root)
        self.assertTrue(all(result.passed for result in results))
