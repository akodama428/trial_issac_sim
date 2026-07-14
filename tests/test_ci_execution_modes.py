from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = ROOT / ".github/workflows/ci.yml"


def _jobs() -> dict[str, object]:
    workflow = yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))
    return workflow["jobs"]


def test_default_servo_and_legacy_local_e2e_are_separate_jobs() -> None:
    jobs = _jobs()

    assert "unit-and-servo-e2e" in jobs
    assert "legacy-local-e2e" in jobs
    assert jobs["legacy-local-e2e"]["needs"] == "unit-and-servo-e2e"


def test_default_servo_job_does_not_enable_legacy_injections() -> None:
    servo_env = _jobs()["unit-and-servo-e2e"]["env"]

    assert "TOMATO_HARVEST_SERVO_MODE" not in servo_env
    assert "TOMATO_HARVEST_INJECT_LOCAL_PLAN_PHASES" not in servo_env
    assert "TOMATO_HARVEST_INJECT_SUFFIX_REPLAN_PHASES" not in servo_env


def test_legacy_job_explicitly_selects_off_mode_and_injections() -> None:
    legacy_env = _jobs()["legacy-local-e2e"]["env"]

    assert legacy_env["TOMATO_HARVEST_SERVO_MODE"] == "off"
    assert legacy_env["TOMATO_HARVEST_INJECT_LOCAL_PLAN_PHASES"]
    assert legacy_env["TOMATO_HARVEST_INJECT_SUFFIX_REPLAN_PHASES"]
