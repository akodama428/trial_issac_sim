from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = ROOT / ".github/workflows/ci.yml"
IN_CONTAINER_E2E = ROOT / "scripts/ci/in_container_e2e.sh"
RUN_E2E = ROOT / "scripts/ci/run_e2e.sh"


def _jobs() -> dict[str, object]:
    workflow = yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))
    return workflow["jobs"]


def test_ci_has_only_unit_and_servo_e2e_job() -> None:
    jobs = _jobs()

    assert set(jobs) == {"unit-and-servo-e2e"}


def test_default_servo_job_does_not_enable_legacy_injections() -> None:
    servo_env = _jobs()["unit-and-servo-e2e"]["env"]

    assert "TOMATO_HARVEST_SERVO_MODE" not in servo_env
    assert "TOMATO_HARVEST_INJECT_LOCAL_PLAN_PHASES" not in servo_env
    assert "TOMATO_HARVEST_INJECT_SUFFIX_REPLAN_PHASES" not in servo_env


def test_ci_workflow_contains_no_legacy_mode_or_injections() -> None:
    source = WORKFLOW.read_text(encoding="utf-8")

    assert "legacy-local-e2e" not in source
    assert "TOMATO_HARVEST_SERVO_MODE" not in source
    assert "TOMATO_HARVEST_INJECT_LOCAL_PLAN_PHASES" not in source


def test_e2e_rebuilds_ros_components_before_launch() -> None:
    source = IN_CONTAINER_E2E.read_text(encoding="utf-8")

    assert "--rebuild" in source


def test_e2e_container_isolated_on_ros_domain_99() -> None:
    source = RUN_E2E.read_text(encoding="utf-8")

    assert "-e ROS_DOMAIN_ID=99" in source


def test_ci_e2e_defaults_to_physics_grasp_mode() -> None:
    launcher_source = RUN_E2E.read_text(encoding="utf-8")
    container_source = IN_CONTAINER_E2E.read_text(encoding="utf-8")

    assert 'CI_GRASP_MODE="${CI_GRASP_MODE:-physics}"' in launcher_source
    assert '--grasp-mode "${CI_GRASP_MODE:-physics}"' in container_source


def test_optional_e2e_rosbag_starts_before_robot_stack() -> None:
    launcher_source = RUN_E2E.read_text(encoding="utf-8")
    container_source = IN_CONTAINER_E2E.read_text(encoding="utf-8")

    assert 'CI_RECORD_HOME_DIVERGENCE_BAG="${CI_RECORD_HOME_DIVERGENCE_BAG:-}"' in (
        launcher_source
    )
    record_index = container_source.index("ros2 bag record")
    stack_index = container_source.index("./scripts/run_ros2_components.sh")
    assert record_index < stack_index
    assert "/tomato_harvest/phase" in container_source
    assert "/joint_trajectory_controller/joint_trajectory" in container_source
    assert "/joint_trajectory_controller/controller_state" in container_source
    assert 'kill -TERM "${BAG_PID}"' in container_source


def test_friction_hold_evaluation_environment_is_forwarded() -> None:
    launcher_source = RUN_E2E.read_text(encoding="utf-8")

    assert "TOMATO_HARVEST_FRICTION_HOLD_EVAL_STEPS" in launcher_source
