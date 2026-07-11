import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "summarize_moveit_metrics.py"
SPEC = importlib.util.spec_from_file_location("summarize_moveit_metrics", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_summary_calculates_required_step0_metrics(tmp_path: Path) -> None:
    log = tmp_path / "e2e.log"
    log.write_text(
        "noise\n"
        'MOVEIT_METRIC {"event":"planner_completed","phase":"target_found","latency_ms":10.0,"success":true}\n'
        'MOVEIT_METRIC {"event":"planner_completed","phase":"moving_to_place","latency_ms":30.0,"success":false}\n'
        'MOVEIT_METRIC {"event":"trajectory_started","phase":"moving_to_pregrasp"}\n'
        'MOVEIT_METRIC {"event":"trajectory_aborted","phase":"moving_to_pregrasp"}\n'
        'MOVEIT_METRIC {"event":"trajectory_started","phase":"moving_to_place"}\n'
        'MOVEIT_METRIC {"event":"trajectory_cancel_requested","phase":"moving_to_place"}\n'
        'MOVEIT_METRIC {"event":"trajectory_replaced","phase":"moving_to_place"}\n'
        'MOVEIT_METRIC {"event":"suffix_replan_completed","phase":"moving_to_place","success":true,"latency_ms":42.5}\n'
        'MOVEIT_METRIC {"event":"suffix_replan_completed","phase":"moving_to_pregrasp","success":true,"latency_ms":30.0}\n'
        'MOVEIT_METRIC {"event":"suffix_replan_completed","phase":"moving_to_pregrasp","success":true,"latency_ms":50.0}\n'
        'MOVEIT_METRIC {"event":"suffix_replan_completed","phase":"moving_to_grasp","success":false,"latency_ms":99.0}\n'
        'MOVEIT_METRIC {"event":"plan_adopted","producer_kind":"global_planner","reason":"adopted_initial"}\n'
        'MOVEIT_METRIC {"event":"plan_adopted","producer_kind":"global_planner","reason":"adopted_newer_revision"}\n'
        'MOVEIT_METRIC {"event":"plan_adopted","producer_kind":"local_planner","reason":"adopted_newer_producer_instance"}\n'
        'MOVEIT_METRIC {"event":"plan_rejected","producer_kind":"local_planner","reason":"rejected_local_without_adopted_plan"}\n',
        encoding="utf-8",
    )

    summary = MODULE.summarize(MODULE.read_events([log]))

    assert summary["planner_latency_ms"] == {
        "count": 2,
        "mean": 20.0,
        "min": 10.0,
        "max": 30.0,
    }
    assert summary["cancel_count"] == 1
    assert summary["trajectory_replacement_count"] == 1
    assert summary["suffix_replan"] == {
        "moving_to_pregrasp": {
            "successful_count": 2,
            "latency_ms": {"mean": 40.0, "min": 30.0, "max": 50.0},
        },
        "moving_to_place": {
            "successful_count": 1,
            "latency_ms": {"mean": 42.5, "min": 42.5, "max": 42.5},
        },
    }
    assert summary["phase_abort"] == {
        "moving_to_place": {"started": 1, "aborted": 0, "abort_rate": 0.0},
        "moving_to_pregrasp": {"started": 1, "aborted": 1, "abort_rate": 1.0},
    }
    assert summary["plan_adoption"] == {
        "global_planner": {"adopted": 2, "rejected": 0},
        "local_planner": {"adopted": 1, "rejected": 1},
    }


def test_read_events_skips_malformed_metric_lines(tmp_path: Path) -> None:
    log = tmp_path / "e2e.log"
    log.write_text("MOVEIT_METRIC not-json\n", encoding="utf-8")

    assert MODULE.read_events([log]) == []
