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
        'MOVEIT_METRIC {"event":"trajectory_replaced","phase":"moving_to_place"}\n',
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
    assert summary["phase_abort"] == {
        "moving_to_place": {"started": 1, "aborted": 0, "abort_rate": 0.0},
        "moving_to_pregrasp": {"started": 1, "aborted": 1, "abort_rate": 1.0},
    }


def test_read_events_skips_malformed_metric_lines(tmp_path: Path) -> None:
    log = tmp_path / "e2e.log"
    log.write_text("MOVEIT_METRIC not-json\n", encoding="utf-8")

    assert MODULE.read_events([log]) == []
