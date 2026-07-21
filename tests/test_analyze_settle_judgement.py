from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/analysis/analyze_settle_judgement.py"
SPEC = spec_from_file_location("analyze_settle_judgement", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _row(seq: int, event: str, decision: str, reason: str) -> dict[str, str]:
    return {
        "cycle": "1", "seq": str(seq), "event": event,
        "decision": decision, "reason": reason, "elapsed": str(seq / 120),
        "x": "0.35", "y": "-0.35", "z": "0.46",
        "local_x": "0.0", "local_y": "0.0", "local_z": "0.01",
        "margin_x": "0.10", "margin_y": "0.07", "speed": "0.01",
        "angular_speed": "1.2", "contact": "1", "contact_seen": "1",
        "settle": str(seq),
    }


def test_summary_calculates_placed_event_latency() -> None:
    rows = [
        _row(0, "release_started", "pending", "release_started"),
        _row(12, "first_tray_contact", "pending", "settling"),
        _row(24, "terminal", "placed", "settled_in_tray"),
    ]

    summary = MODULE.summarize_run(rows, expected_decision="placed")

    assert summary["release_to_contact_sec"] == 0.1
    assert summary["contact_to_terminal_sec"] == 0.1
    assert summary["release_to_terminal_sec"] == 0.2
    assert summary["overall_pass"]


def test_parser_rejects_cycle_without_release_marker(tmp_path: Path) -> None:
    log = tmp_path / "fallen.log"
    row = _row(2, "terminal", "failed", "escaped_tray")
    payload = " ".join(f"{key}={value}" for key, value in row.items())
    log.write_text(f"[PlacementObs] {payload}\n", encoding="utf-8")

    rows = MODULE.parse_log(log)
    summary = MODULE.summarize_run(rows, expected_decision="failed")

    assert not summary["release_marker_pass"]
    assert not summary["overall_pass"]


def test_split_cycles_keeps_independent_runs() -> None:
    first = _row(0, "release_started", "pending", "release_started")
    second = _row(0, "release_started", "pending", "release_started")
    second["cycle"] = "2"

    cycles = MODULE.split_cycles([first, second])

    assert len(cycles) == 2
    assert cycles[0][0]["cycle"] == "1"
    assert cycles[1][0]["cycle"] == "2"


def test_load_runs_does_not_merge_reused_cycle_ids(tmp_path: Path) -> None:
    paths = []
    for index in range(2):
        path = tmp_path / f"run-{index}.log"
        row = _row(index, "release_started", "pending", "release_started")
        payload = " ".join(f"{key}={value}" for key, value in row.items())
        path.write_text(f"[PlacementObs] {payload}\n", encoding="utf-8")
        paths.append(path)

    assert len(MODULE.load_runs(paths)) == 2
