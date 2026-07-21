from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/analysis/analyze_stem_break.py"
SPEC = spec_from_file_location("analyze_stem_break", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _row(seq: int, tension: float, status: str = "held") -> dict[str, str]:
    return {
        "seq": str(seq),
        "t": str(seq / 120.0),
        "status": status,
        "stemF": str(tension),
        "stem_d": str(seq * 0.001),
        "forceL": "8.0",
        "forceR": "8.5",
        "hold_slip": "0.001",
    }


def test_pull_summary_requires_target_break_and_post_break_hold() -> None:
    rows = [_row(seq, 8.0 if seq == 5 else 0.4) for seq in range(10)]
    events = [{"decision": "target_broken", "joint": "/World/TomatoStemJoint", "seq": "5"}]

    summary = MODULE.summarize_run(
        rows,
        events,
        break_force_n=7.5,
        expect_break=True,
        minimum_post_break_samples=3,
    )

    assert summary["target_break_count"] == 1
    assert summary["break_detected_pass"]
    assert summary["post_break_hold_pass"]
    assert summary["overall_pass"]


def test_non_pull_summary_passes_only_without_break_for_ten_seconds() -> None:
    rows = [_row(seq, 0.4) for seq in range(1201)]

    summary = MODULE.summarize_run(
        rows,
        [],
        break_force_n=7.5,
        expect_break=False,
        minimum_non_pull_samples=1200,
    )

    assert summary["observation_duration_sec"] >= 10.0
    assert summary["no_false_break_pass"]
    assert summary["overall_pass"]


def test_non_pull_duration_excludes_pre_grasp_samples() -> None:
    rows = [
        _row(seq, 0.4, status="attached") for seq in range(1000)
    ] + [
        _row(seq, 0.4, status="held") for seq in range(1000, 1600)
    ]

    summary = MODULE.summarize_run(
        rows,
        [],
        break_force_n=7.5,
        expect_break=False,
        minimum_non_pull_samples=1200,
    )

    assert not summary["no_false_break_pass"]
