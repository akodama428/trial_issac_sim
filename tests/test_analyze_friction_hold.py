from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/analysis/analyze_friction_hold.py"
SPEC = spec_from_file_location("analyze_friction_hold", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_summary_requires_0_1m_lift_600_steps_and_less_than_5mm_slip() -> None:
    rows = [
        {
            "hold": "1",
            "hold_steps": str(step),
            "hold_slip": str(0.004 * step / 600),
            "status": "held",
            "stem_d": "0.113",
            "forceL": "12.0",
            "forceR": "11.0",
            "joint_count": "0",
            "fallback_count": "0",
            "teleport_count": "0",
        }
        for step in range(601)
    ]

    summary = MODULE.summarize_observations(rows)

    assert summary["lift_pass"]
    assert summary["duration_pass"]
    assert summary["slip_pass"]
    assert summary["hold_duration_sec"] == 5.0
