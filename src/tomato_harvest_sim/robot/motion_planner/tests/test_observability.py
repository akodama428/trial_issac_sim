import json

from tomato_harvest_sim.robot.motion_planner.observability import metric_line


def test_metric_line_is_machine_readable_json_line() -> None:
    line = metric_line(
        "planner_completed",
        phase="target_found",
        trigger="target_found",
        latency_ms=12.5,
        success=True,
    )

    prefix, payload = line.split(" ", 1)
    assert prefix == "MOVEIT_METRIC"
    assert json.loads(payload) == {
        "event": "planner_completed",
        "latency_ms": 12.5,
        "phase": "target_found",
        "success": True,
        "trigger": "target_found",
    }


def test_metric_line_rejects_non_finite_numbers() -> None:
    try:
        metric_line("planner_completed", latency_ms=float("nan"))
    except ValueError:
        pass
    else:
        raise AssertionError("NaN must not be emitted in structured metrics")
