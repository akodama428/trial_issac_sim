#!/usr/bin/env python3
"""MoveIt2 Step 0 の構造化ログを集計し、JSON・CSV・グラフを生成する。"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import Counter
from pathlib import Path
from typing import Iterable

PREFIX = "MOVEIT_METRIC "


def read_events(log_paths: Iterable[Path]) -> list[dict[str, object]]:
    """複数ログから妥当な MoveIt metric event だけを読み取る。"""
    events: list[dict[str, object]] = []
    for path in log_paths:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            marker = line.find(PREFIX)
            if marker < 0:
                continue
            try:
                payload = json.loads(line[marker + len(PREFIX):])
            except json.JSONDecodeError:
                continue
            if isinstance(payload, dict) and isinstance(payload.get("event"), str):
                events.append(payload)
    return events


def summarize(events: Iterable[dict[str, object]]) -> dict[str, object]:
    """Issue #8 が要求する4指標をイベント列から集計する。"""
    event_list = list(events)
    latencies = [
        float(event["latency_ms"])
        for event in event_list
        if event["event"] == "planner_completed"
        and isinstance(event.get("latency_ms"), (int, float))
    ]
    started = Counter(
        str(event["phase"])
        for event in event_list
        if event["event"] == "trajectory_started" and event.get("phase") is not None
    )
    aborted = Counter(
        str(event["phase"])
        for event in event_list
        if event["event"] == "trajectory_aborted" and event.get("phase") is not None
    )
    phases = sorted(started.keys() | aborted.keys())
    latency_summary = {
        "count": len(latencies),
        "mean": statistics.fmean(latencies) if latencies else None,
        "min": min(latencies) if latencies else None,
        "max": max(latencies) if latencies else None,
    }
    suffix_latencies_by_phase: dict[str, list[float]] = {}
    for event in event_list:
        if (
            event["event"] == "suffix_replan_completed"
            and event.get("success") is True
            and isinstance(event.get("latency_ms"), (int, float))
        ):
            phase = str(event.get("phase", "unknown"))
            suffix_latencies_by_phase.setdefault(phase, []).append(
                float(event["latency_ms"])
            )
    return {
        "planner_latency_ms": latency_summary,
        "cancel_count": sum(event["event"] == "trajectory_cancel_requested" for event in event_list),
        "trajectory_replacement_count": sum(
            event["event"] == "trajectory_replaced" for event in event_list
        ),
        "suffix_replan": {
            phase: {
                "successful_count": len(latencies),
                "latency_ms": {
                    "mean": statistics.fmean(latencies),
                    "min": min(latencies),
                    "max": max(latencies),
                },
            }
            for phase, latencies in sorted(suffix_latencies_by_phase.items())
        },
        "phase_abort": {
            phase: {
                "started": started[phase],
                "aborted": aborted[phase],
                "abort_rate": aborted[phase] / started[phase] if started[phase] else None,
            }
            for phase in phases
        },
    }


def write_outputs(events: list[dict[str, object]], output_dir: Path) -> None:
    """再利用可能な集計ファイルと2種類のグラフを保存する。"""
    output_dir.mkdir(parents=True, exist_ok=True)
    summary = summarize(events)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    keys = sorted({key for event in events for key in event})
    with (output_dir / "events.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(events)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    latencies = [
        float(event["latency_ms"])
        for event in events
        if event["event"] == "planner_completed"
        and isinstance(event.get("latency_ms"), (int, float))
    ]
    figure, axis = plt.subplots(figsize=(8, 4.5))
    axis.plot(range(1, len(latencies) + 1), latencies, marker="o", label="Planner latency")
    axis.set(xlabel="Planning attempt", ylabel="Latency [ms]", title="MoveIt planner latency")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_dir / "planner_latency.png", dpi=150)
    plt.close(figure)

    phase_abort = summary["phase_abort"]
    assert isinstance(phase_abort, dict)
    phases = list(phase_abort)
    rates = [float(phase_abort[phase]["abort_rate"] or 0.0) * 100.0 for phase in phases]
    figure, axis = plt.subplots(figsize=(8, 4.5))
    axis.bar(phases, rates, label="Abort rate")
    axis.set(xlabel="Execution phase", ylabel="Abort rate [%]", title="Abort rate by phase")
    axis.set_ylim(0, max(100.0, max(rates, default=0.0) * 1.1))
    axis.tick_params(axis="x", rotation=25)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_dir / "phase_abort_rate.png", dpi=150)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    events = read_events(args.logs)
    if not events:
        parser.error("no valid MOVEIT_METRIC events found")
    write_outputs(events, args.output_dir)


if __name__ == "__main__":
    main()
