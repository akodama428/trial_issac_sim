#!/usr/bin/env python3
"""10初期姿勢E2EのログをJSON/Markdownへ集約する。"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

SRC_ROOT = Path(__file__).resolve().parents[2] / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from tomato_harvest_sim.simulator.initial_pose_cases import INITIAL_POSE_CASES


def _grasp_diagnostics(log: str) -> dict[str, object]:
    samples: list[dict[str, object]] = []
    for line in log.splitlines():
        marker = "MOVEIT_METRIC "
        if marker not in line:
            continue
        try:
            payload = json.loads(line.split(marker, 1)[1])
        except (json.JSONDecodeError, IndexError):
            continue
        if payload.get("event") == "grasp_evaluation_diagnostic":
            samples.append(payload)
    if not samples:
        return {"sample_count": 0}
    terminal = next((sample for sample in reversed(samples) if sample.get("sample_kind") == "terminal"), samples[-1])
    return {
        "sample_count": len(samples),
        "minimum_position_error_norm_m": min(float(sample["position_error_norm_m"]) for sample in samples),
        "terminal": terminal,
    }


def _live_tracking_errors(log: str) -> list[float]:
    """実行中statusから周期配信された追従誤差を抽出する。"""
    values = re.findall(
        r'execution_status \{"status":"running","tracking_error_rad":([0-9.eE+-]+)',
        log,
    )
    return [float(value) for value in values]


def summarize(root: Path, case_ids: list[str], sha: str) -> dict[str, object]:
    definitions = {case.case_id: case for case in INITIAL_POSE_CASES}
    results = []
    for case_id in case_ids:
        definition = definitions[case_id]
        case_root = root / case_id / "e2e"
        robot_log = (case_root / "robot_node.log").read_text(errors="replace") if (case_root / "robot_node.log").exists() else ""
        controller_log = (case_root / "franka_controller.log").read_text(errors="replace") if (case_root / "franka_controller.log").exists() else ""
        console = (case_root / "docker-e2e-console.log").read_text(errors="replace") if (case_root / "docker-e2e-console.log").exists() else ""
        success = bool(re.search(r"Phase: returning_home .* complete", robot_log)) and not bool(re.search(r"Phase: .* failed", robot_log))
        failed = re.findall(r"Phase: ([a-z_]+) .* failed", robot_log)
        latencies = [float(v) for v in re.findall(r'"event": "planner_completed"[^\n]*"latency_ms": ([0-9.]+)', robot_log)]
        duration = re.findall(r"E2E_CASE_DURATION_SEC=([0-9.]+)", console)
        live_tracking_errors = _live_tracking_errors(controller_log)
        results.append({
            "case_id": case_id,
            "initial_positions_rad": list(definition.positions_rad),
            "is_singularity_case": definition.is_singularity_case,
            "success": success,
            "failure_reason": (
                "" if success
                # 起動flake (Issue #40)。計画・実行系の失敗と区別して集計する。
                else "stack_startup_failed" if "STACK_STARTUP_FAILED" in console
                else f"failed_phase:{failed[-1]}" if failed
                else "cycle_not_completed"
            ),
            "planning_latency_ms": latencies,
            "e2e_duration_sec": float(duration[-1]) if duration else None,
            "grasp_diagnostics": _grasp_diagnostics(robot_log),
            "live_tracking_sample_count": len(live_tracking_errors),
            "maximum_live_tracking_error_rad": (
                max(live_tracking_errors) if live_tracking_errors else None
            ),
        })
    success_count = sum(bool(item["success"]) for item in results)
    return {
        "commit_sha": sha, "case_count": len(results), "success_count": success_count,
        "failure_count": len(results) - success_count,
        "success_rate": success_count / len(results) if results else 0.0,
        "cases": results,
    }


def markdown(summary: dict[str, object]) -> str:
    lines = ["# Initial pose E2E result", "", f"Commit: `{summary['commit_sha']}`", "",
             f"Success: {summary['success_count']}/{summary['case_count']} ({float(summary['success_rate']):.0%})", "",
             "| Case | Result | Failure reason | Live samples | Max tracking error [rad] | E2E sec |", "|---|---|---|---:|---:|---:|"]
    for item in summary["cases"]:  # type: ignore[union-attr]
        maximum_error = item["maximum_live_tracking_error_rad"]
        lines.append(f"| {item['case_id']} | {'PASS' if item['success'] else 'FAIL'} | {item['failure_reason'] or '-'} | {item['live_tracking_sample_count']} | {maximum_error if maximum_error is not None else '-'} | {item['e2e_duration_sec'] or '-'} |")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--cases", required=True)
    parser.add_argument("--sha", default="local")
    parser.add_argument("--threshold", type=float, default=0.7)
    args = parser.parse_args()
    summary = summarize(args.root, args.cases.split(","), args.sha)
    (args.root / "initial-pose-summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    report = markdown(summary)
    (args.root / "initial-pose-summary.md").write_text(report)
    print(report)
    return 0 if float(summary["success_rate"]) >= args.threshold else 1


if __name__ == "__main__":
    raise SystemExit(main())
