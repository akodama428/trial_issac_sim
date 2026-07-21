#!/usr/bin/env python3
"""Issue #5のpull/non-pullログを採点し、stem破断の必須グラフを生成する。"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

PHYSICS_HZ = 120.0
_OBS_PATTERN = re.compile(r"^\[PhysicsObs\] (.+)$")
_BREAK_PATTERN = re.compile(r"^\[JointBreakObs\] (.+)$")


def _key_values(text: str) -> dict[str, str]:
    return dict(part.split("=", 1) for part in text.split() if "=" in part)


def parse_log(path: Path) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    observations: list[dict[str, str]] = []
    events: list[dict[str, str]] = []
    event_keys: set[tuple[str | None, str | None, str | None]] = set()
    with path.open(encoding="utf-8", errors="replace") as stream:
        for raw_line in stream:
            line = raw_line.strip()
            observation = _OBS_PATTERN.match(line)
            if observation is not None:
                observations.append(_key_values(observation.group(1)))
                continue
            event = _BREAK_PATTERN.match(line)
            if event is not None:
                values = _key_values(event.group(1))
                key = (
                    values.get("decision"),
                    values.get("joint"),
                    values.get("seq"),
                )
                if key not in event_keys:
                    event_keys.add(key)
                    events.append(values)
    return observations, events


def summarize_run(
    rows: list[dict[str, str]],
    events: list[dict[str, str]],
    *,
    break_force_n: float,
    expect_break: bool,
    minimum_post_break_samples: int = 3,
    minimum_non_pull_samples: int = 1200,
) -> dict[str, object]:
    evaluated_rows = (
        rows
        if expect_break
        else [row for row in rows if row.get("status") == "held"]
    )
    target_events = [
        event for event in events if event.get("decision") == "target_broken"
    ]
    break_sequence = (
        int(target_events[0]["seq"]) if len(target_events) == 1 else None
    )
    post_break = (
        [row for row in rows if int(row["seq"]) > break_sequence]
        if break_sequence is not None
        else []
    )
    post_break_held = [
        row
        for row in post_break
        if row.get("status") != "fallen"
        and float(row.get("forceL", "0")) > 0.0
        and float(row.get("forceR", "0")) > 0.0
    ]
    duration_sec = (
        float(evaluated_rows[-1]["t"]) - float(evaluated_rows[0]["t"])
        if len(evaluated_rows) >= 2
        else 0.0
    )
    break_detected_pass = len(target_events) == 1
    post_break_hold_pass = (
        len(post_break_held) >= minimum_post_break_samples
        if expect_break
        else True
    )
    no_false_break_pass = (
        len(target_events) == 0
        and len(evaluated_rows) >= minimum_non_pull_samples
        and duration_sec >= (minimum_non_pull_samples - 1) / PHYSICS_HZ
    )
    overall_pass = (
        break_detected_pass and post_break_hold_pass
        if expect_break
        else no_false_break_pass
    )
    return {
        "expect_break": expect_break,
        "observation_samples": len(evaluated_rows),
        "observation_duration_sec": duration_sec,
        "target_break_count": len(target_events),
        "break_sequence": break_sequence,
        "break_force_n": break_force_n,
        "maximum_estimated_tension_n": max(
            (float(row["stemF"]) for row in rows), default=0.0
        ),
        "maximum_stem_distance_m": max(
            (float(row["stem_d"]) for row in rows), default=0.0
        ),
        "post_break_samples": len(post_break),
        "post_break_held_samples": len(post_break_held),
        "break_detected_pass": break_detected_pass,
        "post_break_hold_pass": post_break_hold_pass,
        "no_false_break_pass": no_false_break_pass,
        "overall_pass": overall_pass,
    }


def _break_time(
    rows: list[dict[str, str]], events: list[dict[str, str]]
) -> float | None:
    target = next(
        (event for event in events if event.get("decision") == "target_broken"),
        None,
    )
    if target is None:
        return None
    sequence = int(target["seq"])
    row = next((item for item in rows if int(item["seq"]) >= sequence), None)
    return float(row["t"]) if row is not None else None


def render(
    pull_rows: list[dict[str, str]],
    pull_events: list[dict[str, str]],
    non_pull_rows: list[dict[str, str]],
    non_pull_events: list[dict[str, str]],
    *,
    break_force_n: float,
    out_dir: Path,
) -> dict[str, str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)

    def save(name: str) -> str:
        target = out_dir / name
        plt.savefig(target, dpi=130, bbox_inches="tight")
        plt.close()
        return str(target)

    def tension_plot(
        rows: list[dict[str, str]],
        events: list[dict[str, str]],
        *,
        title: str,
        filename: str,
    ) -> str:
        times = [float(row["t"]) for row in rows]
        plt.figure(figsize=(9, 4))
        plt.plot(times, [float(row["stemF"]) for row in rows], label="estimated tension")
        plt.axhline(
            break_force_n,
            color="tab:red",
            linestyle="--",
            label=f"break force: {break_force_n:g} N",
        )
        event_time = _break_time(rows, events)
        if event_time is not None:
            plt.axvline(event_time, color="black", linestyle=":", label="JOINT_BREAK")
        plt.title(title)
        plt.xlabel("simulation time [s]")
        plt.ylabel("estimated stem tension [N]")
        plt.grid(alpha=0.3)
        plt.legend()
        return save(filename)

    pull_path = tension_plot(
        pull_rows,
        pull_events,
        title="Pull run: stem tension and physical break",
        filename="issue5_pull_tension.png",
    )
    non_pull_path = tension_plot(
        non_pull_rows,
        non_pull_events,
        title="Non-pull run: 10 s false-break guard",
        filename="issue5_non_pull_tension.png",
    )

    plt.figure(figsize=(7, 4))
    labels = ["non-pull max", "break force", "pull max"]
    values = [
        max((float(row["stemF"]) for row in non_pull_rows), default=0.0),
        break_force_n,
        max((float(row["stemF"]) for row in pull_rows), default=0.0),
    ]
    plt.bar(labels, values, color=["tab:blue", "tab:red", "tab:orange"])
    plt.ylabel("force [N]")
    plt.title("Stem break separation margin")
    plt.grid(axis="y", alpha=0.3)
    margin_path = save("issue5_tension_margin.png")

    event_time = _break_time(pull_rows, pull_events)
    selected = [
        row
        for row in pull_rows
        if event_time is None or abs(float(row["t"]) - event_time) <= 1.0
    ]
    plt.figure(figsize=(9, 4))
    plt.plot(
        [float(row["t"]) for row in selected],
        [float(row["forceL"]) for row in selected],
        label="left finger",
    )
    plt.plot(
        [float(row["t"]) for row in selected],
        [float(row["forceR"]) for row in selected],
        label="right finger",
    )
    if event_time is not None:
        plt.axvline(event_time, color="black", linestyle=":", label="JOINT_BREAK")
    plt.xlabel("simulation time [s]")
    plt.ylabel("finger contact force [N]")
    plt.title("Friction hold around stem break")
    plt.grid(alpha=0.3)
    plt.legend()
    hold_path = save("issue5_post_break_hold.png")
    return {
        "pull_tension": pull_path,
        "non_pull_tension": non_pull_path,
        "tension_margin": margin_path,
        "post_break_hold": hold_path,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pull-log", type=Path, required=True)
    parser.add_argument("--non-pull-log", type=Path, required=True)
    parser.add_argument("--break-force-n", type=float, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    pull_rows, pull_events = parse_log(args.pull_log)
    non_pull_rows, non_pull_events = parse_log(args.non_pull_log)
    pull_summary = summarize_run(
        pull_rows,
        pull_events,
        break_force_n=args.break_force_n,
        expect_break=True,
    )
    non_pull_summary = summarize_run(
        non_pull_rows,
        non_pull_events,
        break_force_n=args.break_force_n,
        expect_break=False,
    )
    images = render(
        pull_rows,
        pull_events,
        non_pull_rows,
        non_pull_events,
        break_force_n=args.break_force_n,
        out_dir=args.out_dir,
    )
    summary = {
        "pull": pull_summary,
        "non_pull": non_pull_summary,
        "images": images,
        "overall_pass": bool(pull_summary["overall_pass"])
        and bool(non_pull_summary["overall_pass"]),
    }
    target = args.out_dir / "issue5_stem_break_summary.json"
    target.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
