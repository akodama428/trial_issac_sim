#!/usr/bin/env python3
"""Issue #6のPLACED/FALLENログを採点し、軌跡・速度・遅延を可視化する。"""
from __future__ import annotations

import argparse
import json
import re
import statistics
from pathlib import Path


_OBS_PATTERN = re.compile(r"^\[PlacementObs\] (.+)$")


def _key_values(text: str) -> dict[str, str]:
    return dict(part.split("=", 1) for part in text.split() if "=" in part)


def parse_log(path: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    with path.open(encoding="utf-8", errors="replace") as stream:
        for raw_line in stream:
            match = _OBS_PATTERN.match(raw_line.strip())
            if match is not None:
                rows.append(_key_values(match.group(1)))
    return rows


def split_cycles(rows: list[dict[str, str]]) -> list[list[dict[str, str]]]:
    """cycle IDごとに入力順を維持して観測列を分ける。"""
    cycles: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        cycles.setdefault(row.get("cycle", "unknown"), []).append(row)
    return list(cycles.values())


def load_runs(paths: list[Path]) -> list[list[dict[str, str]]]:
    """複数log内の同名cycle IDを混同せず、全release cycleを返す。"""
    return [cycle for path in paths for cycle in split_cycles(parse_log(path))]


def _event_row(rows: list[dict[str, str]], event: str) -> dict[str, str] | None:
    return next(
        (row for row in rows if event in row.get("event", "").split("+")), None
    )


def summarize_run(
    rows: list[dict[str, str]], *, expected_decision: str
) -> dict[str, object]:
    release = _event_row(rows, "release_started")
    contact = _event_row(rows, "first_tray_contact")
    terminal = _event_row(rows, "terminal")
    # release_startedは最初のphysics sampleへ付与されるが、監視時計の原点は0秒。
    release_time = 0.0 if release else None
    contact_time = float(contact["elapsed"]) if contact else None
    terminal_time = float(terminal["elapsed"]) if terminal else None

    def latency(start: float | None, end: float | None) -> float | None:
        return round(end - start, 6) if start is not None and end is not None else None

    terminal_decision = terminal.get("decision") if terminal else None
    marker_pass = release is not None and terminal is not None
    decision_pass = terminal_decision == expected_decision
    final = terminal or (rows[-1] if rows else {})
    return {
        "observation_samples": len(rows),
        "release_marker_pass": release is not None,
        "contact_marker_pass": contact is not None,
        "terminal_marker_pass": terminal is not None,
        "decision": terminal_decision,
        "reason": terminal.get("reason") if terminal else None,
        "release_to_contact_sec": latency(release_time, contact_time),
        "contact_to_terminal_sec": latency(contact_time, terminal_time),
        "release_to_terminal_sec": latency(release_time, terminal_time),
        "final_local_position_m": {
            axis: float(final.get(f"local_{axis}", "nan")) for axis in ("x", "y", "z")
        },
        "maximum_linear_speed_m_s": max(
            (float(row["speed"]) for row in rows), default=0.0
        ),
        "terminal_linear_speed_m_s": float(final.get("speed", "nan")),
        "marker_pass": marker_pass,
        "decision_pass": decision_pass,
        "overall_pass": marker_pass and decision_pass,
    }


def render(
    runs: dict[str, list[dict[str, str]]], *, out_dir: Path,
    tray_inner_size_m: tuple[float, float], tomato_radius_m: float,
    boundary_margin_m: float, escape_margin_m: float, speed_limit_m_s: float,
) -> dict[str, str]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    out_dir.mkdir(parents=True, exist_ok=True)
    images: dict[str, str] = {}
    half_x = tray_inner_size_m[0] / 2 - tomato_radius_m
    half_y = tray_inner_size_m[1] / 2 - tomato_radius_m
    for name, rows in runs.items():
        times = [float(row["elapsed"]) for row in rows]
        terminal = _event_row(rows, "terminal")
        contact = _event_row(rows, "first_tray_contact")

        plt.figure(figsize=(7, 4))
        plt.plot([float(r["local_x"]) for r in rows], [float(r["local_z"]) for r in rows])
        plt.axhline(0.0, color="black", label="tray base reference")
        plt.xlabel("tray local x [m]"); plt.ylabel("tray local z [m]")
        plt.title(f"{name.upper()}: release trajectory (x-z)"); plt.grid(alpha=.3); plt.legend()
        target = out_dir / f"issue6_{name}_trajectory_xz.png"
        plt.savefig(target, dpi=130, bbox_inches="tight"); plt.close(); images[f"{name}_trajectory_xz"] = str(target)

        plt.figure(figsize=(6, 5))
        plt.plot([float(r["local_x"]) for r in rows], [float(r["local_y"]) for r in rows])
        ax = plt.gca()
        for inset, color, label in ((boundary_margin_m, "tab:green", "valid region"), (-escape_margin_m, "tab:red", "escape boundary")):
            ax.add_patch(Rectangle((-half_x + inset, -half_y + inset), 2*(half_x-inset), 2*(half_y-inset), fill=False, color=color, label=label))
        plt.xlabel("tray local x [m]"); plt.ylabel("tray local y [m]")
        plt.title(f"{name.upper()}: top trajectory"); plt.axis("equal"); plt.grid(alpha=.3); plt.legend()
        target = out_dir / f"issue6_{name}_trajectory_xy.png"
        plt.savefig(target, dpi=130, bbox_inches="tight"); plt.close(); images[f"{name}_trajectory_xy"] = str(target)

        plt.figure(figsize=(8, 4))
        plt.plot(times, [float(r["speed"]) for r in rows], label="linear speed")
        plt.axhline(speed_limit_m_s, color="tab:red", linestyle="--", label="settle threshold")
        for row, label in ((contact, "first contact"), (terminal, "terminal")):
            if row: plt.axvline(float(row["elapsed"]), linestyle=":", label=label)
        plt.xlabel("release elapsed [s]"); plt.ylabel("speed [m/s]")
        plt.title(f"{name.upper()}: settle speed"); plt.grid(alpha=.3); plt.legend()
        target = out_dir / f"issue6_{name}_speed.png"
        plt.savefig(target, dpi=130, bbox_inches="tight"); plt.close(); images[f"{name}_speed"] = str(target)
    return images


def render_latency(summaries: dict[str, list[dict[str, object]]], out_dir: Path) -> str:
    """runごとのrelease-to-terminal遅延を比較する。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels: list[str] = []
    values: list[float] = []
    colors: list[str] = []
    for decision, runs in summaries.items():
        for index, run in enumerate(runs, start=1):
            latency = run["release_to_terminal_sec"]
            if latency is not None:
                labels.append(f"{decision}-{index}")
                values.append(float(latency))
                colors.append("tab:green" if decision == "placed" else "tab:red")
    plt.figure(figsize=(8, 4))
    plt.bar(labels, values, color=colors)
    plt.ylabel("release to terminal [s]")
    plt.title("Issue #6 settle judgement latency")
    plt.grid(axis="y", alpha=0.3)
    target = out_dir / "issue6_settle_latency.png"
    plt.savefig(target, dpi=130, bbox_inches="tight")
    plt.close()
    return str(target)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--placed-log", type=Path, action="append", required=True)
    parser.add_argument("--fallen-log", type=Path, action="append", required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--tray-inner-size-m", nargs=2, type=float, default=(0.22, 0.16))
    parser.add_argument("--tomato-radius-m", type=float, default=0.01)
    parser.add_argument("--boundary-margin-m", type=float, default=0.005)
    parser.add_argument("--escape-margin-m", type=float, default=0.03)
    parser.add_argument("--speed-limit-m-s", type=float, default=0.03)
    args = parser.parse_args()
    cycle_rows = {
        "placed": load_runs(args.placed_log),
        "fallen": load_runs(args.fallen_log),
    }
    summaries = {
        name: [
            summarize_run(rows, expected_decision="placed" if name == "placed" else "failed")
            for rows in cycles
        ]
        for name, cycles in cycle_rows.items()
    }
    representative = {name: cycles[0] for name, cycles in cycle_rows.items() if cycles}
    images = render(representative, out_dir=args.out_dir, tray_inner_size_m=tuple(args.tray_inner_size_m), tomato_radius_m=args.tomato_radius_m, boundary_margin_m=args.boundary_margin_m, escape_margin_m=args.escape_margin_m, speed_limit_m_s=args.speed_limit_m_s)
    images["settle_latency"] = render_latency(summaries, args.out_dir)
    latencies = [float(run["release_to_terminal_sec"]) for runs in summaries.values() for run in runs if run["release_to_terminal_sec"] is not None]
    enough_runs = all(len(runs) >= 3 for runs in summaries.values())
    summary = {
        "runs": summaries,
        "minimum_three_runs_pass": enough_runs,
        "images": images,
        "release_to_terminal_sec": {
            "min": min(latencies, default=None),
            "median": statistics.median(latencies) if latencies else None,
            "max": max(latencies, default=None),
        },
        "overall_pass": enough_runs and all(
            bool(run["overall_pass"]) for runs in summaries.values() for run in runs
        ),
    }
    target = args.out_dir / "issue6_settle_summary.json"
    target.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
