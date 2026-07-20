#!/usr/bin/env python3
"""Issue #4のphysics holdログを採点し、必須グラフとsummaryを生成する。"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path

PHYSICS_HZ = 120.0
SLIP_LIMIT_M = 0.005
MINIMUM_LIFT_M = 0.1
REQUIRED_HOLD_STEPS = 600
_OBS_PATTERN = re.compile(r"^\[PhysicsObs\] (.+)$")


def hold_evaluation_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    """hold開始からrequired step完了を最初に記録したsampleまでを返す。"""
    active = [row for row in rows if row.get("hold") == "1"]
    for index, row in enumerate(active):
        if int(row["hold_steps"]) >= REQUIRED_HOLD_STEPS:
            return active[: index + 1]
    return active


def parse_physics_observations(path: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = _OBS_PATTERN.match(line.strip())
            if match is None:
                continue
            rows.append(
                dict(part.split("=", 1) for part in match.group(1).split())
            )
    return rows


def summarize_observations(rows: list[dict[str, str]]) -> dict[str, object]:
    active_rows = [row for row in rows if row.get("hold") == "1"]
    hold_rows = hold_evaluation_rows(rows)
    max_hold_steps = max(
        (int(row["hold_steps"]) for row in active_rows), default=0
    )
    max_slip = max(
        (float(row["hold_slip"]) for row in hold_rows), default=math.inf
    )
    max_lift = max(
        (float(row["stem_d"]) for row in hold_rows), default=0.0
    )
    return {
        "observation_samples": len(rows),
        "hold_samples": len(hold_rows),
        "hold_elapsed_steps": max_hold_steps,
        "hold_duration_sec": max_hold_steps / PHYSICS_HZ,
        "maximum_lift_distance_m": max_lift,
        "maximum_hold_slip_m": max_slip,
        "minimum_hold_force_left_n": min(
            (float(row["forceL"]) for row in hold_rows), default=0.0
        ),
        "minimum_hold_force_right_n": min(
            (float(row["forceR"]) for row in hold_rows), default=0.0
        ),
        "grasp_joint_create_count": max(
            (int(row["joint_count"]) for row in rows), default=0
        ),
        "geometry_fallback_count": max(
            (int(row["fallback_count"]) for row in rows), default=0
        ),
        "teleport_restore_count": max(
            (int(row["teleport_count"]) for row in rows), default=0
        ),
        "lift_pass": max_lift >= MINIMUM_LIFT_M,
        "duration_pass": max_hold_steps >= REQUIRED_HOLD_STEPS,
        "slip_pass": max_slip < SLIP_LIMIT_M,
    }


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8", errors="replace") as stream:
        return list(csv.DictReader(stream))


def _joint7_hold_velocity(
    controller_rows: list[dict[str, str]],
    phase_rows: list[dict[str, str]],
) -> tuple[list[float], list[float]]:
    detaching_start = next(
        (float(row["t"]) for row in phase_rows if row["phase"] == "detaching"),
        None,
    )
    moving_to_place = next(
        (
            float(row["t"])
            for row in phase_rows
            if row["phase"] == "moving_to_place"
            and (detaching_start is None or float(row["t"]) > detaching_start)
        ),
        None,
    )
    if detaching_start is None or moving_to_place is None:
        return [], []
    hold_start = max(detaching_start, moving_to_place - 5.0)
    selected = [
        row
        for row in controller_rows
        if hold_start <= float(row["t"]) <= moving_to_place
    ]
    return (
        [float(row["t"]) - hold_start for row in selected],
        [float(row["panda_joint7_fb_velocity"]) for row in selected],
    )


def render(
    rows: list[dict[str, str]],
    controller_rows: list[dict[str, str]],
    phase_rows: list[dict[str, str]],
    out_dir: Path,
) -> dict[str, str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    hold_rows = hold_evaluation_rows(rows)
    hold_time = [int(row["hold_steps"]) / PHYSICS_HZ for row in hold_rows]

    def save(name: str) -> str:
        target = out_dir / name
        plt.savefig(target, dpi=130, bbox_inches="tight")
        plt.close()
        return str(target)

    plt.figure(figsize=(9, 4))
    plt.plot(
        hold_time,
        [1000.0 * float(row["hold_slip"]) for row in hold_rows],
        label="hand-local relative displacement",
    )
    plt.axhline(5.0, color="tab:red", linestyle="--", label="limit: 5 mm")
    plt.xlabel("hold time [s]")
    plt.ylabel("relative displacement [mm]")
    plt.grid(alpha=0.3)
    plt.legend()
    slip_path = save("issue4_hold_relative_displacement.png")

    plt.figure(figsize=(9, 4))
    plt.plot(
        hold_time,
        [float(row["forceL"]) for row in hold_rows],
        label="left finger",
    )
    plt.plot(
        hold_time,
        [float(row["forceR"]) for row in hold_rows],
        label="right finger",
    )
    plt.xlabel("hold time [s]")
    plt.ylabel("contact force [N]")
    plt.grid(alpha=0.3)
    plt.legend()
    force_path = save("issue4_hold_finger_forces.png")

    joint_time, joint7_velocity = _joint7_hold_velocity(
        controller_rows, phase_rows
    )
    plt.figure(figsize=(9, 4))
    plt.plot(joint_time, joint7_velocity, label="panda_joint7 feedback")
    plt.axhline(0.0, color="black", linewidth=0.8)
    plt.xlabel("hold time [s]")
    plt.ylabel("joint7 velocity [rad/s]")
    plt.grid(alpha=0.3)
    plt.legend()
    joint_path = save("issue4_hold_joint7_velocity.png")
    return {
        "relative_displacement": slip_path,
        "finger_forces": force_path,
        "joint7_velocity": joint_path,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim-log", type=Path, required=True)
    parser.add_argument("--robot-log", type=Path, required=True)
    parser.add_argument("--controller-csv", type=Path, required=True)
    parser.add_argument("--phase-csv", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    observations = parse_physics_observations(args.sim_log)
    summary = summarize_observations(observations)
    robot_text = args.robot_log.read_text(encoding="utf-8", errors="replace")
    summary["natural_release_pass"] = (
        "Phase: moving_to_place → releasing" in robot_text
        and "Phase: releasing → placed" in robot_text
    )
    summary["cycle_complete"] = "Phase: returning_home → complete" in robot_text
    controller_rows = _read_csv(args.controller_csv)
    phase_rows = _read_csv(args.phase_csv)
    _, joint7_velocity = _joint7_hold_velocity(controller_rows, phase_rows)
    summary["maximum_abs_joint7_velocity_rad_s"] = max(
        (abs(value) for value in joint7_velocity), default=math.nan
    )
    summary["images"] = render(
        observations, controller_rows, phase_rows, args.out_dir
    )
    summary["overall_pass"] = all(
        bool(summary[key])
        for key in (
            "lift_pass",
            "duration_pass",
            "slip_pass",
            "natural_release_pass",
            "cycle_complete",
        )
    ) and all(
        summary[key] == 0
        for key in (
            "grasp_joint_create_count",
            "geometry_fallback_count",
            "teleport_restore_count",
        )
    )
    target = args.out_dir / "issue4_friction_hold_summary.json"
    target.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
