#!/usr/bin/env python3
"""物理観測ログ（[PhysicsObs]）とフェーズ遷移ログの可視化。

Step 0（観測基盤）の検証レポート用グラフを生成する。
入力:
  - sim ログ: isaac_viewer 標準出力。physics_harvest が出す [PhysicsObs] 行を含む。
  - robot ログ: behavior_planner_node の "Phase: a → b" 行（rclpy 形式）を含む。
出力:
  - <out-dir>/<prefix>_contact_impulse.png : finger 別接触力積（力換算の第2軸付き）
  - <out-dir>/<prefix>_tomato_motion.png   : トマト速度・茎張力推定
  - <out-dir>/<prefix>_distances.png       : hand-トマト距離・stem-トマト距離
  - <out-dir>/<prefix>_phases.png          : フェーズ滞在時間の横棒グラフ
  - <out-dir>/<prefix>_summary.json        : レポート埋め込み用の数値サマリ

使用例:
  python3 scripts/plot_physics_observation.py \
      --sim-log /tmp/sim_run1.log --robot-log /tmp/robot_run1.log \
      --out-dir docs/reports/img --prefix run1
"""
from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass, field
from pathlib import Path

OBSERVATION_PHYSICS_DT_SEC = 1.0 / 60.0

_OBS_PATTERN = re.compile(r"^\[PhysicsObs\] (.+)$")
_PHASE_PATTERN = re.compile(
    r"\[INFO\] \[(?P<stamp>\d+\.\d+)\].*Phase: (?P<src>\S+) (?:→|->) (?P<dst>\S+)"
)


@dataclass
class ObservationSeries:
    time_sec: list[float] = field(default_factory=list)
    status: list[str] = field(default_factory=list)
    gripper_closed: list[int] = field(default_factory=list)
    grasp_joint: list[int] = field(default_factory=list)
    impulse_left: list[float] = field(default_factory=list)
    impulse_right: list[float] = field(default_factory=list)
    tomato_speed: list[float] = field(default_factory=list)
    hand_distance: list[float] = field(default_factory=list)
    stem_distance: list[float] = field(default_factory=list)
    stem_tension: list[float] = field(default_factory=list)


@dataclass(frozen=True)
class PhaseTransition:
    stamp_sec: float
    src: str
    dst: str


def parse_observation_log(path: Path) -> ObservationSeries:
    """sim ログから [PhysicsObs] 行を時系列として読み出す。"""
    series = ObservationSeries()
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = _OBS_PATTERN.match(line.strip())
            if match is None:
                continue
            fields = dict(part.split("=", 1) for part in match.group(1).split())
            series.time_sec.append(float(fields["t"]))
            series.status.append(fields["status"])
            series.gripper_closed.append(int(fields["grip"]))
            series.grasp_joint.append(int(fields["joint"]))
            series.impulse_left.append(float(fields["impL"]))
            series.impulse_right.append(float(fields["impR"]))
            series.tomato_speed.append(float(fields["v"]))
            series.hand_distance.append(float(fields["hand_d"]))
            series.stem_distance.append(float(fields["stem_d"]))
            series.stem_tension.append(float(fields["stemF"]))
    return series


def parse_phase_transitions(path: Path) -> list[PhaseTransition]:
    """robot ログからフェーズ遷移を時刻付きで読み出す（最後のサイクルのみ）。

    robot ログは追記式のため、同一ファイルに複数回の実行が混在し得る。
    最後の「idle → detecting」（サイクル開始）以降だけを対象にする。
    """
    transitions: list[PhaseTransition] = []
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = _PHASE_PATTERN.search(line)
            if match is None:
                continue
            transitions.append(
                PhaseTransition(
                    stamp_sec=float(match.group("stamp")),
                    src=match.group("src"),
                    dst=match.group("dst"),
                )
            )
    last_cycle_start = 0
    for index, transition in enumerate(transitions):
        if transition.src == "idle" and transition.dst == "detecting":
            last_cycle_start = index
    return transitions[last_cycle_start:]


def phase_durations_sec(transitions: list[PhaseTransition]) -> list[tuple[str, float]]:
    """遷移列から各フェーズの滞在時間を求める（最初のサイクルのみ）。

    headless 実行では step 予算が余ると 2 周目のサイクルが始まることがあるため、
    最初に complete へ到達した時点で集計を打ち切る。最後のフェーズは滞在中のため除外。
    """
    durations: list[tuple[str, float]] = []
    for current, following in zip(transitions, transitions[1:]):
        durations.append((current.dst, following.stamp_sec - current.stamp_sec))
        if following.dst == "complete":
            break
    return durations


def reached_complete(transitions: list[PhaseTransition]) -> bool:
    return any(t.dst == "complete" for t in transitions)


def _relative_time(series: ObservationSeries) -> list[float]:
    if not series.time_sec:
        return []
    origin = series.time_sec[0]
    return [t - origin for t in series.time_sec]


def trim_idle_tail(series: ObservationSeries, *, margin_sec: float = 45.0) -> ObservationSeries:
    """サイクル完了後のアイドル区間を除去し、グラフの可読性を確保する。

    最後に tomato_status が変化した時刻 + margin_sec までを残す。
    headless 実行では所定 step 数までアイドルで回り続けるため、
    トリムしないと有効区間がグラフの一部に圧縮されてしまう。
    """
    if not series.time_sec:
        return series
    last_change_time = series.time_sec[0]
    for index in range(1, len(series.status)):
        if series.status[index] != series.status[index - 1]:
            last_change_time = series.time_sec[index]
    cutoff = last_change_time + margin_sec
    keep = sum(1 for t in series.time_sec if t <= cutoff)
    trimmed = ObservationSeries()
    for name in vars(trimmed):
        setattr(trimmed, name, getattr(series, name)[:keep])
    return trimmed


def render_plots(series: ObservationSeries, transitions: list[PhaseTransition],
                 out_dir: Path, prefix: str) -> dict[str, object]:
    """グラフ4種と数値サマリを出力する。matplotlib は Agg で動作する。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    rel_time = _relative_time(series)

    def _saved(fig: object, name: str) -> str:
        target = out_dir / f"{prefix}_{name}.png"
        fig.savefig(target, dpi=110, bbox_inches="tight")
        plt.close(fig)
        return str(target)

    # 1. 接触力積（力換算の目安付き）
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(rel_time, series.impulse_left, label="left finger", linewidth=0.9)
    ax.plot(rel_time, series.impulse_right, label="right finger", linewidth=0.9)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("contact impulse [N·s / step]")
    secondary = ax.secondary_yaxis(
        "right",
        functions=(
            lambda v: v / OBSERVATION_PHYSICS_DT_SEC,
            lambda v: v * OBSERVATION_PHYSICS_DT_SEC,
        ),
    )
    secondary.set_ylabel("approx. force [N] (dt=1/60s)")
    ax.set_title(f"{prefix}: finger contact impulses")
    ax.legend()
    ax.grid(alpha=0.3)
    contact_png = _saved(fig, "contact_impulse")

    # 2. トマト速度と茎張力推定
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(rel_time, series.tomato_speed, label="tomato speed [m/s]", linewidth=0.9)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("speed [m/s]")
    ax2 = ax.twinx()
    ax2.plot(rel_time, series.stem_tension, label="stem tension est. [N]",
             color="tab:red", linewidth=0.9, alpha=0.7)
    ax2.set_ylabel("tension [N]")
    ax.set_title(f"{prefix}: tomato motion / stem tension estimate")
    ax.grid(alpha=0.3)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right")
    motion_png = _saved(fig, "tomato_motion")

    # 3. 距離（hand-トマト / stem-トマト）
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(rel_time, series.hand_distance, label="hand ↔ tomato [m]", linewidth=0.9)
    ax.plot(rel_time, series.stem_distance, label="stem anchor ↔ tomato [m]", linewidth=0.9)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("distance [m]")
    ax.set_title(f"{prefix}: distances")
    ax.legend()
    ax.grid(alpha=0.3)
    distance_png = _saved(fig, "distances")

    # 4. フェーズ滞在時間
    durations = phase_durations_sec(transitions)
    fig, ax = plt.subplots(figsize=(8, max(2.5, 0.4 * len(durations))))
    if durations:
        labels = [f"{index}: {name}" for index, (name, _) in enumerate(durations)]
        values = [seconds for _, seconds in durations]
        ax.barh(labels, values, color="tab:blue")
        ax.invert_yaxis()
    ax.set_xlabel("duration [s]")
    ax.set_title(f"{prefix}: phase durations")
    ax.grid(alpha=0.3, axis="x")
    phases_png = _saved(fig, "phases")

    summary = {
        "prefix": prefix,
        "observation_samples": len(series.time_sec),
        "duration_sec": rel_time[-1] if rel_time else 0.0,
        "max_impulse_left_ns": max(series.impulse_left, default=0.0),
        "max_impulse_right_ns": max(series.impulse_right, default=0.0),
        "max_stem_tension_n": max(series.stem_tension, default=0.0),
        "max_tomato_speed_m_s": max(series.tomato_speed, default=0.0),
        "phase_transitions": [
            {"time": t.stamp_sec, "from": t.src, "to": t.dst} for t in transitions
        ],
        "phase_durations_sec": [
            {"phase": name, "sec": seconds} for name, seconds in durations
        ],
        "reached_complete": reached_complete(transitions),
        "images": {
            "contact_impulse": contact_png,
            "tomato_motion": motion_png,
            "distances": distance_png,
            "phases": phases_png,
        },
    }
    summary_path = out_dir / f"{prefix}_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim-log", type=Path, required=True)
    parser.add_argument("--robot-log", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--prefix", type=str, required=True)
    args = parser.parse_args()

    series = trim_idle_tail(parse_observation_log(args.sim_log))
    transitions = parse_phase_transitions(args.robot_log)
    summary = render_plots(series, transitions, args.out_dir, args.prefix)
    print(
        f"[plot_physics_observation] prefix={args.prefix} "
        f"samples={summary['observation_samples']} "
        f"complete={summary['reached_complete']} "
        f"images={len(summary['images'])}"
    )


if __name__ == "__main__":
    main()
