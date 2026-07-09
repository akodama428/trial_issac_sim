#!/usr/bin/env python3
"""ATTACHED 静置検証の before/after 比較グラフ（Step 1）。

物理チューニング適用前後の 2 つの sim ログから、トマトの初期位置からの
偏差（茎アンカー距離）と速度の時系列を重ねて描画し、テレポート復元の
発動回数を数える。

使用例:
  python3 scripts/plot_idle_comparison.py \
      --before docs/reports/data/idle_before.log --after docs/reports/data/idle_after.log \
      --out-dir docs/reports/img --prefix step1_idle
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from plot_physics_observation import parse_observation_log  # 同ディレクトリ


def count_teleport_restores(path: Path) -> int:
    with path.open(encoding="utf-8", errors="replace") as stream:
        return sum(1 for line in stream if "restoring unstable attached tomato pose" in line)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--prefix", type=str, required=True)
    args = parser.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    args.out_dir.mkdir(parents=True, exist_ok=True)
    results = {}
    fig, (ax_dev, ax_vel) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    for label, path, color in (
        ("before (tuning off)", args.before, "tab:red"),
        ("after (tuning on)", args.after, "tab:green"),
    ):
        series = parse_observation_log(path)
        if not series.time_sec:
            raise SystemExit(f"no observation lines in {path}")
        origin = series.time_sec[0]
        rel = [t - origin for t in series.time_sec]
        deviation_mm = [d * 1000.0 for d in series.stem_distance]
        speed = series.tomato_speed
        ax_dev.plot(rel, deviation_mm, label=label, color=color, linewidth=0.9)
        ax_vel.plot(rel, speed, label=label, color=color, linewidth=0.9)
        results[label] = {
            "samples": len(rel),
            "max_deviation_mm": max(deviation_mm),
            "mean_deviation_mm": sum(deviation_mm) / len(deviation_mm),
            "max_speed_m_s": max(speed),
            "teleport_restores": count_teleport_restores(path),
        }

    ax_dev.set_ylabel("deviation from anchor [mm]")
    ax_dev.set_title(f"{args.prefix}: ATTACHED idle stability (before/after)")
    ax_dev.grid(alpha=0.3)
    ax_dev.legend()
    ax_vel.set_ylabel("tomato speed [m/s]")
    ax_vel.set_xlabel("time [s]")
    ax_vel.grid(alpha=0.3)
    ax_vel.legend()
    png = args.out_dir / f"{args.prefix}_comparison.png"
    fig.savefig(png, dpi=110, bbox_inches="tight")

    summary_path = args.out_dir / f"{args.prefix}_summary.json"
    summary_path.write_text(json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"[plot_idle_comparison] {json.dumps(results, ensure_ascii=False)}")


if __name__ == "__main__":
    main()
