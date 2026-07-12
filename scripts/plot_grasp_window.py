#!/usr/bin/env python3
"""把持ウィンドウ限定の A/B 分析（Step 2 レポート用）。

トマトを実際に把持している区間（status=held とその直前の閉じ込み遷移）だけを
切り出し、finger ギャップの静定値と finger 別接触力を before/after で比較する。
pregrasp 接近中の「トマト無し完全閉鎖」区間を除外するのが目的。

使用例:
  python3 scripts/plot_grasp_window.py \
      --runs before:run10 before:run11 after:run12 after:run13 after:run14 \
      --data-dir docs/reports/data --out-dir docs/reports/img --max-force-n 5.0
"""
from __future__ import annotations

import argparse
import gzip
import json
import shutil
import statistics
import tempfile
from pathlib import Path

from plot_physics_observation import ObservationSeries, parse_observation_log

PHYSICS_HZ = 60.0
CLOSING_LEAD_STEPS = 60  # held 突入前の閉じ込み区間として含めるステップ数


def load_series(path: Path) -> ObservationSeries:
    """gz 圧縮ログにも対応して観測系列を読み込む。"""
    if path.suffix == ".gz":
        with tempfile.NamedTemporaryFile(suffix=".log", delete=False) as tmp:
            with gzip.open(path, "rb") as src:
                shutil.copyfileobj(src, tmp)
            tmp_path = Path(tmp.name)
        try:
            return parse_observation_log(tmp_path)
        finally:
            tmp_path.unlink(missing_ok=True)
    return parse_observation_log(path)


def grasp_window_indices(series: ObservationSeries) -> tuple[int, int, list[int]]:
    """held 区間 + 直前の閉じ込み区間のインデックス範囲を返す。

    Returns:
        (window開始, window終了(排他), held のインデックス列)。held が無ければ (0,0,[])。
    """
    held = [i for i, s in enumerate(series.status) if s == "held"]
    if not held:
        return 0, 0, []
    start = max(0, held[0] - CLOSING_LEAD_STEPS)
    end = min(len(series.status), held[-1] + 1)
    return start, end, held


def held_metrics(series: ObservationSeries) -> dict[str, float]:
    """held 区間のギャップ静定値と finger 力（中央値）を求める。"""
    _, _, held = grasp_window_indices(series)
    if not held:
        return {}
    gap_mm = [series.finger_gap[i] * 1000.0 for i in held]
    force_left = [series.impulse_left[i] * PHYSICS_HZ for i in held]
    force_right = [series.impulse_right[i] * PHYSICS_HZ for i in held]
    nonzero = lambda vals: [v for v in vals if v > 0.0]
    med = lambda vals: statistics.median(vals) if vals else 0.0
    return {
        "held_steps": len(held),
        "gap_median_mm": med(gap_mm),
        "gap_min_mm": min(gap_mm),
        "gap_max_mm": max(gap_mm),
        "force_left_median_n": med(nonzero(force_left)),
        "force_right_median_n": med(nonzero(force_right)),
        "force_left_p95_n": (
            statistics.quantiles(nonzero(force_left), n=20)[18]
            if len(nonzero(force_left)) >= 20 else max(nonzero(force_left), default=0.0)
        ),
        "left_contact_ratio": len(nonzero(force_left)) / len(held),
        "right_contact_ratio": len(nonzero(force_right)) / len(held),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", nargs="+", required=True,
                        help="label:runname 形式（label は before/after）")
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--max-force-n", type=float, required=True)
    args = parser.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    args.out_dir.mkdir(parents=True, exist_ok=True)
    fig, (ax_gap, ax_force) = plt.subplots(2, 1, figsize=(11, 7))
    summary: dict[str, dict] = {}

    for spec in args.runs:
        label, run = spec.split(":", 1)
        log = args.data_dir / f"sim_{run}.log.gz"
        if not log.exists():
            log = args.data_dir / f"sim_{run}.log"
        series = load_series(log)
        start, end, held = grasp_window_indices(series)
        metrics = held_metrics(series)
        summary[run] = {"label": label, **metrics}
        if not held:
            continue
        origin = series.time_sec[start]
        rel = [series.time_sec[i] - origin for i in range(start, end)]
        style = "--" if label == "before" else "-"
        ax_gap.plot(rel, [series.finger_gap[i] * 1000.0 for i in range(start, end)],
                    style, linewidth=1.0, label=f"{run} ({label})")
        ax_force.plot(rel, [series.impulse_left[i] * PHYSICS_HZ for i in range(start, end)],
                      style, linewidth=1.0, label=f"{run} L ({label})")

    ax_gap.axhline(20.0, color="tab:red", linestyle=":", linewidth=1.2,
                   label="tomato diameter 20 mm")
    ax_gap.set_ylabel("finger gap [mm]")
    ax_gap.set_title("grasp window: finger gap (held + closing lead)")
    ax_gap.grid(alpha=0.3)
    ax_gap.legend(fontsize=8)
    ax_force.axhline(args.max_force_n, color="tab:red", linestyle=":", linewidth=1.2,
                     label=f"maxForce {args.max_force_n:.0f} N (left drive)")
    ax_force.set_ylabel("left finger force [N]")
    ax_force.set_xlabel("time from window start [s]")
    ax_force.set_title("grasp window: left finger contact force (impulse × 60 Hz)")
    ax_force.grid(alpha=0.3)
    ax_force.legend(fontsize=8)
    png = args.out_dir / "step2_grasp_window.png"
    fig.savefig(png, dpi=110, bbox_inches="tight")

    (args.out_dir / "step2_grasp_window_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    for run, m in summary.items():
        print(f"[grasp_window] {run} ({m['label']}): "
              f"held={m.get('held_steps', 0)} "
              f"gap_med={m.get('gap_median_mm', 0):.1f}mm "
              f"gap_range=[{m.get('gap_min_mm', 0):.1f},{m.get('gap_max_mm', 0):.1f}]mm "
              f"F_L={m.get('force_left_median_n', 0):.1f}N(p95={m.get('force_left_p95_n', 0):.1f}) "
              f"F_R={m.get('force_right_median_n', 0):.1f}N "
              f"contact L={m.get('left_contact_ratio', 0):.0%}/R={m.get('right_contact_ratio', 0):.0%}")


if __name__ == "__main__":
    main()
