#!/usr/bin/env python3
"""ベースライン E2E 複数回分のサマリ JSON を集計し、比較グラフを生成する。

plot_physics_observation.py が出力した <prefix>_summary.json を複数受け取り、
Step 0 検証レポート用の集計グラフ2種を出力する。

出力:
  - <out-dir>/baseline_success.png        : run 別の成否と成功率
  - <out-dir>/baseline_phase_durations.png : run 別フェーズ所要時間の比較
  - <out-dir>/baseline_summary.json       : 集計値（成功率・平均所要時間）

使用例:
  python3 scripts/plot_baseline_summary.py \
      --summaries docs/reports/img/run*_summary.json --out-dir docs/reports/img
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def load_summaries(paths: list[Path]) -> list[dict]:
    return [json.loads(path.read_text(encoding="utf-8")) for path in sorted(paths)]


def render_baseline_plots(summaries: list[dict], out_dir: Path) -> dict:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. run 別の成否
    names = [s["prefix"] for s in summaries]
    successes = [bool(s["reached_complete"]) for s in summaries]
    success_rate = sum(successes) / len(successes) if successes else 0.0

    fig, ax = plt.subplots(figsize=(8, 3))
    colors = ["tab:green" if ok else "tab:red" for ok in successes]
    ax.bar(names, [1] * len(names), color=colors)
    for index, ok in enumerate(successes):
        ax.text(index, 0.5, "COMPLETE" if ok else "NOT COMPLETE",
                ha="center", va="center", color="white", fontweight="bold")
    ax.set_yticks([])
    ax.set_title(
        f"baseline E2E success: {sum(successes)}/{len(successes)} "
        f"(rate={success_rate:.0%})"
    )
    success_png = out_dir / "baseline_success.png"
    fig.savefig(success_png, dpi=110, bbox_inches="tight")
    plt.close(fig)

    # 2. run 別フェーズ所要時間（フェーズ名でグループ化した棒グラフ）
    phase_order: list[str] = []
    per_run_durations: list[dict[str, float]] = []
    for summary in summaries:
        durations: dict[str, float] = {}
        for entry in summary["phase_durations_sec"]:
            phase = entry["phase"]
            durations[phase] = durations.get(phase, 0.0) + float(entry["sec"])
            if phase not in phase_order:
                phase_order.append(phase)
        per_run_durations.append(durations)

    fig, ax = plt.subplots(figsize=(11, 4.5))
    width = 0.8 / max(1, len(summaries))
    for run_index, durations in enumerate(per_run_durations):
        offsets = [i + run_index * width for i in range(len(phase_order))]
        values = [durations.get(phase, 0.0) for phase in phase_order]
        ax.bar(offsets, values, width=width, label=names[run_index])
    ax.set_xticks([i + 0.4 - width / 2 for i in range(len(phase_order))])
    ax.set_xticklabels(phase_order, rotation=30, ha="right")
    ax.set_ylabel("duration [s]")
    ax.set_title("baseline phase durations per run")
    ax.legend()
    ax.grid(alpha=0.3, axis="y")
    durations_png = out_dir / "baseline_phase_durations.png"
    fig.savefig(durations_png, dpi=110, bbox_inches="tight")
    plt.close(fig)

    mean_durations = {
        phase: sum(d.get(phase, 0.0) for d in per_run_durations) / len(per_run_durations)
        for phase in phase_order
    } if per_run_durations else {}

    aggregate = {
        "runs": names,
        "success_count": sum(successes),
        "total_runs": len(successes),
        "success_rate": success_rate,
        "mean_phase_durations_sec": mean_durations,
        "images": {
            "success": str(success_png),
            "phase_durations": str(durations_png),
        },
    }
    (out_dir / "baseline_summary.json").write_text(
        json.dumps(aggregate, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return aggregate


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summaries", type=Path, nargs="+", required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    summaries = load_summaries(args.summaries)
    aggregate = render_baseline_plots(summaries, args.out_dir)
    print(
        f"[plot_baseline_summary] runs={aggregate['total_runs']} "
        f"success={aggregate['success_count']} rate={aggregate['success_rate']:.0%}"
    )


if __name__ == "__main__":
    main()
