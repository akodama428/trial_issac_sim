#!/usr/bin/env python3
"""Step 1 plan 契約の stale plan 抑止を再現・可視化する (Issue #9)。

Step 0 で観測した abort 起点 replan の遅延分散 (86〜768 ms) を前提に、
phase が先へ進んだ後に古い plan が届く再現シナリオを、実装そのものの
`evaluate_plan_adoption` へ通して、旧契約 (無条件採用) と新契約の採用結果を
比較する。結果は JSON と PNG として step1_artifacts へ出力する。
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from tomato_harvest_sim.msg.contracts import (  # noqa: E402
    HarvestMotionPlan,
    HarvestTaskPhase,
    PlanProducerKind,
    Pose3D,
)
from tomato_harvest_sim.robot.execute_manager.plan_adoption import (  # noqa: E402
    evaluate_plan_adoption,
)

_POSE = Pose3D(x=0.1, y=0.2, z=0.3, roll=0.0, pitch=0.0, yaw=0.0)

# 参照パレット (dataviz skill validated defaults)
COLOR_ADOPTED = "#2a78d6"      # categorical blue: 採用された plan
COLOR_STALE_ADOPTED = "#d03b3b"  # status critical: stale なのに採用 (旧契約の欠陥)
COLOR_REJECTED = "#52514e"     # secondary ink: 棄却 (新契約の抑止)
COLOR_GRID = "#d8d7d2"


@dataclass(frozen=True)
class PlanArrival:
    """consumer に plan が届いた1イベントと、その期待判定 (シナリオ仕様)。"""

    label: str
    arrival_time_sec: float
    plan_revision: int
    planned_from_phase: HarvestTaskPhase
    consumer_phase: HarvestTaskPhase
    expected_stale: bool
    note: str


# Step 0 実測に基づく再現シナリオ:
# - 初回 full-chain plan は target_found 起点
# - moving_to_grasp の abort 起点 replan は latency 分散が大きく (86〜768 ms)、
#   遅い replan や QoS 再配送は phase が進んだ後に届き得る
# expected_stale はシナリオ仕様としての期待値であり、実装の判定結果とは独立に定義する。
SCENARIO: tuple[PlanArrival, ...] = (
    PlanArrival("A", 0.0, 1, HarvestTaskPhase.TARGET_FOUND,
                HarvestTaskPhase.PLANNING, False, "初回 full-chain plan"),
    PlanArrival("B", 4.2, 2, HarvestTaskPhase.MOVING_TO_GRASP,
                HarvestTaskPhase.MOVING_TO_GRASP, False, "abort 起点 replan (86ms)"),
    PlanArrival("C", 4.5, 2, HarvestTaskPhase.MOVING_TO_GRASP,
                HarvestTaskPhase.MOVING_TO_GRASP, True, "同一 revision の再配送"),
    PlanArrival("D", 7.8, 3, HarvestTaskPhase.MOVING_TO_GRASP,
                HarvestTaskPhase.DETACHING, True, "遅い replan (768ms) が phase 通過後に到着"),
    PlanArrival("E", 12.0, 4, HarvestTaskPhase.MOVING_TO_PLACE,
                HarvestTaskPhase.MOVING_TO_PLACE, False, "place replan"),
    PlanArrival("F", 12.6, 3, HarvestTaskPhase.MOVING_TO_GRASP,
                HarvestTaskPhase.MOVING_TO_PLACE, True, "古い revision の遅延再配送"),
)


def _plan_for(arrival: PlanArrival) -> HarvestMotionPlan:
    return HarvestMotionPlan(
        planner_name="moveit2_service_bridge",
        target_pose=_POSE,
        pregrasp_pose=_POSE,
        grasp_pose=_POSE,
        pull_pose=_POSE,
        place_pose=_POSE,
        plan_revision=arrival.plan_revision,
        generated_at_sec=arrival.arrival_time_sec,
        planned_from_phase=arrival.planned_from_phase,
        producer_kind=PlanProducerKind.GLOBAL_PLANNER,
    )


def replay_scenario() -> list[dict[str, object]]:
    """シナリオを新旧両契約で再生し、イベントごとの採用結果を返す。

    Returns:
        イベントごとの dict。legacy_adopted は旧契約 (無条件採用)、
        step1_adopted / step1_reason は実装の evaluate_plan_adoption の結果。
    """
    results: list[dict[str, object]] = []
    current_plan: HarvestMotionPlan | None = None
    for arrival in SCENARIO:
        candidate = _plan_for(arrival)
        decision = evaluate_plan_adoption(
            candidate=candidate,
            current_plan=current_plan,
            current_phase=arrival.consumer_phase,
        )
        if decision.adopted:
            current_plan = candidate
        results.append({
            "label": arrival.label,
            "arrival_time_sec": arrival.arrival_time_sec,
            "plan_revision": arrival.plan_revision,
            "planned_from_phase": arrival.planned_from_phase.value,
            "consumer_phase": arrival.consumer_phase.value,
            "note": arrival.note,
            "expected_stale": arrival.expected_stale,
            "legacy_adopted": True,  # 旧契約は届いた plan を無条件に採用していた
            "step1_adopted": decision.adopted,
            "step1_reason": decision.reason,
        })
    return results


def stale_adoption_counts(results: list[dict[str, object]]) -> dict[str, int]:
    """シナリオ仕様上 stale な plan を採用してしまった数を契約別に数える。"""
    stale_events = [r for r in results if r["expected_stale"]]
    return {
        "legacy_contract": sum(1 for r in stale_events if r["legacy_adopted"]),
        "step1_contract": sum(1 for r in stale_events if r["step1_adopted"]),
        "stale_event_total": len(stale_events),
    }


def _configure_matplotlib():
    """日本語ラベルを含むため CJK フォントがあれば優先して使う。"""
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import font_manager
    available = {font.name for font in font_manager.fontManager.ttflist}
    for name in ("Noto Sans CJK JP", "IPAexGothic", "TakaoGothic", "Droid Sans Fallback"):
        if name in available:
            matplotlib.rcParams["font.family"] = name
            break
    import matplotlib.pyplot as plt
    return plt


def _render_timeline(results: list[dict[str, object]], output_dir: Path) -> None:
    plt = _configure_matplotlib()

    figure, axis = plt.subplots(figsize=(9.5, 4.2))
    lanes = {"旧契約 (無条件採用)": 1.0, "新契約 (Step 1 採用規則)": 0.0}

    for index, result in enumerate(results):
        legacy_color = COLOR_STALE_ADOPTED if result["expected_stale"] else COLOR_ADOPTED
        axis.scatter(index, 1.0, s=140, color=legacy_color, zorder=3)
        if bool(result["step1_adopted"]):
            axis.scatter(index, 0.0, s=140, color=COLOR_ADOPTED, zorder=3)
        else:
            axis.scatter(index, 0.0, s=140, color=COLOR_REJECTED, marker="X", zorder=3)
        axis.annotate(
            f"rev{result['plan_revision']}\n"
            f"from={result['planned_from_phase']}\n@{result['consumer_phase']}",
            (index, 0.5), ha="center", va="center", fontsize=7.5, color="#0b0b0b",
        )

    for lane_y in lanes.values():
        axis.axhline(lane_y, color=COLOR_GRID, linewidth=1, zorder=1)

    axis.set_xticks(
        range(len(results)),
        [f"{r['label']}\nt={r['arrival_time_sec']}s" for r in results],
        fontsize=8,
    )
    axis.set_xlim(-0.6, len(results) - 0.4)

    handles = [
        plt.Line2D([], [], linestyle="", marker="o", color=COLOR_ADOPTED,
                   label="採用 (新鮮な plan)"),
        plt.Line2D([], [], linestyle="", marker="o", color=COLOR_STALE_ADOPTED,
                   label="stale plan を誤採用 (旧契約)"),
        plt.Line2D([], [], linestyle="", marker="X", color=COLOR_REJECTED,
                   label="stale plan を棄却 (新契約)"),
    ]
    axis.set_yticks(list(lanes.values()), list(lanes.keys()))
    axis.set(
        xlabel="plan 到着順 (ラベルは到着イベントとシナリオ時刻)",
        title="plan 到着タイムライン: 旧契約と Step 1 契約の採用結果比較",
    )
    axis.set_ylim(-0.55, 1.55)
    axis.spines[["top", "right", "left"]].set_visible(False)
    axis.legend(handles=handles, loc="upper left", fontsize=8, frameon=False)
    figure.tight_layout()
    figure.savefig(output_dir / "plan_adoption_timeline.png", dpi=150)
    plt.close(figure)


def _render_comparison(results: list[dict[str, object]], output_dir: Path) -> None:
    plt = _configure_matplotlib()

    counts = stale_adoption_counts(results)
    fresh_total = sum(1 for r in results if r["step1_adopted"])

    figure, axis = plt.subplots(figsize=(6.4, 4.0))
    contracts = ["旧契約\n(無条件採用)", "新契約\n(Step 1 採用規則)"]
    fresh = [fresh_total, fresh_total]
    stale = [counts["legacy_contract"], counts["step1_contract"]]
    x = range(len(contracts))
    width = 0.34
    axis.bar([i - width / 2 for i in x], fresh, width,
             color=COLOR_ADOPTED, label="新鮮な plan の採用")
    bars = axis.bar([i + width / 2 for i in x], stale, width,
                    color=COLOR_STALE_ADOPTED, label="stale plan の採用 (少ないほど良い)")
    for bar, value in zip(bars, stale):
        axis.annotate(str(value), (bar.get_x() + bar.get_width() / 2, value),
                      ha="center", va="bottom", fontsize=11, color="#0b0b0b")
    axis.set_xticks(list(x), contracts)
    axis.set(
        ylabel="採用された plan 数 [件]",
        title=f"stale plan 採用数の比較 (stale 到着 {counts['stale_event_total']} 件中)",
    )
    axis.set_ylim(0, max(fresh_total, counts["legacy_contract"]) + 1)
    axis.spines[["top", "right"]].set_visible(False)
    axis.legend(fontsize=8, frameon=False)
    figure.tight_layout()
    figure.savefig(output_dir / "stale_adoption_comparison.png", dpi=150)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("docs/reports/moveit_replanning/step1_artifacts"),
    )
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    results = replay_scenario()
    summary = {
        "scenario_events": results,
        "stale_adoption_counts": stale_adoption_counts(results),
    }
    (args.output_dir / "plan_adoption_scenario.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    _render_timeline(results, args.output_dir)
    _render_comparison(results, args.output_dir)
    print(json.dumps(summary["stale_adoption_counts"], ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
