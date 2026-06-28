#!/usr/bin/env python3
"""hoge.log から関節角度（目標値 vs 実値）を時系列プロット"""

import re
import sys
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

LOG_FILE = "hoge.log"
N_JOINTS = 7
JOINT_NAMES = [f"J{i+1}" for i in range(N_JOINTS)]

# フェーズ遷移マーカー色
PHASE_COLORS = {
    "detecting": "#aaaaaa",
    "target_found": "#aaaaaa",
    "planning": "#aaaaaa",
    "moving_to_pregrasp": "#4488ff",
    "pregrasp_reached": "#4488ff",
    "moving_to_grasp": "#ff8800",
    "at_grasp": "#ff8800",
    "grasp_evaluation": "#ff8800",
    "detaching": "#cc00cc",
    "detached": "#cc00cc",
    "moving_to_place": "#cc0000",
    "placed": "#00aa00",
    "returning_home": "#00aa00",
    "complete": "#00aa00",
}

_RE_TRAJ = re.compile(
    r"context=(\w+)\s+command_q=\[([^\]]+)\]\s+readback_q=\[([^\]]+)\]"
)
_RE_PHASE = re.compile(r"\[State\]\s+\w+\s+->\s+(\w+)")
_RE_REPLAN = re.compile(r"\[Replan\]")
_RE_LINENO = re.compile(r"^\[?(\d+)\]?")


def _parse_vec(s: str) -> list[float]:
    return [float(x) for x in s.split(",")]


def parse_log(path: str) -> tuple[list, list, list]:
    """
    Returns:
        entries: list of (step, context, command_q, readback_q)
        phases:  list of (step, phase_name)
        replans: list of step
    """
    entries = []
    phases = []
    replans = []
    step = 0

    with open(path) as f:
        for raw in f:
            line = raw.strip()

            m = _RE_TRAJ.search(line)
            if m:
                context = m.group(1)
                cmd = _parse_vec(m.group(2))
                rbk = _parse_vec(m.group(3))
                entries.append((step, context, cmd, rbk))
                step += 1
                continue

            m = _RE_PHASE.search(line)
            if m:
                phases.append((step, m.group(1)))
                continue

            if _RE_REPLAN.search(line):
                replans.append(step)

    return entries, phases, replans


def plot(entries, phases, replans, *, context_filter=None, title="Joint angle: command vs actual"):
    steps  = [e[0] for e in entries if context_filter is None or e[1] == context_filter]
    cmds   = [[e[2][j] for e in entries if context_filter is None or e[1] == context_filter] for j in range(N_JOINTS)]
    actuals= [[e[3][j] for e in entries if context_filter is None or e[1] == context_filter] for j in range(N_JOINTS)]

    if not steps:
        print(f"データなし (context_filter={context_filter})")
        return

    fig, axes = plt.subplots(N_JOINTS, 1, figsize=(14, 16), sharex=True)
    fig.suptitle(title, fontsize=12)

    for j, ax in enumerate(axes):
        ax.plot(steps, cmds[j],    "b-", lw=0.8, alpha=0.8, label="command")
        ax.plot(steps, actuals[j], "r-", lw=0.8, alpha=0.8, label="actual")
        ax.fill_between(
            steps,
            cmds[j], actuals[j],
            alpha=0.15, color="red",
        )
        ax.set_ylabel(JOINT_NAMES[j], fontsize=8)
        ax.yaxis.set_major_formatter(ticker.FormatStrFormatter("%.2f"))
        ax.grid(True, lw=0.4)
        if j == 0:
            ax.legend(loc="upper right", fontsize=7)

    # フェーズ境界を縦線で表示
    phase_steps_all = [s for s, _ in phases]
    for ax in axes:
        for ps, pname in phases:
            color = PHASE_COLORS.get(pname, "#888888")
            ax.axvline(ps, color=color, lw=0.8, alpha=0.6)
        for rs in replans:
            ax.axvline(rs, color="orange", lw=0.5, alpha=0.4, ls="--")

    # フェーズ名ラベルを最上段に表示
    ax0 = axes[0]
    prev_ps = 0
    for i, (ps, pname) in enumerate(phases):
        mid = (prev_ps + ps) / 2
        ax0.text(mid, ax0.get_ylim()[1], pname, fontsize=5, rotation=45,
                 ha="right", va="bottom", color=PHASE_COLORS.get(pname, "#888888"))
        prev_ps = ps

    axes[-1].set_xlabel("step")
    fig.tight_layout()
    plt.show()


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else LOG_FILE
    print(f"解析中: {path}")
    entries, phases, replans = parse_log(path)
    print(f"  軌道ステップ数: {len(entries)}")
    print(f"  フェーズ遷移数: {len(phases)}  {[p for _, p in phases]}")
    print(f"  リプラン回数: {len(replans)}")

    # Full timeline
    plot(entries, phases, replans, title="All phases: command vs actual")

    # Zoom into moving_to_place
    place_start = next((s for s, p in phases if p == "moving_to_place"), None)
    place_end   = next((s for s, p in phases if p == "placed"), None)
    if place_start is not None:
        place_entries = [e for e in entries if place_start <= e[0] <= (place_end or 999999)]
        place_replans = [r for r in replans if place_start <= r <= (place_end or 999999)]
        if place_entries:
            plot(
                place_entries,
                [(s, p) for s, p in phases if place_start <= s <= (place_end or 999999)],
                place_replans,
                title="moving_to_place: command vs actual  (orange dashed = replan)",
            )


if __name__ == "__main__":
    main()
