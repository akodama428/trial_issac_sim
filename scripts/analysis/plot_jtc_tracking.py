"""controller_stateのreference/feedbackをphase行付きper-jointサブプロットで描く。

使い方 (extract_jtc_tracking_bag.pyの出力ディレクトリを渡す):
    python3 scripts/analysis/plot_jtc_tracking.py <data_dir>
"""
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt

plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]

OUT_DIR = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
SURFACE = "#fcfcfb"
INK = "#33322e"
INK_MUTED = "#77766e"
GRID = "#e8e8e5"
PHASE_LINE = "#c9c8c2"
REF_COLOR = "#2a78d6"   # 目標 (categorical slot 1)
FB_COLOR = "#1baf7a"    # 実測 (categorical slot 2)

with open(OUT_DIR / "controller_state.csv") as f:
    rows = list(csv.reader(f))
header, data = rows[0], rows[1:]
joints = [name[: -len("_ref")] for name in header[1:8]]
t0 = float(data[0][0])
t = [float(r[0]) - t0 for r in data]
ref = [[float(r[1 + j]) for r in data] for j in range(7)]
fb = [[float(r[8 + j]) for r in data] for j in range(7)]

with open(OUT_DIR / "phase.csv") as f:
    phase_rows = list(csv.reader(f))[1:]
phase_t = [float(r[0]) - t0 for r in phase_rows]
phase_names = [r[1] for r in phase_rows]
t_end = t[-1]

fig, axes = plt.subplots(
    8, 1, sharex=True, figsize=(14, 16),
    gridspec_kw={"height_ratios": [1.4] + [1] * 7},
)
fig.patch.set_facecolor(SURFACE)

place_span = None
for i, name in enumerate(phase_names):
    if name == "moving_to_place":
        end = phase_t[i + 1] if i + 1 < len(phase_t) else t_end
        place_span = (phase_t[i], end)

ax_phase = axes[0]
steps_t = phase_t + [t_end]
steps_y = list(range(len(phase_names))) + [len(phase_names) - 1]
ax_phase.step(steps_t, steps_y, where="post", color=INK, linewidth=2)
ax_phase.set_yticks(range(len(phase_names)))
ax_phase.set_yticklabels(phase_names, fontsize=8, color=INK)
ax_phase.set_ylabel("phase", fontsize=10, color=INK)

for ax, name, ref_y, fb_y in zip(axes[1:], joints, ref, fb):
    ax.plot(t, ref_y, color=REF_COLOR, linewidth=2, label="目標 (reference)")
    ax.plot(t, fb_y, color=FB_COLOR, linewidth=2, label="実測 (feedback)")
    ax.set_ylabel(f"{name}\n[rad]", fontsize=9, color=INK)

for ax in axes:
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.tick_params(colors=INK_MUTED, labelsize=8)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(INK_MUTED)
    for x in phase_t:
        ax.axvline(x, color=PHASE_LINE, linewidth=0.8, zorder=0)
    if place_span is not None:
        ax.axvspan(*place_span, color="#000000", alpha=0.04, zorder=0)

axes[-1].set_xlabel("time from bag start [s]", fontsize=10, color=INK)
axes[1].legend(
    loc="upper right", fontsize=9, frameon=False, labelcolor=INK, ncols=2
)
axes[0].set_title(
    "JTC 目標軌道 vs 実関節角 (freeze_bag, 2026-07-17 09:58, 93s / "
    "moving_to_place区間を網掛け)",
    fontsize=12, color=INK, pad=12,
)
fig.align_ylabels(axes)
fig.tight_layout()
fig.savefig(OUT_DIR / "joint_tracking.png", dpi=120, facecolor=SURFACE)
print(OUT_DIR / "joint_tracking.png")
