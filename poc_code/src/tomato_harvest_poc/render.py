from __future__ import annotations

from html import escape

from .config import RuntimeConfig
from .model import Snapshot, SimulationStatus


def render_stage_items(snapshot: Snapshot) -> str:
    items = "".join(f"<li>{escape(item)}</li>" for item in snapshot.stage_items)
    return f"<ul>{items}</ul>"


def render_viewport_svg(snapshot: Snapshot, config: RuntimeConfig) -> str:
    tomato_x = 360 if not snapshot.visual.tomato_detached else 430
    tomato_y = 180 if not snapshot.visual.tomato_detached else 255
    stroke = "#facc15" if snapshot.visual.target_highlighted else "#ffffff"
    arm_x = 80 + int(snapshot.visual.arm_progress * 210)
    arm_color = "#2563eb" if snapshot.status != SimulationStatus.FAILED else "#dc2626"
    status_text = escape(snapshot.status.value)
    return f"""
<svg viewBox="0 0 640 360" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Viewport">
  <rect width="640" height="360" fill="#f7f5ef"/>
  <rect x="32" y="40" width="576" height="280" rx="24" fill="#f2efe6" stroke="#d6d3c6"/>
  <line x1="360" y1="70" x2="360" y2="260" stroke="#4b7f39" stroke-width="10" />
  <line x1="360" y1="110" x2="430" y2="95" stroke="#5d8a43" stroke-width="8" />
  <line x1="360" y1="150" x2="300" y2="135" stroke="#5d8a43" stroke-width="8" />
  <line x1="360" y1="190" x2="420" y2="210" stroke="#5d8a43" stroke-width="8" />
  <circle cx="{tomato_x}" cy="{tomato_y}" r="26" fill="#d94b38" stroke="{stroke}" stroke-width="6" />
  <text x="{tomato_x - 44}" y="{tomato_y - 36}" fill="#6b1d12" font-size="14">Target Tomato</text>
  <circle cx="170" cy="70" r="18" fill="#111827" />
  <line x1="170" y1="88" x2="260" y2="144" stroke="#111827" stroke-width="6" />
  <line x1="80" y1="270" x2="{arm_x}" y2="225" stroke="{arm_color}" stroke-width="16" stroke-linecap="round" />
  <line x1="{arm_x}" y1="225" x2="{arm_x + 60}" y2="200" stroke="{arm_color}" stroke-width="12" stroke-linecap="round" />
  <line x1="{arm_x + 60}" y1="200" x2="{arm_x + 88}" y2="{tomato_y}" stroke="{arm_color}" stroke-width="10" stroke-linecap="round" />
  <line x1="{arm_x + 88}" y1="{tomato_y}" x2="{arm_x + 100}" y2="{tomato_y - 12}" stroke="#111827" stroke-width="4" />
  <line x1="{arm_x + 88}" y1="{tomato_y}" x2="{arm_x + 100}" y2="{tomato_y + 12}" stroke="#111827" stroke-width="4" />
  <rect x="448" y="24" width="148" height="36" rx="18" fill="#ffffff" stroke="#d6d3c6" />
  <text x="462" y="47" fill="#111827" font-size="18">{status_text}</text>
</svg>
""".strip()


def render_camera_svg(snapshot: Snapshot) -> str:
    target = "#facc15" if snapshot.visual.target_highlighted else "#ffffff"
    tomato_y = 128 if snapshot.visual.tomato_detached else 148
    return f"""
<svg viewBox="0 0 320 240" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="Camera view">
  <rect width="320" height="240" fill="#1f2937"/>
  <rect x="20" y="18" width="280" height="204" rx="16" fill="#334155"/>
  <line x1="160" y1="44" x2="160" y2="196" stroke="#94a3b8" stroke-dasharray="6 6"/>
  <line x1="70" y1="120" x2="250" y2="120" stroke="#94a3b8" stroke-dasharray="6 6"/>
  <circle cx="178" cy="{tomato_y}" r="28" fill="#ef4444" stroke="{target}" stroke-width="6"/>
  <text x="98" y="48" fill="#e2e8f0" font-size="15">Eye-to-hand camera</text>
  <text x="118" y="92" fill="#f8fafc" font-size="14">Target Tomato</text>
  <text x="110" y="214" fill="#e2e8f0" font-size="14">{escape(snapshot.result_message)}</text>
</svg>
""".strip()
