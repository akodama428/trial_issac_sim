from __future__ import annotations

import os
from dataclasses import dataclass


class IsaacRuntimeUnavailable(RuntimeError):
    """Raised when Isaac Sim Python modules are not available."""


@dataclass(frozen=True)
class IsaacLaunchPlan:
    runtime_mode: str
    container_entrypoint: str
    notes: tuple[str, ...]


def build_launch_plan() -> IsaacLaunchPlan:
    runtime = os.environ.get("POC_RUNTIME", "mock")
    return IsaacLaunchPlan(
        runtime_mode=runtime,
        container_entrypoint="./python.sh scripts/run_poc.py --mode isaac",
        notes=(
            "Container should source ROS 2 Jazzy before launching Isaac Sim.",
            "Isaac mode is intended to open a native Isaac Sim 3DView with Start/Stop/Reset controls.",
            "When Isaac libraries are unavailable, local verification falls back to mock mode.",
        ),
    )


def require_isaac_modules() -> None:
    try:
        __import__("omni")
    except ImportError as exc:  # pragma: no cover - exercised only in Isaac environment
        raise IsaacRuntimeUnavailable(
            "Isaac Sim Python modules are not available in this environment. "
            "Use mock mode locally or run inside the Isaac Sim container."
        ) from exc
