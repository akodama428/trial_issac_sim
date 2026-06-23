"""Backward-compatible import wrapper for the canonical Isaac scene runtime."""

from tomato_harvest_sim.simulator.scene_runtime import (
    IsaacSceneRuntime,
    SceneRuntimeState,
)

SimulatorState = SceneRuntimeState
SimulatorRuntime = IsaacSceneRuntime

__all__ = [
    "IsaacSceneRuntime",
    "SceneRuntimeState",
    "SimulatorRuntime",
    "SimulatorState",
]
