from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class SimulationStatus(str, Enum):
    IDLE = "Idle"
    LOADING = "Loading"
    READY = "Ready"
    APPROACHING = "Approaching"
    GRASPING = "Grasping"
    PULLING = "Pulling"
    DETACHED = "Detached"
    FAILED = "Failed"


class FailureReason(str, Enum):
    TARGET_LOST = "Target lost from view"
    GRASP_FAILED = "Grasp failed"
    DETACH_FAILED = "Pull motion finished but detach did not occur"
    COLLISION_LIMIT = "Collision or motion limit reached"


class ScenarioMode(str, Enum):
    SUCCESS = "success"
    GRASP_FAILED = "grasp_failed"
    DETACH_FAILED = "detach_failed"
    TARGET_LOST = "target_lost"


@dataclass(frozen=True)
class LogEntry:
    step: str
    message: str


@dataclass(frozen=True)
class VisualState:
    target_highlighted: bool = True
    tomato_detached: bool = False
    gripper_closed: bool = False
    arm_progress: float = 0.0
    attempts_completed: int = 0


@dataclass(frozen=True)
class Snapshot:
    status: SimulationStatus
    result_message: str
    target_label: str
    stage_items: tuple[str, ...]
    logs: tuple[LogEntry, ...] = field(default_factory=tuple)
    failure_reason: FailureReason | None = None
    visual: VisualState = field(default_factory=VisualState)
    instructions: tuple[str, ...] = (
        "1. Confirm target",
        "2. Press Harvest Start",
        "3. Check result",
    )
