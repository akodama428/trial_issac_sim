"""Backward-compatible import wrapper for the canonical application composition root."""

from tomato_harvest_sim.app.application import (
    TomatoHarvestApplication,
    create_tomato_harvest_application,
)

SprintOneSystem = TomatoHarvestApplication
create_sprint_one_system = create_tomato_harvest_application

__all__ = [
    "TomatoHarvestApplication",
    "create_tomato_harvest_application",
    "SprintOneSystem",
    "create_sprint_one_system",
]
