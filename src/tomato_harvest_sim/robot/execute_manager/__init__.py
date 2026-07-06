"""execute_manager パッケージ — 実行管理ノード群。"""
from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from tomato_harvest_sim.msg.contracts import (
        HarvestMotionPlan,
        HarvestTaskPhase,
        JointStateSnapshot,
        MotionCommand,
    )


def build_motion_command(
    phase: "HarvestTaskPhase",
    plan: "HarvestMotionPlan",
    current_joints: "JointStateSnapshot",
) -> "MotionCommand":
    from tomato_harvest_sim.robot.execute_manager.motion_command import (
        build_motion_command as impl,
    )
    return impl(phase, plan, current_joints)


def main_motion_command() -> None:
    from tomato_harvest_sim.robot.execute_manager.motion_command import main
    main()


def trajectory_status_from_execution_status(execution_status: str) -> str:
    from tomato_harvest_sim.robot.execute_manager.trajectory_monitor import (
        trajectory_status_from_execution_status as impl,
    )
    return impl(execution_status)


def main_trajectory_monitor() -> None:
    from tomato_harvest_sim.robot.execute_manager.trajectory_monitor import main
    main()

__all__ = [
    "build_motion_command",
    "main_motion_command",
    "main_trajectory_monitor",
    "trajectory_status_from_execution_status",
]
