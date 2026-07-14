from __future__ import annotations

import inspect
from pathlib import Path

from tomato_harvest_sim.robot.motion_planner import build_planner
from tomato_harvest_sim.robot.motion_planner.pregrasp_planner import MoveItStylePreGraspPlanner


ROOT = Path(__file__).resolve().parents[1]


def test_planners_do_not_accept_fixed_grasp_lateral_offset() -> None:
    assert "grasp_lateral_offset_m" not in inspect.signature(MoveItStylePreGraspPlanner).parameters
    assert "grasp_lateral_offset_m" not in inspect.signature(build_planner).parameters


def test_runtime_does_not_expose_fixed_grasp_lateral_offset() -> None:
    paths = (
        ROOT / "scripts/ci/run_e2e.sh",
        ROOT / "src/tomato_harvest_sim/robot/motion_planner/node.py",
        ROOT / "src/tomato_harvest_sim/robot/motion_planner/pregrasp_planner.py",
        ROOT / "src/tomato_harvest_sim/robot/motion_planner/moveit_service_bridge.py",
    )
    for path in paths:
        source = path.read_text(encoding="utf-8")
        assert "GRASP_LATERAL_OFFSET" not in source
        assert "grasp_lateral_offset" not in source
