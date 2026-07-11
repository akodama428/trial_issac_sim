"""MOVING_TO_PLACE専用suffix replanのpure policyと多重起動gate。"""
from __future__ import annotations

from dataclasses import dataclass
from threading import Lock

from tomato_harvest_sim.msg.contracts import HarvestMotionPlan, JointTrajectory


@dataclass(frozen=True)
class PlaceSuffixUpdateDecision:
    adopted: bool
    reason: str
    max_trajectory_delta_rad: float | None = None


class PlaceSuffixReplanGate:
    """planner実行中の二重起動をthread-safeに抑止する。"""

    def __init__(self) -> None:
        self._lock = Lock()
        self._in_flight = False

    def try_begin(self) -> bool:
        """未実行なら実行権を取得し、取得可否を返す。"""
        with self._lock:
            if self._in_flight:
                return False
            self._in_flight = True
            return True

    def finish(self) -> None:
        """planner実行権を解放する。"""
        with self._lock:
            self._in_flight = False


def evaluate_place_suffix_update(
    *,
    current_plan: HarvestMotionPlan,
    candidate_plan: HarvestMotionPlan,
    minimum_endpoint_delta_rad: float = 0.02,
) -> PlaceSuffixUpdateDecision:
    """candidateの軌道差がgoal差し替えに値するか判定する。"""
    candidate = candidate_plan.place_joint_trajectory
    if candidate is None or not candidate.points:
        return PlaceSuffixUpdateDecision(False, "rejected_missing_place_trajectory")
    current = current_plan.place_joint_trajectory
    if current is None or not current.points:
        return PlaceSuffixUpdateDecision(True, "adopted_missing_current_trajectory")
    delta = _boundary_trajectory_delta(current, candidate)
    if delta is None:
        return PlaceSuffixUpdateDecision(True, "adopted_incomparable_trajectory")
    if delta < minimum_endpoint_delta_rad:
        return PlaceSuffixUpdateDecision(
            False, "rejected_small_trajectory_delta", delta
        )
    return PlaceSuffixUpdateDecision(
        True, "adopted_significant_trajectory_delta", delta
    )


def _boundary_trajectory_delta(
    current: JointTrajectory, candidate: JointTrajectory
) -> float | None:
    """開始点と終端点の共通関節に対する最大差分を返す。"""
    common = tuple(name for name in current.joint_names if name in candidate.joint_names)
    if not common:
        return None
    current_index = {name: index for index, name in enumerate(current.joint_names)}
    candidate_index = {name: index for index, name in enumerate(candidate.joint_names)}
    deltas = []
    for current_point, candidate_point in (
        (current.points[0], candidate.points[0]),
        (current.points[-1], candidate.points[-1]),
    ):
        deltas.extend(
            abs(
                current_point.positions_rad[current_index[name]]
                - candidate_point.positions_rad[candidate_index[name]]
            )
            for name in common
        )
    return max(deltas)
