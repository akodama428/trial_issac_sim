"""自由空間phase共通のsuffix replan pure policyと多重起動gate (Issue #12, Step 4)。

Step 3のplace限定suffix replanを、自由空間の移動phase全体
(MOVING_TO_PREGRASP / MOVING_TO_GRASP / MOVING_TO_PLACE) へ一般化する。
"""
from __future__ import annotations

from dataclasses import dataclass
from threading import Lock

from tomato_harvest_sim.msg.contracts import (
    HarvestMotionPlan,
    HarvestTaskPhase,
    JointStateSnapshot,
    JointTrajectory,
)

# suffix replanの対象phase。DETACHING は茎からの引き剥がしという接触支配区間で、
# global plannerの経路再計画より局所的な力・微修正が支配的なため、周期replan対象に
# しない (Issue #12 設計判断)。Step 6 の local planner 候補として残す。
# RETURNING_HOME も自由空間移動であり、abort反復がstep予算切れを招くため
# suffix replan / local補正の対象に含める (Issue #32)。
SUFFIX_REPLAN_PHASES: frozenset[HarvestTaskPhase] = frozenset({
    HarvestTaskPhase.MOVING_TO_PREGRASP,
    HarvestTaskPhase.MOVING_TO_GRASP,
    HarvestTaskPhase.MOVING_TO_PLACE,
    HarvestTaskPhase.RETURNING_HOME,
})

# phaseごとの残区間trajectoryを保持するHarvestMotionPlanのfield名。
SUFFIX_TRAJECTORY_FIELD_BY_PHASE: dict[HarvestTaskPhase, str] = {
    HarvestTaskPhase.MOVING_TO_PREGRASP: "pregrasp_joint_trajectory",
    HarvestTaskPhase.MOVING_TO_GRASP: "grasp_joint_trajectory",
    HarvestTaskPhase.MOVING_TO_PLACE: "place_joint_trajectory",
    HarvestTaskPhase.RETURNING_HOME: "home_joint_trajectory",
}


def suffix_trajectory(
    plan: HarvestMotionPlan, phase: HarvestTaskPhase
) -> JointTrajectory | None:
    """planから、phaseの残区間に対応するtrajectoryを取り出す。

    Args:
        plan: 対象のplan。
        phase: 残区間を選択するphase。

    Returns:
        phaseがsuffix replan対象ならその区間のtrajectory、対象外ならNone。
    """
    field = SUFFIX_TRAJECTORY_FIELD_BY_PHASE.get(phase)
    if field is None:
        return None
    trajectory = getattr(plan, field)
    return trajectory if isinstance(trajectory, JointTrajectory) else None


def terminal_joint_state_of_phase(
    plan: HarvestMotionPlan, phase: HarvestTaskPhase
) -> JointStateSnapshot | None:
    """採用済みplanのphase終端関節構成を、既知の有効goalとして取り出す (Issue #28 改善2)。

    採用済みtrajectoryの終端は一度planning・衝突チェックを通過した構成であり、
    abort後のsuffix replanでpose goalのIKサンプリングが全滅したときの
    関節空間goal fallbackに使える。

    Args:
        plan: 現在採用中のplan。
        phase: 終端構成を取り出すphase。

    Returns:
        phase残区間trajectoryの終端関節構成。対象外phaseやtrajectory欠落時はNone。
    """
    trajectory = suffix_trajectory(plan, phase)
    if trajectory is None or not trajectory.points:
        return None
    return JointStateSnapshot(
        joint_names=trajectory.joint_names,
        positions_rad=trajectory.points[-1].positions_rad,
    )


@dataclass(frozen=True)
class SuffixUpdateDecision:
    adopted: bool
    reason: str
    max_trajectory_delta_rad: float | None = None


class SuffixReplanGate:
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


def evaluate_suffix_update(
    *,
    phase: HarvestTaskPhase,
    current_plan: HarvestMotionPlan,
    candidate_plan: HarvestMotionPlan,
    minimum_endpoint_delta_rad: float = 0.02,
) -> SuffixUpdateDecision:
    """candidateのphase残区間trajectory差がgoal差し替えに値するか判定する。"""
    if phase not in SUFFIX_REPLAN_PHASES:
        return SuffixUpdateDecision(False, "rejected_unsupported_phase")
    candidate = suffix_trajectory(candidate_plan, phase)
    if candidate is None or not candidate.points:
        return SuffixUpdateDecision(False, "rejected_missing_suffix_trajectory")
    current = suffix_trajectory(current_plan, phase)
    if current is None or not current.points:
        return SuffixUpdateDecision(True, "adopted_missing_current_trajectory")
    delta = _boundary_trajectory_delta(current, candidate)
    if delta is None:
        return SuffixUpdateDecision(True, "adopted_incomparable_trajectory")
    if delta < minimum_endpoint_delta_rad:
        return SuffixUpdateDecision(
            False, "rejected_small_trajectory_delta", delta
        )
    return SuffixUpdateDecision(
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
