"""phase開始時計画と自由空間suffix replanのpure policy・多重起動gate。

実行用JointTrajectoryは初期planで一括生成せず、各移動phaseの開始時に
最新joint stateから生成する。自由空間phaseではabort後も同じ経路を使う。
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

# phaseごとの実行用trajectoryを保持するHarvestMotionPlanのfield名。
PHASE_TRAJECTORY_FIELD_BY_PHASE: dict[HarvestTaskPhase, str] = {
    HarvestTaskPhase.MOVING_TO_PREGRASP: "pregrasp_joint_trajectory",
    HarvestTaskPhase.MOVING_TO_GRASP: "grasp_joint_trajectory",
    HarvestTaskPhase.DETACHING: "pull_joint_trajectory",
    HarvestTaskPhase.MOVING_TO_PLACE: "place_joint_trajectory",
    HarvestTaskPhase.RETURNING_HOME: "home_joint_trajectory",
}

# MoveItによる実行用trajectoryをphase開始時に生成する移動phase。
PHASE_ENTRY_PLANNING_PHASES: frozenset[HarvestTaskPhase] = frozenset(
    PHASE_TRAJECTORY_FIELD_BY_PHASE
)

# abort後にも同じphaseを再計画する自由空間phase。DETACHINGは接触支配区間のため、
# 開始時計画は行うがsuffix replan対象には含めない (Issue #12)。
SUFFIX_REPLAN_PHASES: frozenset[HarvestTaskPhase] = frozenset({
    HarvestTaskPhase.MOVING_TO_PREGRASP,
    HarvestTaskPhase.MOVING_TO_GRASP,
    HarvestTaskPhase.MOVING_TO_PLACE,
    HarvestTaskPhase.RETURNING_HOME,
})


def phase_trajectory(
    plan: HarvestMotionPlan, phase: HarvestTaskPhase
) -> JointTrajectory | None:
    """planからphaseに対応する実行用trajectoryを取り出す。

    Args:
        plan: 対象のplan。
        phase: trajectoryを選択するphase。

    Returns:
        phaseが計画対象ならその区間のtrajectory、対象外ならNone。
    """
    field = PHASE_TRAJECTORY_FIELD_BY_PHASE.get(phase)
    if field is None:
        return None
    trajectory = getattr(plan, field)
    return trajectory if isinstance(trajectory, JointTrajectory) else None


def should_plan_phase_on_entry(
    previous_phase: HarvestTaskPhase | None, phase: HarvestTaskPhase | None
) -> bool:
    """移動phase進入時に実行用trajectory計画を起動するか判定する。

    Args:
        previous_phase: 直前に観測していたphase。未観測ならNone。
        phase: 新しく観測したphase。

    Returns:
        計画対象の移動phaseへの遷移を新規に観測した場合のみTrue。
    """
    return phase in PHASE_ENTRY_PLANNING_PHASES and previous_phase is not phase


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
    trajectory = phase_trajectory(plan, phase)
    if trajectory is None or not trajectory.points:
        return None
    return JointStateSnapshot(
        joint_names=trajectory.joint_names,
        positions_rad=trajectory.points[-1].positions_rad,
    )


@dataclass(frozen=True)
class PhasePlanUpdateDecision:
    adopted: bool
    reason: str
    max_trajectory_delta_rad: float | None = None


@dataclass(frozen=True)
class PhasePlanRetryMemory:
    """直近のphase計画試行と、次に再試行できる時刻を保持する。"""

    phase: HarvestTaskPhase | None = None
    retry_after_sec: float = 0.0


def memory_after_phase_plan_attempt(
    *,
    phase: HarvestTaskPhase,
    now_sec: float,
    retry_interval_sec: float,
) -> PhasePlanRetryMemory:
    """phase計画試行後の再試行時刻を記録する。

    Args:
        phase: 試行した移動phase。
        now_sec: 試行完了時のmonotonic時刻。
        retry_interval_sec: 次回試行までの最小間隔。

    Returns:
        更新後の再試行memory。
    """
    return PhasePlanRetryMemory(
        phase=phase,
        retry_after_sec=now_sec + max(0.0, retry_interval_sec),
    )


def should_retry_missing_phase_plan(
    *,
    phase: HarvestTaskPhase | None,
    plan: HarvestMotionPlan | None,
    memory: PhasePlanRetryMemory,
    now_sec: float,
) -> bool:
    """現在phaseの実行軌道が欠落している場合だけ再計画を許可する。

    Args:
        phase: 現在のtask phase。
        plan: 最新のpose/trajectory plan。
        memory: 直近のphase計画試行記録。
        now_sec: 判定時のmonotonic時刻。

    Returns:
        最新状態からphase計画を再試行すべき場合はTrue。
    """
    if phase not in PHASE_ENTRY_PLANNING_PHASES or plan is None:
        return False
    if (
        plan.planned_from_phase is phase
        and phase_trajectory(plan, phase) is not None
    ):
        return False
    if memory.phase is not phase:
        return True
    return now_sec >= memory.retry_after_sec


class PhasePlanningGate:
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


def evaluate_phase_plan_update(
    *,
    phase: HarvestTaskPhase,
    current_plan: HarvestMotionPlan,
    candidate_plan: HarvestMotionPlan,
    minimum_endpoint_delta_rad: float = 0.02,
) -> PhasePlanUpdateDecision:
    """candidateのphase trajectoryを新しい実行計画として採用するか判定する。"""
    if phase not in PHASE_ENTRY_PLANNING_PHASES:
        return PhasePlanUpdateDecision(False, "rejected_unsupported_phase")
    candidate = phase_trajectory(candidate_plan, phase)
    if candidate is None or not candidate.points:
        return PhasePlanUpdateDecision(False, "rejected_missing_phase_trajectory")
    current = phase_trajectory(current_plan, phase)
    if current is None or not current.points:
        return PhasePlanUpdateDecision(True, "adopted_missing_current_trajectory")
    delta = _boundary_trajectory_delta(current, candidate)
    if delta is None:
        return PhasePlanUpdateDecision(True, "adopted_incomparable_trajectory")
    if delta < minimum_endpoint_delta_rad:
        return PhasePlanUpdateDecision(
            False, "rejected_small_trajectory_delta", delta
        )
    return PhasePlanUpdateDecision(
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
