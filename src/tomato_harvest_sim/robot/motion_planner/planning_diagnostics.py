"""planning失敗時の証跡保存 (Issue #28 改善1)。

suffix replanが`error_code=99999`等で失敗したとき、原因切り分けに必要な
「start stateの有効性・衝突ペア・goal種別・目標位置」をJSONとして残す。
E2E CIはこのディレクトリをartifactへ含め、goal sampling失敗と
start state不正のどちらが支配的かを後から判定できるようにする。
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

PLANNING_DIAGNOSTIC_DIR_ENV = "TOMATO_HARVEST_PLANNING_DIAGNOSTIC_DIR"


@dataclass(frozen=True)
class StateValidityReport:
    """start stateの有効性確認結果。

    Attributes:
        checked: 有効性serviceへ問い合わせできたか。Falseならvalid/contactsは不明。
        valid: MoveItが返したstart stateの有効性。未確認ならNone。
        contacts: 衝突していたbodyペア。"body1|body2" 形式。
    """

    checked: bool
    valid: bool | None = None
    contacts: tuple[str, ...] = ()


@dataclass(frozen=True)
class PlanningFailureDiagnostic:
    """1回のplanning失敗を再調査可能にする証跡。

    Attributes:
        captured_at_sec: 失敗を観測したepoch秒。
        phase: 失敗したharvest phase (文字列値)。
        goal_kind: goal指定の種別。"pose" (IKサンプリング要) か "joint"。
        reason: 失敗分類 (例: "motion_plan_error", "service_timeout")。
        error_code: MoveItのerror_code。応答が無い場合はNone。
        target_xyz_m: pose goal時の目標位置。joint goal時はNone。
        start_joint_names: planning開始状態の関節名。
        start_positions_rad: planning開始状態の関節角。
        start_state: start stateの有効性確認結果。
    """

    captured_at_sec: float
    phase: str
    goal_kind: str
    reason: str
    error_code: int | None
    target_xyz_m: tuple[float, float, float] | None
    start_joint_names: tuple[str, ...]
    start_positions_rad: tuple[float, ...]
    start_state: StateValidityReport


def diagnostics_directory(environ: Mapping[str, str]) -> Path | None:
    """環境変数から診断保存先を決める。未設定なら診断は無効 (None)。"""
    raw = environ.get(PLANNING_DIAGNOSTIC_DIR_ENV, "").strip()
    return Path(raw) if raw else None


def diagnostic_to_dict(diagnostic: PlanningFailureDiagnostic) -> dict[str, object]:
    """診断をJSON化可能なdictへ変換する。"""
    return {
        "captured_at_sec": diagnostic.captured_at_sec,
        "phase": diagnostic.phase,
        "goal_kind": diagnostic.goal_kind,
        "reason": diagnostic.reason,
        "error_code": diagnostic.error_code,
        "target_xyz_m": (
            list(diagnostic.target_xyz_m)
            if diagnostic.target_xyz_m is not None else None
        ),
        "start_joint_names": list(diagnostic.start_joint_names),
        "start_positions_rad": list(diagnostic.start_positions_rad),
        "start_state": {
            "checked": diagnostic.start_state.checked,
            "valid": diagnostic.start_state.valid,
            "contacts": list(diagnostic.start_state.contacts),
        },
    }


def save_planning_failure_diagnostic(
    diagnostic: PlanningFailureDiagnostic, directory: Path | None
) -> Path | None:
    """診断を1失敗=1ファイルのJSONとして保存する。

    診断保存はplanner本体の副作用であってはならないため、保存失敗
    (書き込み権限なし等) は例外にせずNoneで返し、呼び出し側がログに残す。

    Args:
        diagnostic: 保存する診断。
        directory: 保存先。Noneなら診断無効として何もしない。

    Returns:
        書き込んだファイルのpath。無効時・保存失敗時はNone。
    """
    if directory is None:
        return None
    try:
        directory.mkdir(parents=True, exist_ok=True)
        base_name = (
            f"planning_failure_{diagnostic.phase}_{diagnostic.goal_kind}_"
            f"{int(diagnostic.captured_at_sec * 1_000_000)}"
        )
        path = directory / f"{base_name}.json"
        sequence = 1
        while path.exists():
            path = directory / f"{base_name}_{sequence}.json"
            sequence += 1
        path.write_text(
            json.dumps(diagnostic_to_dict(diagnostic), sort_keys=True, indent=2),
            encoding="utf-8",
        )
        return path
    except OSError:
        return None
