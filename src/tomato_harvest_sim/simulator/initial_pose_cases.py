"""Issue #28で継続計測する再現可能なFranka初期関節姿勢。"""
from __future__ import annotations

import math
from dataclasses import dataclass


PANDA_JOINT_LIMITS_RAD = (
    (-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973),
    (-3.0718, -0.0698), (-2.8973, 2.8973), (-0.0175, 3.7525),
    (-2.8973, 2.8973),
)


@dataclass(frozen=True)
class InitialPoseCase:
    case_id: str
    positions_rad: tuple[float, ...]
    description: str
    is_singularity_case: bool = False


INITIAL_POSE_CASES = (
    InitialPoseCase("default", (0.0, -0.4, 0.0, -2.1, 0.0, 1.7, 0.8), "現在の標準姿勢"),
    InitialPoseCase("elbow_left", (0.35, -0.55, -0.25, -2.0, 0.20, 1.55, 0.55), "左寄りの肘構成"),
    InitialPoseCase("elbow_right", (-0.35, -0.55, 0.25, -2.0, -0.20, 1.55, 1.05), "右寄りの肘構成"),
    InitialPoseCase("shoulder_high", (0.10, 0.15, -0.15, -1.65, 0.10, 1.85, 0.70), "肩を高くした構成"),
    InitialPoseCase("shoulder_low", (-0.10, -0.85, 0.15, -2.35, -0.10, 1.45, 0.90), "肩を低くした構成"),
    InitialPoseCase("wrist_left", (0.15, -0.45, -0.10, -2.05, 0.65, 1.65, 0.25), "手首を左へ回した構成"),
    InitialPoseCase("wrist_right", (-0.15, -0.45, 0.10, -2.05, -0.65, 1.65, 1.35), "手首を右へ回した構成"),
    InitialPoseCase("folded_near", (0.0, -0.15, 0.0, -2.65, 0.0, 2.45, 0.80), "折り畳んだ近距離構成"),
    InitialPoseCase("extended_far", (0.20, -0.25, -0.15, -1.05, 0.10, 0.75, 0.50), "伸展した遠距離構成"),
    InitialPoseCase(
        "near_singularity_extended",
        (0.0, -0.05, 0.0, -0.10, 0.0, 0.15, 0.0),
        "肩・肘・手首軸が整列する伸展特異姿勢近傍。回復能力を計測する評価ケース",
        True,
    ),
)


def validate_cases(cases: tuple[InitialPoseCase, ...]) -> tuple[str, ...]:
    """関節数・有限値・Panda関節制限を検査し、違反理由を返す。"""
    errors: list[str] = []
    for case in cases:
        if len(case.positions_rad) != len(PANDA_JOINT_LIMITS_RAD):
            errors.append(f"{case.case_id}: expected 7 joints")
            continue
        for index, (value, limits) in enumerate(zip(
            case.positions_rad, PANDA_JOINT_LIMITS_RAD, strict=True
        ), start=1):
            if not math.isfinite(value) or not limits[0] <= value <= limits[1]:
                errors.append(f"{case.case_id}: joint{index} outside limits")
    return tuple(errors)


def initial_pose_from_environment(case_id: str) -> tuple[float, ...]:
    """固定IDから初期姿勢を取得し、未知IDをfail-fastにする。"""
    for case in INITIAL_POSE_CASES:
        if case.case_id == case_id:
            return case.positions_rad
    raise ValueError(f"Unknown TOMATO_HARVEST_INITIAL_POSE_ID: {case_id}")
