"""物理観測ロジック — 接触力積の集計と茎張力の推定。

Step 0（観測基盤）で導入した読み取り専用モジュール。物理判定へは一切介入せず、
PhysX contact report の生データと剛体状態から、チューニングと検証レポートに
使う数値（finger 別接触力積・茎張力推定）を導出する。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, Sequence

_GRAVITY_M_S2 = 9.81


@dataclass(frozen=True)
class FingerContactImpulses:
    """1 物理ステップ内の finger 別接触力積の合計ノルム [N·s]。"""

    left_ns: float
    right_ns: float

    def merged_with(self, other: "FingerContactImpulses") -> "FingerContactImpulses":
        """同一ステップ内の複数バッチ報告を合算する。"""
        return FingerContactImpulses(
            left_ns=self.left_ns + other.left_ns,
            right_ns=self.right_ns + other.right_ns,
        )


def summarize_finger_contact_impulses(
    contact_headers: Iterable[object],
    contact_data: Sequence[object],
    *,
    finger_of_pair: Callable[[int, int], str | None],
) -> FingerContactImpulses:
    """contact report の生データから finger 別の接触力積合計を求める。

    把持力の定量観測が目的。力積 [N·s] を物理 dt で割れば平均法線力 [N] の
    近似になる（換算はレポート側で行う）。

    Args:
        contact_headers: ContactEventHeader 列。actor0 / actor1 /
            contact_data_offset / num_contact_data を持つこと。
        contact_data: ContactData 列。impulse（x, y, z 属性）を持つこと。
        finger_of_pair: actor ペアから "left" / "right" / None を返す判定関数。
            USD パス解決は呼び出し側の責務とし、本関数は純ロジックに保つ。

    Returns:
        finger 別の力積ノルム合計。finger 接触が無ければゼロ。
    """
    left_total = 0.0
    right_total = 0.0
    for header in contact_headers:
        finger = finger_of_pair(header.actor0, header.actor1)
        if finger is None:
            continue
        offset = int(header.contact_data_offset)
        count = int(header.num_contact_data)
        magnitude = 0.0
        for index in range(offset, offset + count):
            impulse = contact_data[index].impulse
            magnitude += (
                float(impulse.x) ** 2 + float(impulse.y) ** 2 + float(impulse.z) ** 2
            ) ** 0.5
        if finger == "left":
            left_total += magnitude
        else:
            right_total += magnitude
    return FingerContactImpulses(left_ns=left_total, right_ns=right_total)


def format_observation_line(
    *,
    timestamp_sec: float,
    tomato_status: str,
    gripper_closed: bool,
    grasp_joint_active: bool,
    impulses: FingerContactImpulses,
    tomato_speed_m_s: float,
    hand_distance_m: float,
    stem_distance_m: float,
    stem_tension_n: float,
    finger_gap_m: float = 0.0,
) -> str:
    """1 物理ステップ分の観測値を、プロットスクリプトが機械解析できる1行に整形する。

    フォーマットは `scripts/plot_physics_observation.py` と対で管理する。
    key=value の空白区切りで、キー順も固定とする。
    """
    return (
        "[PhysicsObs] "
        f"t={timestamp_sec:.3f} "
        f"status={tomato_status} "
        f"grip={int(gripper_closed)} "
        f"joint={int(grasp_joint_active)} "
        f"impL={impulses.left_ns:.6f} "
        f"impR={impulses.right_ns:.6f} "
        f"v={tomato_speed_m_s:.5f} "
        f"hand_d={hand_distance_m:.4f} "
        f"stem_d={stem_distance_m:.4f} "
        f"stemF={stem_tension_n:.4f} "
        f"gap={finger_gap_m:.4f}"
    )


def estimate_stem_tension_n(
    *,
    mass_kg: float,
    velocity_m_s: tuple[float, float, float],
    previous_velocity_m_s: tuple[float, float, float],
    dt_sec: float,
) -> float:
    """トマトの運動状態から stem joint に掛かる張力を近似する [N]。

    枝に吊られたトマトを質点とみなし、張力 ≒ m × |a − g| で推定する。
    Step 4（自然分離）の破断しきい値設計で「pull 時張力 ≫ 振動時張力」の
    分離余裕を確認するための観測値であり、判定には使わない。

    Args:
        mass_kg: トマト質量。
        velocity_m_s: 現ステップの速度ベクトル。
        previous_velocity_m_s: 前ステップの速度ベクトル。
        dt_sec: 物理ステップ幅。0 以下の場合は加速度 0 として自重のみ返す。

    Returns:
        張力の推定ノルム [N]。静止時は自重 m×g に一致する。
    """
    if dt_sec > 0.0:
        acceleration = tuple(
            (velocity_m_s[axis] - previous_velocity_m_s[axis]) / dt_sec
            for axis in range(3)
        )
    else:
        acceleration = (0.0, 0.0, 0.0)
    # 重力は -z。張力は「重力を打ち消しつつ加速させる力」なので a − g を用いる。
    force_x = mass_kg * acceleration[0]
    force_y = mass_kg * acceleration[1]
    force_z = mass_kg * (acceleration[2] + _GRAVITY_M_S2)
    return (force_x**2 + force_y**2 + force_z**2) ** 0.5
