"""物理観測ロジック（接触力積の集計・茎張力の推定）のテスト。

Step 0（観測基盤）: 観測は判定へ一切介入しない読み取り専用ロジックであり、
contact report の生データと剛体状態から人間が検証可能な数値を導出する。
"""
from __future__ import annotations

import unittest

from tomato_harvest_sim.simulator.physics_observation import (
    FingerContactImpulses,
    FingerContactForces,
    contact_forces_from_impulses,
    estimate_stem_tension_n,
    format_observation_line,
    summarize_finger_contact_impulses,
    summarize_matching_contact_impulse,
)


class _Vec3:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


class _Header:
    def __init__(
        self, actor0: int, actor1: int, offset: int, count: int,
        collider0: int | None = None, collider1: int | None = None,
    ) -> None:
        self.actor0 = actor0
        self.actor1 = actor1
        self.contact_data_offset = offset
        self.num_contact_data = count
        self.collider0 = actor0 if collider0 is None else collider0
        self.collider1 = actor1 if collider1 is None else collider1


class _Contact:
    def __init__(self, impulse: _Vec3) -> None:
        self.impulse = impulse


class SummarizeFingerContactImpulsesTest(unittest.TestCase):
    def test_impulses_are_summed_per_finger(self) -> None:
        """左右 finger それぞれの接触力積ノルムが接触点単位で合算される。"""
        headers = [
            _Header(actor0=1, actor1=100, offset=0, count=2),  # left: 2接触点
            _Header(actor0=2, actor1=100, offset=2, count=1),  # right: 1接触点
        ]
        data = [
            _Contact(_Vec3(3.0, 4.0, 0.0)),   # ノルム 5.0
            _Contact(_Vec3(0.0, 0.0, 1.0)),   # ノルム 1.0
            _Contact(_Vec3(0.0, 6.0, 8.0)),   # ノルム 10.0
        ]

        def finger_of_pair(actor0: int, actor1: int) -> str | None:
            return {1: "left", 2: "right"}.get(actor0)

        result = summarize_finger_contact_impulses(headers, data, finger_of_pair=finger_of_pair)

        self.assertAlmostEqual(result.left_ns, 6.0)
        self.assertAlmostEqual(result.right_ns, 10.0)

    def test_non_finger_contacts_are_ignored(self) -> None:
        """トマトと地面など finger 以外の接触は集計に含めない。"""
        headers = [_Header(actor0=99, actor1=100, offset=0, count=1)]
        data = [_Contact(_Vec3(7.0, 0.0, 0.0))]

        result = summarize_finger_contact_impulses(
            headers, data, finger_of_pair=lambda a0, a1: None
        )

        self.assertEqual(result.left_ns, 0.0)
        self.assertEqual(result.right_ns, 0.0)

    def test_reversed_actor_slots_are_classified(self) -> None:
        """finger が actor1 に入る PhysX report も取りこぼさない。"""
        headers = [_Header(actor0=100, actor1=1, offset=0, count=1)]
        data = [_Contact(_Vec3(0.0, 3.0, 4.0))]

        result = summarize_finger_contact_impulses(
            headers,
            data,
            finger_of_pair=lambda first, second: "left" if 1 in (first, second) else None,
        )

        self.assertEqual(result.left_ns, 5.0)

    def test_collider_ids_are_used_when_actor_paths_are_articulation_roots(self) -> None:
        """actor が articulation root でも collider 子 prim から左右を識別する。"""
        headers = [
            _Header(actor0=10, actor1=20, collider0=101, collider1=200, offset=0, count=1)
        ]
        data = [_Contact(_Vec3(2.0, 0.0, 0.0))]

        result = summarize_finger_contact_impulses(
            headers,
            data,
            finger_of_pair=lambda first, second: "left" if first == 101 else None,
        )

        self.assertEqual(result.left_ns, 2.0)

    def test_invalid_contact_data_range_is_ignored_fail_closed(self) -> None:
        headers = [_Header(actor0=1, actor1=100, offset=3, count=2)]

        result = summarize_finger_contact_impulses(
            headers, [], finger_of_pair=lambda first, second: "left"
        )

        self.assertEqual(result.left_ns, 0.0)

    def test_empty_report_yields_zero(self) -> None:
        result = summarize_finger_contact_impulses([], [], finger_of_pair=lambda a0, a1: None)

        self.assertEqual(result.left_ns, 0.0)
        self.assertEqual(result.right_ns, 0.0)


class EstimateStemTensionTest(unittest.TestCase):
    def test_tension_at_rest_equals_tomato_weight(self) -> None:
        """静止吊り下げ中の張力推定は自重（m×g）に一致する。"""
        tension = estimate_stem_tension_n(
            mass_kg=0.03,
            velocity_m_s=(0.0, 0.0, 0.0),
            previous_velocity_m_s=(0.0, 0.0, 0.0),
            dt_sec=1.0 / 60.0,
        )

        self.assertAlmostEqual(tension, 0.03 * 9.81, places=6)


def test_contact_force_conversion_uses_physics_dt() -> None:
    forces = contact_forces_from_impulses(
        FingerContactImpulses(left_ns=0.5, right_ns=0.25), dt_sec=0.02
    )

    assert forces.left_n == 25.0
    assert forces.right_n == 12.5

    def test_upward_acceleration_increases_tension(self) -> None:
        """上向き加速（引き上げ）中は自重より大きな張力が推定される。"""
        tension = estimate_stem_tension_n(
            mass_kg=0.03,
            velocity_m_s=(0.0, 0.0, 0.1),
            previous_velocity_m_s=(0.0, 0.0, 0.0),
            dt_sec=0.1,
        )

        # a_z = 1.0 m/s^2 → m×(g+a) = 0.03×10.81
        self.assertAlmostEqual(tension, 0.03 * 10.81, places=6)

    def test_zero_dt_falls_back_to_static_weight(self) -> None:
        """dt が 0 以下でもゼロ除算せず自重ベースの推定を返す。"""
        tension = estimate_stem_tension_n(
            mass_kg=0.03,
            velocity_m_s=(0.0, 0.0, 0.5),
            previous_velocity_m_s=(0.0, 0.0, 0.0),
            dt_sec=0.0,
        )

        self.assertAlmostEqual(tension, 0.03 * 9.81, places=6)


class FormatObservationLineTest(unittest.TestCase):
    def test_line_is_machine_parseable_key_value_format(self) -> None:
        """プロットスクリプトが解析する key=value 形式で全項目が出力される。"""
        line = format_observation_line(
            sequence_id=42,
            timestamp_sec=12.345,
            tomato_status="held",
            gripper_closed=True,
            grasp_joint_active=False,
            impulses=FingerContactImpulses(left_ns=0.5, right_ns=0.25),
            forces=FingerContactForces(left_n=25.0, right_n=12.5),
            tomato_speed_m_s=0.01,
            hand_distance_m=0.08,
            stem_distance_m=0.001,
            stem_tension_n=0.2943,
            finger_gap_m=0.0512,
            finger_midpoint_z_m=0.555,
            tomato_center_z_m=0.540,
            tray_contact_force_n=3.25,
        )

        self.assertTrue(line.startswith("[PhysicsObs] "))
        fields = dict(part.split("=", 1) for part in line.split()[1:])
        self.assertEqual(fields["status"], "held")
        self.assertEqual(fields["seq"], "42")
        self.assertEqual(fields["grip"], "1")
        self.assertEqual(fields["joint"], "0")
        self.assertAlmostEqual(float(fields["impL"]), 0.5)
        self.assertAlmostEqual(float(fields["impR"]), 0.25)
        self.assertAlmostEqual(float(fields["forceL"]), 25.0)
        self.assertAlmostEqual(float(fields["forceR"]), 12.5)
        self.assertAlmostEqual(float(fields["v"]), 0.01)
        self.assertAlmostEqual(float(fields["stemF"]), 0.2943)
        self.assertAlmostEqual(float(fields["gap"]), 0.0512)
        self.assertAlmostEqual(float(fields["finger_z"]), 0.555)
        self.assertAlmostEqual(float(fields["tomato_z"]), 0.540)
        self.assertAlmostEqual(float(fields["grasp_dz"]), 0.015)
        self.assertAlmostEqual(float(fields["trayF"]), 3.25)


def test_matching_contact_impulse_sums_only_selected_pairs() -> None:
    headers = [
        _Header(1, 2, 0, 1),
        _Header(3, 4, 1, 1),
    ]
    data = [_Contact(_Vec3(0.3, 0.4, 0.0)), _Contact(_Vec3(9.0, 0.0, 0.0))]

    assert summarize_matching_contact_impulse(
        headers, data, pair_matches=lambda first, second: (first, second) == (1, 2)
    ) == 0.5


if __name__ == "__main__":
    unittest.main()
