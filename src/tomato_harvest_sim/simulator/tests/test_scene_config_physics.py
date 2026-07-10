"""physics チューニング設定（scene.yaml physics セクション）の読み込みテスト。

Step 1（物理モデル土台）: 摩擦・コリジョン・ソルバのパラメータはコードに
ハードコードせず scene.yaml で管理し、セクションが無い場合は「適用しない」
安全側デフォルトへフォールバックする。
"""
from __future__ import annotations

import os
import unittest
from unittest.mock import patch

from tomato_harvest_sim.simulator.scene_config import (
    load_physics_tuning_config,
    physics_tuning_from_payload,
)

_FULL_PAYLOAD = {
    "physics": {
        "enabled": True,
        "tomato_material": {
            "static_friction": 1.2,
            "dynamic_friction": 1.0,
            "restitution": 0.1,
        },
        "gripper_material": {
            "static_friction": 1.1,
            "dynamic_friction": 0.9,
            "restitution": 0.05,
        },
        "container_material": {
            "static_friction": 0.8,
            "dynamic_friction": 0.7,
            "restitution": 0.2,
        },
        "friction_combine_mode": "max",
        "restitution_combine_mode": "min",
        "tomato_collision": {
            "contact_offset_m": 0.002,
            "rest_offset_m": 0.0,
            "torsional_patch_radius_m": 0.004,
            "min_torsional_patch_radius_m": 0.001,
        },
        "tomato_solver": {
            "position_iterations": 16,
            "velocity_iterations": 4,
        },
        "finger_drive": {
            "stiffness": 3000.0,
            "damping": 120.0,
            "max_force_n": 5.0,
        },
    }
}


class PhysicsTuningFromPayloadTest(unittest.TestCase):
    def test_full_payload_is_loaded(self) -> None:
        """yaml の physics セクション全項目が設定値として読み込まれる。"""
        config = physics_tuning_from_payload(_FULL_PAYLOAD)

        self.assertTrue(config.enabled)
        self.assertAlmostEqual(config.tomato_material.static_friction, 1.2)
        self.assertAlmostEqual(config.tomato_material.dynamic_friction, 1.0)
        self.assertAlmostEqual(config.gripper_material.restitution, 0.05)
        self.assertAlmostEqual(config.container_material.static_friction, 0.8)
        self.assertEqual(config.friction_combine_mode, "max")
        self.assertEqual(config.restitution_combine_mode, "min")
        self.assertAlmostEqual(config.tomato_contact_offset_m, 0.002)
        self.assertAlmostEqual(config.tomato_rest_offset_m, 0.0)
        self.assertAlmostEqual(config.tomato_torsional_patch_radius_m, 0.004)
        self.assertEqual(config.tomato_solver_position_iterations, 16)
        self.assertEqual(config.tomato_solver_velocity_iterations, 4)

    def test_finger_drive_is_loaded(self) -> None:
        """Step 2: finger drive の力制限パラメータが読み込まれる。"""
        config = physics_tuning_from_payload(_FULL_PAYLOAD)

        self.assertAlmostEqual(config.finger_drive_stiffness, 3000.0)
        self.assertAlmostEqual(config.finger_drive_damping, 120.0)
        self.assertAlmostEqual(config.finger_drive_max_force_n, 5.0)

    def test_missing_finger_drive_leaves_drive_untouched(self) -> None:
        """finger_drive 未定義なら maxForce=0（drive へ何も適用しない）。"""
        physics = {k: v for k, v in _FULL_PAYLOAD["physics"].items() if k != "finger_drive"}
        config = physics_tuning_from_payload({"physics": physics})

        self.assertTrue(config.enabled)
        self.assertEqual(config.finger_drive_max_force_n, 0.0)

    def test_missing_section_disables_tuning(self) -> None:
        """physics セクションが無い場合は enabled=False（従来挙動を維持）。"""
        config = physics_tuning_from_payload({"scene": {}})

        self.assertFalse(config.enabled)

    def test_invalid_combine_mode_is_rejected(self) -> None:
        """PhysX が受け付けない combine mode は設定ミスとして即座に検出する。"""
        payload = {"physics": dict(_FULL_PAYLOAD["physics"], friction_combine_mode="bogus")}

        with self.assertRaises(ValueError):
            physics_tuning_from_payload(payload)

    def test_env_kill_switch_disables_tuning(self) -> None:
        """TOMATO_HARVEST_PHYSICS_TUNING=0 で A/B 比較用に適用を無効化できる。"""
        with patch.dict(os.environ, {"TOMATO_HARVEST_PHYSICS_TUNING": "0"}):
            config = physics_tuning_from_payload(_FULL_PAYLOAD)

        self.assertFalse(config.enabled)


class LoadPhysicsTuningConfigTest(unittest.TestCase):
    def test_repo_scene_yaml_enables_tuning(self) -> None:
        """リポジトリの scene.yaml には physics セクションが定義され有効である。"""
        load_physics_tuning_config.cache_clear()
        config = load_physics_tuning_config()

        self.assertTrue(config.enabled)
        self.assertGreater(config.tomato_material.static_friction, 0.0)
        self.assertGreater(config.tomato_torsional_patch_radius_m, 0.0)


if __name__ == "__main__":
    unittest.main()
