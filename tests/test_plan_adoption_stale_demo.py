"""stale plan 再現シナリオが仕様どおり抑止されることを確認するテスト (Issue #9)。"""
from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

_SCRIPT = Path(__file__).resolve().parent.parent / "scripts" / "plan_adoption_stale_demo.py"
_spec = importlib.util.spec_from_file_location("plan_adoption_stale_demo", _SCRIPT)
assert _spec is not None and _spec.loader is not None
demo = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = demo
_spec.loader.exec_module(demo)


class TestStaleScenarioReplay(unittest.TestCase):
    def test_step1_policy_matches_scenario_expectation_per_event(self) -> None:
        """全イベントで、実装の採用判定がシナリオ仕様の期待 stale 判定と一致する。"""
        for result in demo.replay_scenario():
            with self.subTest(event=result["label"]):
                self.assertEqual(
                    bool(result["step1_adopted"]),
                    not bool(result["expected_stale"]),
                    msg=f"{result['label']}: reason={result['step1_reason']}",
                )

    def test_legacy_contract_adopts_stale_plans_but_step1_suppresses_all(self) -> None:
        counts = demo.stale_adoption_counts(demo.replay_scenario())
        self.assertEqual(counts["stale_event_total"], 3)
        self.assertEqual(counts["legacy_contract"], 3)
        self.assertEqual(counts["step1_contract"], 0)


if __name__ == "__main__":
    unittest.main()
