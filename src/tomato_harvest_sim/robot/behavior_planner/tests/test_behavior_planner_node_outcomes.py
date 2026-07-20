"""behavior_planner ノードの成果ベースフェーズ遷移判定のテスト。

JTC の succeeded/aborted に依存せず、物理的成果（tomato_status・到達距離）で
DETACHING / MOVING_TO_PLACE の完了を判定できることを検証する。
トマト把持中は残留振動で JTC がタイムアウト abort するため（run12 で実測）、
この成果ベース判定が唯一の前進経路になるケースがある。
"""
from __future__ import annotations

import unittest

from tomato_harvest_sim.msg.contracts import HarvestTaskPhase, Pose3D, TomatoStatus


class DetachingOutcomeTest(unittest.TestCase):
    def setUp(self) -> None:
        from tomato_harvest_sim.robot.behavior_planner import detaching_outcome
        self.outcome = detaching_outcome

    def test_detached_advances_to_moving_to_place(self) -> None:
        """トマトが枝から分離したら MOVING_TO_PLACE へ進む。"""
        self.assertIs(
            self.outcome(TomatoStatus.DETACHED), HarvestTaskPhase.MOVING_TO_PLACE
        )

    def test_fallen_fails(self) -> None:
        """トマトが落下したら FAILED。"""
        self.assertIs(self.outcome(TomatoStatus.FALLEN), HarvestTaskPhase.FAILED)

    def test_held_keeps_waiting(self) -> None:
        """まだ枝に付いたまま把持中なら遷移しない（引き離し継続）。"""
        self.assertIsNone(self.outcome(TomatoStatus.HELD))

    def test_attached_keeps_waiting(self) -> None:
        self.assertIsNone(self.outcome(TomatoStatus.ATTACHED))


class MovingToPlaceOutcomeTest(unittest.TestCase):
    def setUp(self) -> None:
        from tomato_harvest_sim.robot.behavior_planner import moving_to_place_outcome
        self.outcome = moving_to_place_outcome
        self.place_pose = Pose3D(0.35, -0.35, 0.57, 180.0, 0.0, 0.0)

    def test_tool_within_tolerance_advances_to_placed(self) -> None:
        """ツールが place_pose の許容距離内に入ったら RELEASING へ進む。"""
        tool = Pose3D(0.35, -0.34, 0.57, 180.0, 0.0, 0.0)  # 1cm 誤差
        self.assertIs(
            self.outcome(TomatoStatus.DETACHED, tool, self.place_pose),
            HarvestTaskPhase.RELEASING,
        )

    def test_tool_far_keeps_waiting(self) -> None:
        """ツールが遠ければ遷移しない。"""
        tool = Pose3D(0.5, 0.3, 0.4, 180.0, 0.0, 0.0)
        self.assertIsNone(self.outcome(TomatoStatus.DETACHED, tool, self.place_pose))

    def test_fallen_fails_regardless_of_distance(self) -> None:
        """搬送中にトマトが落下したら距離に関係なく FAILED。"""
        tool = Pose3D(0.35, -0.35, 0.57, 180.0, 0.0, 0.0)
        self.assertIs(
            self.outcome(TomatoStatus.FALLEN, tool, self.place_pose),
            HarvestTaskPhase.FAILED,
        )

    def test_missing_place_pose_keeps_waiting(self) -> None:
        """place_pose 未設定（plan 未受信）なら遷移しない。"""
        tool = Pose3D(0.35, -0.35, 0.57, 180.0, 0.0, 0.0)
        self.assertIsNone(self.outcome(TomatoStatus.DETACHED, tool, None))

    def test_missing_tool_pose_keeps_waiting(self) -> None:
        self.assertIsNone(self.outcome(TomatoStatus.DETACHED, None, self.place_pose))


if __name__ == "__main__":
    unittest.main()


class ExecutionStatusObservationTest(unittest.TestCase):
    """Issue #58でabort理由をtask state machineまで保持する。"""

    def setUp(self) -> None:
        from tomato_harvest_sim.robot.behavior_planner import (
            execution_status_observation,
        )
        self.observe = execution_status_observation

    def test_plain_status_preserves_backward_compatibility(self) -> None:
        observation = self.observe(" aborted ")
        self.assertEqual(observation.status, "aborted")
        self.assertIsNone(observation.abort_reason)

    def test_json_status_field_is_preserved(self) -> None:
        observation = self.observe(
            '{"status": "aborted", "max_joint_error_rad": 0.18}'
        )
        self.assertEqual(observation.status, "aborted")
        self.assertIsNone(observation.abort_reason)

    def test_json_abort_reason_is_preserved(self) -> None:
        observation = self.observe(
            '{"status":"aborted","abort_reason":"missing_trajectory"}'
        )
        self.assertEqual(observation.status, "aborted")
        self.assertEqual(observation.abort_reason, "missing_trajectory")

    def test_json_without_status_is_unknown(self) -> None:
        observation = self.observe('{"abort_reason":"missing_trajectory"}')
        self.assertEqual(observation.status, "unknown")
        self.assertEqual(observation.abort_reason, "missing_trajectory")
