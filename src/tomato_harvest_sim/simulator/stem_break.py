from __future__ import annotations

from enum import StrEnum
from typing import Any


def encoded_joint_path_parts(payload: Any) -> tuple[int, int] | None:
    """carb payloadから公式API形式のjointPath 2要素を取り出す。"""
    try:
        encoded_path = payload["jointPath"]
        return int(encoded_path[0]), int(encoded_path[1])
    except (AttributeError, IndexError, KeyError, TypeError, ValueError):
        return None


class StemBreakDecision(StrEnum):
    """stem破断イベントの照合結果。"""

    TARGET_BROKEN = "target_broken"
    DUPLICATE = "duplicate"
    IGNORED_EVENT_TYPE = "ignored_event_type"
    IGNORED_JOINT = "ignored_joint"
    INVALID_JOINT_PATH = "invalid_joint_path"


class StemBreakEventMatcher:
    """外部APIに依存せず、対象stem jointの破断だけを一度受理する。"""

    def __init__(self, target_joint_path: str) -> None:
        self._target_joint_path = target_joint_path
        self._broken = False

    @property
    def broken(self) -> bool:
        return self._broken

    def observe(
        self, event_type: str, decoded_joint_path: str | None
    ) -> StemBreakDecision:
        if event_type != "joint_break":
            return StemBreakDecision.IGNORED_EVENT_TYPE
        if not decoded_joint_path:
            return StemBreakDecision.INVALID_JOINT_PATH
        if decoded_joint_path != self._target_joint_path:
            return StemBreakDecision.IGNORED_JOINT
        if self._broken:
            return StemBreakDecision.DUPLICATE
        self._broken = True
        return StemBreakDecision.TARGET_BROKEN

    def reset(self) -> None:
        self._broken = False
