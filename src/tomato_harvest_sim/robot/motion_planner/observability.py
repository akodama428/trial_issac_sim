"""MoveIt2 改善 Step 0 の構造化観測イベントを生成する。"""

from __future__ import annotations

import json


METRIC_PREFIX = "MOVEIT_METRIC"


def metric_line(event: str, **fields: object) -> str:
    """ログ集計で再利用できる JSON Lines 互換のイベントを返す。

    Args:
        event: 安定したイベント種別名。
        **fields: イベント固有の観測値。

    Returns:
        固定プレフィックスと JSON object を空白で連結した文字列。

    Raises:
        ValueError: NaN や Infinity が fields に含まれる場合。
    """
    payload = {"event": event, **fields}
    return f"{METRIC_PREFIX} {json.dumps(payload, sort_keys=True, allow_nan=False)}"
