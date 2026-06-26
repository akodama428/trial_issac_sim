---
title: franka_velocity_based_joint_trajectory_execution.md
version: 0.1.0
status: draft
owner: atsushi
created: 2026-06-26
updated: 2026-06-26
---

# Franka 速度指令ベース Joint Trajectory 実装計画

## Summary
- MoveIt の trajectory 生成は維持し、`IsaacFrankaMotionExecutor` の実行則だけを速度指令ベースへ変更する。
- `JointTrajectoryPoint.time_from_start_sec` を segment 実行時間へ反映する。
- joint trajectory 実行が停止した場合は、同一 snapshot に `motion_waypoints` があれば waypoint IK へ、無ければ direct IK へフォールバックする。
- `hoge.log` で確認できたとおり、停止要因は MoveIt の不在ではなく executor 側の追従則にある。

## Current State
- 変更前の `_step_joint_trajectory()` は `step_toward_joint_positions()` による位置ステップ制御だった。
- `time_from_start_sec` は未使用だった。
- `TOMATO_HARVEST_USE_JOINT_TRAJECTORY_EXECUTION=1` の経路でも、MoveIt の planning 自体は `accepted trajectory` まで成立していた。
- 停止時は同じ joint target を送り続け、timeout や stall 判定が無かった。

## Implementation
- `sync_with_snapshot()` は joint trajectory と waypoint を同時保持する。
- trajectory 実行時は private な `_TrajectorySegment` 列を構築する。
- 各 segment は `start_positions`, `target_positions`, `duration_sec`, `deadline_sec`, `start_time_sec`, `initial_error_max` を持つ。
- 先頭 target が現在 joint state と離れている場合は、現在値を synthetic start として first segment の開始点に使う。
- `time_from_start_sec` が非単調な区間は `1e-3` 秒に丸めて warning を出す。
- `_step_joint_trajectory()` は `qdot_cmd = clip((target_q - current_q) / max(remaining_time_sec, control_dt_sec), joint_limits)` で arm 速度を計算する。
- 速度上限は `src/tomato_harvest_sim/robot/moveit_config/joint_limits.yaml` から読む。
- 出力は `ArticulationAction(joint_velocities=...)` を優先し、速度 action が使えない場合は `current_q + qdot_cmd * dt` を position command として送る。
- gripper は従来どおり位置指令で維持し、arm だけ速度指令へ切り替える。
- final target 到達時は zero velocity を送りつつ gripper 保持を継続する。
- stall 判定は `0.5` 秒窓で `0.01 rad` 未満の誤差改善、timeout 判定は `max(duration_sec * 2.0, 0.5)` 秒とする。
- stall / timeout 時は debug log に理由を残して trajectory state を捨て、waypoint IK または direct IK へ切り替える。

## Logging
- 追加ログは `segment`, `target_q`, `command_q`, `qdot_cmd`, `remaining_time_sec`, `joint_error_max`, `timeout/stall reason`, `fallback started` を出す。
- 既存の `[Simulator][TrajectoryDebug]` prefix を維持し、`hoge.log` と同じ粒度で追えるようにする。

## Tests
- `tests/test_franka_motion_executor.py` で以下を確認する。
- `time_from_start_sec` から segment duration が構築されること。
- synthetic start が現在 joint state から構築されること。
- `apply_action` 利用時に velocity action が優先されること。
- velocity action 非対応時に position command fallback が動くこと。
- joint velocity が `joint_limits.yaml` を超えないこと。
- final point 到達後も gripper 保持が継続すること。
- trajectory stall 時に waypoint IK へフォールバックすること。
- env var 無効時は従来どおり waypoint IK 優先であること。
