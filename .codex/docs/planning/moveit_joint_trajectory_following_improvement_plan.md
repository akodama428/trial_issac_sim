---
title: moveit_joint_trajectory_following_improvement_plan.md
version: 0.1.0
status: draft
owner: atsushi
created: 2026-06-27
updated: 2026-06-27
---

# MoveIt Joint Trajectory 追従制御 改善プラン

## Summary
- 現在の `segment_timeout -> waypoint IK fallback` は、一般的な MoveIt 実行方式とは一致しない。
- MoveIt の一般形は `time-parameterized JointTrajectory` を `FollowJointTrajectory` 経由で controller に渡し、controller 側が `path_tolerance`、`goal_tolerance`、`goal_time_tolerance` と expected duration を基準に成否を判定する構成である。
- したがって改善方針は、`segment_timeout` を一次判定から降ろし、`trajectory 全体の時間基準追従 + tolerance 基準の abort + current-state replanning` へ寄せる。
- waypoint IK は即時フォールバックではなく、`trajectory unavailable` または `replan failed` 時の劣化モードへ下げる。

## Research-Backed Findings
### 1. MoveIt の標準 execution 境界
- MoveIt 公式 docs では、controller interface として `FollowJointTrajectory` を使い、low-level 実行は robot controller 側へ委譲する構成が基本である。
- MoveIt 側で監視する代表値は `allowed_execution_duration_scaling`、`allowed_goal_duration_margin`、`allowed_start_tolerance` である。

### 2. 一般的な追従制御則
- `ros2_control` の `joint_trajectory_controller` は、trajectory point 間を時間補間して `q_ref(t)` を生成する。
- velocity command interface では、`position + velocity` の追従誤差を PID loop で velocity command へ写像する。
- trajectory に velocity が含まれれば、それも参照値として扱う。

### 3. 一般的な失敗判定
- `FollowJointTrajectory` action は `path_tolerance`、`goal_tolerance`、`goal_time_tolerance` を持つ。
- path tolerance を外れた場合は abort、終了時刻 + goal_time_tolerance までに goal tolerance に入らない場合も abort である。
- つまり一般的な失敗判定は、segment ごとの独自 timeout より、trajectory 全体に対する tolerance / duration 監視である。

## Current Gap In This Repository
- `src/tomato_harvest_sim/robot/trajectory_execution.py` は `TrajectorySegment.deadline_sec` を使い、各 segment に `duration * 2.0` もしくは `0.5s` の deadline を置いている。
- 同ファイルは progress stall も `0.5s` 窓の `joint_error_max` 改善量で判定し、失敗時に waypoint IK または direct IK へ即切り替える。
- この設計だと、MoveIt が妥当な trajectory を返していても、局所的な追従遅れだけで execution semantics が別方式へ切り替わる。
- さらに phase ごとの後続 trajectory が事前生成済みだと、途中 abort 後に remaining phase trajectory が stale になりやすい。

## 結論
- `segment_timeout` を完全に禁止する必要はないが、一次判定に置くのは妥当ではない。
- `segment_timeout` を残すなら debug / watchdog 用の二次保護に下げるべきであり、主判定は `trajectory-level duration` と `path/goal tolerance` に置くべきである。
- 追従失敗時の第一選択は `waypoint IK fallback` ではなく `trajectory abort + current-state replan` にするのが MoveIt 流儀に近い。

## Target Execution Policy
```text
MoveIt trajectory accepted
  -> validate start state against trajectory first point
  -> execute time-parameterized reference continuously
  -> monitor path tolerance during execution
  -> monitor total allowed duration near trajectory end
  -> if success: hold final target with zero terminal velocity
  -> if failure: abort trajectory, stop smoothly, replan from current joint state
  -> if replan unavailable or retry budget exhausted: degrade to waypoint IK
```

## Proposed Changes
### 1. 失敗判定を segment 基準から trajectory 基準へ変更する
- `segment_timeout` を primary failure reason から外す。
- executor に次の trajectory-level state を持たせる。
  - `trajectory_start_wall_time_sec`
  - `trajectory_expected_duration_sec`
  - `trajectory_allowed_duration_sec`
  - `allowed_start_tolerance_rad`
  - `path_tolerance_rad`
  - `goal_tolerance_rad`
  - `goal_time_tolerance_sec`
- `trajectory_allowed_duration_sec` は、MoveIt 公式 docs に合わせて
  - `expected_duration * allowed_execution_duration_scaling + allowed_goal_duration_margin`
  で求める。

### 2. 開始点整合チェックを明示化する
- trajectory 受理時に、current joint state と trajectory first point の誤差を評価する。
- 誤差が `allowed_start_tolerance` を超える場合は、synthetic start を差し込んで無理に走らせるのではなく、まず replan を要求する。
- synthetic start は `minor mismatch` 吸収の補助手段へ格下げする。

### 3. 追従制御を time-based reference tracking として固定する
- 各周期で active segment の endpoint を直接狙うのではなく、trajectory 上の現在時刻 `t` から `q_ref(t)` と `qd_ref(t)` を評価する。
- 速度指令は少なくとも次の形にする。
  - `qdot_cmd = qd_ref + Kp * (q_ref - q) + Kd * (qd_ref - qd)`
- terminal 区間では `alpha=1.0` 到達後に `qd_ref=0` を明示し、終端で速度を残さない。
- これは既存 `trajectory_tracking.py` の方向性を残しつつ、abort 条件と terminal handling を MoveIt 流儀へ寄せる変更である。

### 4. path tolerance / goal tolerance を主監視値にする
- 実行中は各周期で `joint_error = q_ref - q` を測り、`max_abs(joint_error)` が `path_tolerance_rad` を超えたら abort 候補にする。
- 単発ノイズで abort しないため、simulation 専用の微小 debounce を入れてよい。
  - これは MoveIt 標準ではなく、数値ノイズ対策としての実装判断である。
- 最終点付近では `goal_tolerance_rad` と `goal_time_tolerance_sec` で完了判定する。

### 5. 失敗時は waypoint IK ではなく current-state replan を第一選択にする
- `PATH_TOLERANCE_VIOLATED` 相当、`GOAL_TOLERANCE_VIOLATED` 相当、`START_TOLERANCE_VIOLATED` 相当の失敗理由を executor 内で分類する。
- abort 後は arm 速度をゼロ化し、最新 joint state と同一 phase 目標から MoveIt 再計画を要求する。
- 再計画成功時は、その phase 以降の remaining trajectory を丸ごと再生成し、stale な phase trajectory を破棄する。
- waypoint IK は次のときだけ使う。
  - MoveIt trajectory が得られない
  - replan が失敗した
  - retry 回数上限を超えた

### 6. `segment_timeout` は watchdog に限定する
- 完全に progress を失い、かつ controller 出力も飽和しているような異常を検出するための最終 watchdog は残してよい。
- ただし watchdog 発火時の動作は `abort + replan` とし、即 waypoint IK へ飛ばさない。
- しきい値は現状の `duration * 2.0` 固定ではなく、trajectory 全体の duration と tolerance 監視で吸収できない異常時だけを対象にする。

## Implementation Delta
### `src/tomato_harvest_sim/robot/trajectory_execution.py`
- `TrajectorySegment.deadline_sec` 依存の primary abort を廃止する。
- trajectory-level execution state を追加する。
- failure reason を `segment_timeout/segment_stall` 文字列ではなく、`start_tolerance_violation/path_tolerance_violation/goal_timeout/watchdog_timeout` に整理する。
- `_fallback_from_joint_trajectory()` は `waypoint IK fallback` ではなく、`abort_current_trajectory_and_request_replan()` を第一経路に差し替える。

### `src/tomato_harvest_sim/robot/trajectory_tracking.py`
- `q_ref(t)`、`qd_ref(t)`、terminal zero velocity を明示する。
- trajectory point に速度がある場合はそれを優先する。
- final sample 後に reference velocity が残らないようにする。

### `src/tomato_harvest_sim/robot/runtime.py`
- phase 実行失敗時に、同一 phase 目標に対する再計画要求を追加する。
- 途中 abort 後は、残り phase に対する旧 trajectory を無効化する。

### Planner / contract 側
- `JointTrajectory` public contract 自体は変えない。
- ただし runtime からは「current-state から単一 phase を再計画する」呼び出し経路を追加する。

## Rollout Plan
### Phase 1
- `segment_timeout` を debug/watchdog 用へ格下げする。
- trajectory-level duration 監視と start tolerance 判定を先に入れる。

### Phase 2
- path tolerance / goal tolerance ベースの abort へ切り替える。
- terminal velocity zeroing を入れる。

### Phase 3
- failure 時の第一選択を current-state replan に置き換える。
- waypoint IK は degraded mode に下げる。

### Phase 4
- 必要なら velocity PID gain を `joint_trajectory_controller` に合わせて調整する。
- 可能なら later phase の一括事前計画をやめ、phase ごとの逐次再計画へ寄せる。

## Test Plan
- trajectory first point と current state の大きな不一致で replan が選ばれ、即 synthetic start 実行にならないこと。
- expected duration 超過時に `goal_timeout` 相当で abort されること。
- path tolerance 超過時に abort され、直接 waypoint IK へ行かないこと。
- abort 後に current joint state から再計画が要求されること。
- 再計画成功時、旧 phase trajectory が再利用されないこと。
- terminal 到達時に arm velocity command がゼロになること。
- MoveIt unavailable / replan failed 時のみ waypoint IK degraded mode へ入ること。

## Sources
- MoveIt Low Level Controllers
  - https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html
- MoveIt Time Parameterization
  - https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html
- ros2_control `joint_trajectory_controller`
  - https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
- ros2_control Trajectory Representation
  - https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html
- `control_msgs/FollowJointTrajectory.action`
  - https://raw.githubusercontent.com/ros-controls/control_msgs/master/control_msgs/action/FollowJointTrajectory.action
