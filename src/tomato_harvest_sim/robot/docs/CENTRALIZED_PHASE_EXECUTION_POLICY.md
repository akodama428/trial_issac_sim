# Centralized Phase Execution Policy

## 目的
- `pregrasp / grasp / pull / place / home` の各フェーズについて、目標 pose、成功条件、timeout、abort、replan 条件を 1 箇所で定義する。
- `behavior_planner`、`trajectory_tracking`、`ros2_control`、`simulator` に分散している execution 意味論を整理し、責務境界を明確にする。
- `simulator` は Isaac Sim API の実現系に限定し、フェーズ判定や timeout 判定を持たない構成へ寄せる。

## 現状の問題
現状は各フェーズの execution 条件が複数箇所に分散している。

- `robot/runtime.py`
  - フェーズ遷移
  - `pregrasp` / `grasp` の成功判定
- `robot/trajectory_tracking/state_store.py`
  - snapshot から active target を再解釈
- `robot/trajectory_tracking/coordinator.py`
  - 実行 request の構成
- `robot/ros2_control/joint_trajectory_controller_bridge.py`
  - `goal_timeout`
  - `path_tolerance_violation`
  - controller 側の success / abort
- `simulator/scene_runtime.py`
  - `target_tool_pose`
  - waypoint 進行
  - pose tolerance

この構成だと次の問題が起こる。

- 同じフェーズに対して複数の成功条件が存在する。
- `phase goal` と `active waypoint` が同じ変数に押し込まれ、意味論が崩れる。
- timeout / abort のチューニング箇所が分散し、挙動変更の影響範囲が読めない。
- `runtime` が見ている目標と `trajectory_tracking` / `simulator` が追っている目標がずれる。
- `simulator` が execution policy を持ってしまい、責務境界が崩れる。

## 解決方針
各フェーズの execution 条件を `ExecutionPhaseSpec` に集約するが、上位計画と下位計画は分離する。

- `behavior_planner`
  - `SceneSnapshot` と task 状態から、次に実行すべき phase を選ぶ。
  - `PhaseExecutionIntent` を生成する。
  - 各フェーズの success / abort / replan 条件は `yaml` から読み込む。
- `planner`
  - `PhaseExecutionIntent` を入力として、対応する `PhaseMotionPlan` を生成する。
  - MoveIt / 幾何計画 / `JointTrajectory` 生成を担当する。
- `runtime`
  - `PhaseExecutionIntent` と `PhaseMotionPlan` から `ExecutionPhaseSpec` を組み立てる。
  - `ExecutionPhaseSpec` を dispatch する。
- `trajectory_tracking`
  - `ExecutionPhaseSpec` を解釈して実行する。
- `ros2_control`
  - `ExecutionPhaseSpec` に含まれる tolerance / timeout / abort 条件に従って controller semantics を返す。
- `simulator`
  - command を Isaac Sim に適用し、observation を返すだけに限定する。

## 変更後アーキ図
```mermaid
flowchart TB
  subgraph Robot["robot"]
    direction LR

    subgraph Perception["perception"]
      D1["TargetEstimator"]
    end

    subgraph Behavior["behavior_planner"]
      B1["BehaviorPlanner"]
      B2["PhaseExecutionIntentBuilder"]
    end

    subgraph Runtime["runtime"]
      R1["HarvestRuntime"]
    end

    subgraph Planner["planner"]
      P1["MotionPlanner"]
      P2["MoveIt2ServiceBridge"]
    end

    subgraph Tracking["trajectory_tracking"]
      T1["TrajectoryExecutionCoordinator"]
      T2["ExecutionStateStore"]
      T3["ExecutionMonitor"]
    end

    subgraph Control["ros2_control"]
      C1["JointTrajectoryControllerBridge"]
    end
  end

  subgraph Boundary["src/tomato_harvest_sim/api"]
    direction LR
    A1["SceneSnapshot"]
    A2["PhaseExecutionIntent"]
    A3["PhaseMotionPlan"]
    A4["ExecutionPhaseSpec"]
    A5["TrajectoryExecutionRequest / Result"]
    A6["HardwareCommandSample / HardwareStateSample"]
  end

  subgraph Simulation["simulator"]
    direction LR
    S1["IsaacRos2ControlSystem"]
    S2["IsaacFrankaDriver"]
    S3["Isaac Sim API"]
  end

  D1 --> A1
  A1 --> R1
  A1 --> B1
  A1 --> P1
  B1 --> A2
  A2 --> P1
  P1 --> P2
  P2 --> A3
  A2 --> R1
  A3 --> R1
  R1 --> A4
  A4 --> T1
  T1 --> T2
  T1 --> T3
  T1 --> A5
  A5 --> C1
  C1 --> A6
  A6 --> S1
  S1 --> S2
  S2 --> S3
  S3 --> S2
  S1 --> A6
```

## フェーズ spec の責務
`ExecutionPhaseSpec` は 1 フェーズ分の execution contract を表す。

- 何を実行するか
- 何を success とみなすか
- どの条件で abort するか
- abort 後に replan するか

これは `BehaviorPlanner` が出す `PhaseExecutionIntent` と、`MotionPlanner` が出す `PhaseMotionPlan` を `runtime` が束ねて組み立てる。

`PhaseExecutionIntent` に含まれる phase 条件の正本は、`PhaseExecutionIntentBuilder` が読む `yaml` とする。

## 提案する IF
`src/tomato_harvest_sim/robot/api` に次のような定義を置く。

```python
from dataclasses import dataclass
from enum import StrEnum

from tomato_harvest_sim.api.contracts import JointTrajectory, Pose3D


class PhaseId(StrEnum):
    MOVING_TO_PREGRASP = "moving_to_pregrasp"
    MOVING_TO_GRASP = "moving_to_grasp"
    PULL_TO_DETACH = "pull_to_detach"
    MOVING_TO_PLACE = "moving_to_place"
    RETURNING_HOME = "returning_home"


class PoseSemantics(StrEnum):
    TOOL_CENTER = "tool_center"
    GRASP_CENTER = "grasp_center"
    MOVEIT_LINK = "moveit_link"


class SuccessJudge(StrEnum):
    END_EFFECTOR_POSE = "end_effector_pose"
    JOINT_TRAJECTORY_COMPLETED = "joint_trajectory_completed"
    TOMATO_STATE = "tomato_state"


@dataclass(frozen=True)
class PhaseExecutionIntent:
    phase_id: PhaseId
    phase_goal_pose: Pose3D | None
    pose_semantics: PoseSemantics
    success: "SuccessPolicy"
    abort: "AbortPolicy"


@dataclass(frozen=True)
class PhaseMotionPlan:
    phase_goal_pose: Pose3D | None
    active_waypoints: tuple[Pose3D, ...]
    joint_trajectory: JointTrajectory | None


@dataclass(frozen=True)
class SuccessPolicy:
    judge: SuccessJudge
    position_tolerance_m: float | None = None
    stable_steps: int = 1


@dataclass(frozen=True)
class AbortPolicy:
    nominal_timeout_sec: float | None = None
    stall_timeout_sec: float | None = None
    min_progress_delta_m: float | None = None
    joint_path_tolerance_rad: float | None = None
    allow_replan: bool = True


@dataclass(frozen=True)
class ExecutionPhaseSpec:
    phase_id: PhaseId
    intent: PhaseExecutionIntent
    motion: PhaseMotionPlan
```

## YAML 設定
`PhaseExecutionIntentBuilder` は phase ごとの execution 条件を `yaml` から読み込む。

- ここに置くもの
  - success judge
  - position tolerance
  - stable steps
  - nominal timeout
  - stall timeout
  - min progress delta
  - allow replan
  - pose semantics
- ここに置かないもの
  - goal pose
  - waypoint
  - `JointTrajectory`
  - MoveIt planning request の具体値

例:

```yaml
phases:
  moving_to_pregrasp:
    pose_semantics: tool_center
    success:
      judge: end_effector_pose
      position_tolerance_m: 0.03
      stable_steps: 1
    abort:
      nominal_timeout_sec: 3.0
      stall_timeout_sec: 0.5
      min_progress_delta_m: 0.005
      allow_replan: true

  moving_to_grasp:
    pose_semantics: grasp_center
    success:
      judge: end_effector_pose
      position_tolerance_m: 0.005
      stable_steps: 2
    abort:
      nominal_timeout_sec: 2.0
      stall_timeout_sec: 0.5
      min_progress_delta_m: 0.002
      allow_replan: true

  pull_to_detach:
    pose_semantics: grasp_center
    success:
      judge: tomato_state
    abort:
      nominal_timeout_sec: 2.0
      stall_timeout_sec: 0.5
      allow_replan: true

  moving_to_place:
    pose_semantics: tool_center
    success:
      judge: end_effector_pose
      position_tolerance_m: 0.05
      stable_steps: 1
    abort:
      nominal_timeout_sec: 3.0
      stall_timeout_sec: 0.5
      min_progress_delta_m: 0.005
      allow_replan: true
```

推奨配置:

```text
src/tomato_harvest_sim/robot/behavior_planner/config/phase_execution.yaml
```

## 重要な設計原則
### 1. `phase_goal_pose` と `active_waypoint_pose` を分ける
`phase_goal_pose` はフェーズ全体の最終目標であり、`active_waypoint_pose` は途中経由点である。  
今のように `target_tool_pose` 1 つに潰してはいけない。

### 2. success 判定は `ExecutionPhaseSpec` だけが決める
`behavior_planner`、`runtime`、`simulator`、`bridge` が独自に success を再定義しない。

### 3. abort 条件は `ExecutionPhaseSpec.abort` だけが決める
`goal_timeout`、`stall`、`path_tolerance_violation` の閾値は 1 箇所に集約する。

### 4. `simulator` は phase 判定を持たない
`scene_runtime` は active target を内部で決めず、受け取った command の適用と pose 同期だけを担当する。

## フェーズごとの spec 例
### `moving_to_pregrasp`
```text
phase_goal_pose:
  pregrasp_pose
active_waypoints:
  pregrasp_waypoints
joint_trajectory:
  pregrasp_joint_trajectory
success:
  judge=end_effector_pose
  position_tolerance_m=0.03
  stable_steps=1
abort:
  nominal_timeout_sec=trajectory_duration + margin
  stall_timeout_sec=0.5
  min_progress_delta_m=0.005
  allow_replan=true
```

### `moving_to_grasp`
```text
phase_goal_pose:
  grasp_pose
active_waypoints:
  grasp_waypoints
joint_trajectory:
  grasp_joint_trajectory
success:
  judge=end_effector_pose
  position_tolerance_m=0.005
  stable_steps=2
abort:
  nominal_timeout_sec=trajectory_duration + margin
  stall_timeout_sec=0.5
  min_progress_delta_m=0.002
  allow_replan=true
```

### `pull_to_detach`
```text
phase_goal_pose:
  pull_pose
success:
  judge=tomato_state
abort:
  nominal_timeout_sec=...
  stall_timeout_sec=...
  allow_replan=true
```

### `moving_to_place`
```text
phase_goal_pose:
  place_pose
success:
  judge=end_effector_pose
  position_tolerance_m=0.05
abort:
  nominal_timeout_sec=...
  stall_timeout_sec=...
  allow_replan=true
```

## 処理フロー
```mermaid
flowchart TD
  A["HarvestRuntime が task state を更新"] --> B["BehaviorPlanner が次フェーズを選び PhaseExecutionIntent を生成"]
  B --> C["MotionPlanner が intent を受けて PhaseMotionPlan を生成"]
  C --> D["HarvestRuntime が intent + motion から ExecutionPhaseSpec を構成"]
  D --> E["TrajectoryExecutionCoordinator が spec を受け取る"]
  E --> F["joint trajectory があれば TrajectoryExecutionRequest を構成"]
  F --> G["JointTrajectoryControllerBridge が実行"]
  G --> H["HardwareState を読み取り"]
  H --> I{"spec.success を満たしたか"}
  I -->|yes| J["phase completed"]
  I -->|no| K{"spec.abort を満たしたか"}
  K -->|yes| L["aborted result + replan reason"]
  K -->|no| M["execution 継続"]
  L --> N{"allow_replan?"}
  N -->|yes| O["runtime へ replan request"]
  N -->|no| P["runtime へ failure report"]
```

## 変更後の責務分離
### `behavior_planner`
- task の進行順序を持つ。
- 現在状態から、今実行すべき phase を選ぶ。
- `PhaseExecutionIntent` を生成する。
- `yaml` を読み、phase ごとの success / abort / replan 条件を `SuccessPolicy` / `AbortPolicy` として組み立てる。

### `planner`
- `PhaseExecutionIntent` を入力に `PhaseMotionPlan` を生成する。
- 幾何学的な target pose、waypoint、`JointTrajectory` を生成する。
- MoveIt を使った motion planning を担当する。

### `runtime`
- phase state machine を持つ。
- `PhaseExecutionIntent` と `PhaseMotionPlan` から `ExecutionPhaseSpec` を組み立てる。
- `ExecutionPhaseSpec` を dispatch する。
- replan を行うか、次 phase に進むかを決める。
- phase 条件の定数値そのものは持たない。

### `trajectory_tracking`
- `ExecutionPhaseSpec` をそのまま実行に変換する。
- `ExecutionPhaseSpec` に基づき success / abort / progress を判定する。
- phase 固有の閾値をハードコードしない。

### `ros2_control`
- `ExecutionPhaseSpec` で与えられた tolerance / timeout を controller semantics に適用する。
- フェーズ名や収穫タスク状態は持たない。

### `simulator`
- state read / command write / debug 可視化だけを担当する。
- execution policy を持たない。

## `scene_runtime` の扱い
`scene_runtime` からは次を外すべきである。

- `MOTION_TARGET_TOLERANCE_M`
- `target_tool_pose` を success 判定に使う責務
- waypoint 進行を実行意味論として持つ責務

残してよいものは次だけである。

- debug 表示用の active target
- `robot_tool_pose` / `tomato_pose` / `gripper_closed` の scene 状態
- physics based な grasp / detach の scene 更新

つまり `scene_runtime` の active target は「可視化用 mirror」であり、execution owner ではない。

## 実装ステップ
1. `robot/api` に `ExecutionPhaseSpec` と関連 enum / dataclass を追加する。
2. `robot/behavior_planner` を追加し、`BehaviorPlanner` と `PhaseExecutionIntentBuilder` を置く。
3. `PhaseExecutionIntentBuilder` が読む `yaml` を追加し、phase ごとの success / abort / replan 条件を外出しする。
4. `planner` は `PhaseExecutionIntent` を入力に `PhaseMotionPlan` を返す構造へ寄せる。
5. `runtime` で `intent + motion -> ExecutionPhaseSpec` を組み立てる。
6. `TrajectoryExecutionRequest` を `ExecutionPhaseSpec` ベースへ寄せる。
7. `state_store` の `target_pose` 中心設計を、`phase_goal_pose` / `active_waypoint_pose` 分離へ変更する。
8. `scene_runtime.target_tool_pose` を execution の正本として使う設計をやめる。
9. `joint_trajectory_controller_bridge` の timeout / abort 判定を `ExecutionPhaseSpec.abort` 参照へ変更する。
10. `runtime` の `POSITION_TOLERANCE_M` / `GRASP_CLOSE_TOLERANCE_M` などの定数を削除し、`yaml` 由来の policy へ移す。

## この案の利点
- 各フェーズ条件が 1 箇所で読める。
- `pregrasp` と `grasp` の目標意味論が混ざらない。
- timeout と abort 条件の変更が追いやすい。
- replan の理由が phase spec と 1 対 1 に対応する。
- simulator を純粋な実現系へ戻せる。

## この案で最初に直すべき点
最優先は次の 3 点である。

1. `target_tool_pose` に `phase goal` と `active waypoint` を同居させている構造をやめる。
2. `grasp` フェーズの success / timeout / abort を `ExecutionPhaseSpec` へ集約する。
3. `scene_runtime` の tolerance 判定を execution owner から外す。

この 3 点が終わると、その後の `ros2_control` tuning や Cartesian servo 補正の議論が正しい責務境界の上でできるようになる。
