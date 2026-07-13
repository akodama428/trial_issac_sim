# Issue #39/#40 安定化レポート: 特異近傍のhome経由開始と起動flake対策

## 目的と、この検証が次につながる点

Issue #37 (物理固着flakeの根本調査) の最終検証で残った失敗は2種類だった。

1. `near_singularity_extended` の固着型失敗 — 伸展特異姿勢近傍ではseed収束IKも窓付きgoal samplerも安定せず、Issue #37 の三重fallbackでも遠いIK枝を踏むことがある (Issue #39)
2. `controller_manager` 起動タイムアウト — 計画・実行系と無関係なスタック起動のインフラflakeが、成功率計測を汚染する (Issue #40)

本対応でこの2つを塞ぎ、初期姿勢10ケースE2Eの失敗を「物理把持 (Issue #33) と真の計画・実行課題」だけに絞り込む。あわせてIssue #41 (JTC指令凍結) の初期調査を行い、凍結メカニズムの仮説を特定した。

## 変更内容

### 1. home差が大きい初期姿勢のhome経由開始 (Issue #39)

Issue #28 検討時に提案された「タスク実行前のホームポジション移動」を、**必要なケースだけに限定して**実装した。

- 初期full-chain計画で、現在構成とhome構成の最大関節差が閾値 (`TOMATO_HARVEST_HOME_VIA_THRESHOLD_RAD`、既定1.2 rad、0で無効) を超える場合、pregrasp軌道を「現在→home」+「home→pregrasp」の連結として生成する (`should_start_via_home()` / `_plan_pregrasp_via_home()`)
- home区間は**関節空間goal (IK不要)** で計画するため、特異近傍でもIK不安定性の影響を受けない。home以降は通常ケースと同一の挙動になる
- via-home計画が失敗した場合は従来の直行pregrasp計画へfallbackする
- phase状態機械・executor契約は無変更 (pregrasp trajectoryの内容として実現)

10ケースの初期姿勢とhome構成の最大関節差は、near_singularity_extended だけが2.0 radで、他は最大1.05 rad (extended_far)。閾値1.2により**該当ケースのみ**がhome経由になる。

### 2. スタック起動の強化と起動flakeの分類 (Issue #40)

- `controller_manager` 起動待機を15秒→45秒へ延長し、タイムアウト時は `ros2_control_node` を1回だけ再起動して再待機する
- 最終失敗時に `STACK_STARTUP_FAILED` マーカーをログへ出力し、集計スクリプトが `failure_reason=stack_startup_failed` として通常の実行失敗と区別する

### 3. JTC指令凍結の初期調査 (Issue #41、調査のみ)

`franka_controllers.yaml` の設計確認により、Issue #37 で観測した凍結・abortの機序仮説を特定した (詳細は #41 コメント)。

- 関節ごとの `goal: 0.0` により**最終位置toleranceは無効化済み**。観測された`goal_tolerance_violated`の実体は**静止判定** (`stopped_velocity_tolerance: 0.05` が `goal_time: 5.0` 以内に満たされない) である
- 指令凍結は `open_loop_control: true` のhold挙動 (最終サンプル位置の保持) と符合する。hold中に新goalが指令へ反映されなかった点は、executorのcancel-in-flight直後の新goal送信とJTC受理順序の競合が疑われ、#41で対応する

## 変更後の全体アーキテクチャ

凡例: 赤は今回変更、緑は既存利用、灰は変更範囲外。

```mermaid
flowchart LR
  Camera[Camera nodes] --> Perception[perception node]
  Perception --> Behavior[behavior planner node]
  Simulator[Isaac Sim node] --> State[joint state / scene snapshot]
  Behavior --> Manager[trajectory planner node]
  State --> Manager
  Manager --> Global[global planner<br/>MoveIt bridge<br/>+ home経由pregrasp計画]
  Manager --> Event[hybrid planning event]
  Event --> Local[local planner node]
  Global --> Arbitration[motion command node]
  Local --> Arbitration
  Arbitration --> Executor[trajectory executor node]
  Executor --> Controller[joint trajectory controller]
  Controller --> Simulator
  CI[initial-pose matrix CI<br/>+ 起動リトライ / 失敗分類] -.-> Manager
  classDef changed fill:#ffd6d6,stroke:#c62828,stroke-width:3px
  classDef existing fill:#d9f2df,stroke:#238636
  classDef outside fill:#eeeeee,stroke:#777777
  class Global,CI changed
  class Manager,State,Event,Local,Arbitration,Executor,Controller existing
  class Camera,Perception,Behavior,Simulator outside
```

## 変更差分の詳細アーキテクチャ

```mermaid
flowchart TD
  Snapshot[scene snapshot / joint states]

  subgraph TrajectoryNode[trajectory_planner_node]
    FullChain[initial full-chain planning]
  end

  subgraph Bridge[MoveIt bridge / global planner]
    Judge[should_start_via_home<br/>home差 > 1.2 rad?]
    ViaHome[_plan_pregrasp_via_home<br/>現在→home 関節空間goal<br/>+ home→pregrasp 連結]
    Direct[直行pregrasp計画<br/>三重fallback goal選択]
    Judge -->|該当| ViaHome
    Judge -->|非該当| Direct
    ViaHome -->|失敗時fallback| Direct
  end

  subgraph CIScripts[CI scripts]
    Wait[controller_manager待機 45秒<br/>+ ros2_control_node再起動リトライ]
    Marker[STACK_STARTUP_FAILEDマーカー]
    Classify[summarize: stack_startup_failed分類]
    Wait -->|最終失敗| Marker --> Classify
  end

  MoveGroup[move_group services]
  PlanTopic[harvest motion plan topic]

  Snapshot --> FullChain
  FullChain --> Judge
  ViaHome --> MoveGroup
  Direct --> MoveGroup
  ViaHome --> PlanTopic
  Direct --> PlanTopic
  classDef changed fill:#ffd6d6,stroke:#c62828,stroke-width:3px
  classDef nodebox fill:#fff8e1,stroke:#9a6700,stroke-width:2px
  class Judge,ViaHome,Wait,Marker,Classify changed
  class TrajectoryNode,Bridge,CIScripts nodebox
```

黄色の大枠がROS 2 node / スクリプト境界、赤が今回追加・変更した処理を表す。

## 変更ファイル

| ファイル | 変更 |
|---|---|
| `robot/motion_planner/moveit_service_bridge.py` | `should_start_via_home()`、`_plan_pregrasp_via_home()` (home区間+pregraspの連結、失敗時直行fallback) |
| `scripts/run_ros2_components.sh` | controller_manager待機45秒化・再起動リトライ・`STACK_STARTUP_FAILED`マーカー |
| `scripts/ci/summarize_initial_pose_e2e.py` | `stack_startup_failed`の失敗分類 |

## 検証

- unit test: pytest 239 passed + gtest (コンテナCI同等)。追加は、home経由判定 (3件: 特異近傍該当・通常非該当・閾値0で無効)、起動失敗分類 (1件)
- **near_singularity_extended 3回連続実行: 3/3 PASS、全runでabortゼロ**。via-home経路の発動 (home区間50点+pregrasp16点の連結) をログで確認した。このケースはIssue #28以降、単発・マトリクスを通じて安定完走したことがなく、abortゼロでの3連続成功は初である

<!-- MATRIX39_RESULTS -->
