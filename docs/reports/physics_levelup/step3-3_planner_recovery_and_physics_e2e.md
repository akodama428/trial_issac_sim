# Step 3-3 Contact観測再構築・Servo終端Pose Tracking・Physics E2E

## 目的

Step 3-2の外部ベストプラクティス調査で推奨した案Aと案Bを実装し、観測系と把持終端制御を同時に改善する。案AはPhysX contact reportを左右fingerへ確実に対応付け、同一physics stepの力積と力を記録する。案Bは`MOVING_TO_GRASP`だけをMoveIt ServoのPose commandへ切り替え、閉爪前の6D終端整列を閉ループ化する。

対象コミット:

- `3a4108f` `Implement robust per-finger contact observation`
- `d02c071` `Add Servo terminal pose tracking for grasp`

## 変更後の全体アーキテクチャ

```mermaid
flowchart LR
  classDef changed fill:#d5f2dc,stroke:#287a3d,stroke-width:3px,color:#111
  classDef existing fill:#eee,stroke:#666,color:#111
  classDef observed fill:#dcecff,stroke:#316ca6,stroke-width:2px,color:#111
  classDef failed fill:#ffd6d6,stroke:#a62929,stroke-width:3px,color:#111

  Detector["tomato_detector_node"]:::existing
  Behavior["behavior_planner_node"]:::existing
  Planner["trajectory_planner_node<br/>global JointTrajectory"]:::existing
  Command["motion_command_node<br/>phase goal pose + trajectory"]:::existing
  Adapter["servo_execution_adapter<br/>phase別command arbitration"]:::changed
  JointJog["JointJog<br/>pregrasp / pull / place / home"]:::existing
  PoseTrack["PoseStamped tracking<br/>MOVING_TO_GRASP only"]:::changed
  Servo["MoveIt Servo<br/>PlanningScene / limits / smoothing"]:::existing
  JTC["joint_trajectory_controller"]:::existing
  Panda["Isaac Sim Franka"]:::existing
  Contact["PhysX contact reports<br/>actor + collider / all points"]:::changed
  Force["sequence-aligned<br/>impulse [N s] + force [N]"]:::changed
  Strategy["FrictionGraspStrategy<br/>bilateral fail-closed gate"]:::observed
  Timeout["E2E: pose target timeout"]:::failed

  Detector --> Behavior --> Planner --> Command --> Adapter
  Adapter --> JointJog --> Servo
  Adapter --> PoseTrack --> Servo
  Servo --> JTC --> Panda
  Panda --> Contact --> Force --> Strategy --> Behavior
  PoseTrack -. "今回未到達" .-> Timeout
```

pregraspまでは既存JointJogの閉ループ実行を維持する。`MOVING_TO_GRASP`では、planに含まれるruntime tool goalを`panda_link8` goalへ変換し、Servoのabsolute pose commandへ切り替える。把持位置へ到達するまではgripper openを維持し、成功後に既存phase遷移から閉爪する。

## 案A: contact観測系の変更箇所アーキテクチャ

```mermaid
flowchart TB
  classDef changed fill:#d5f2dc,stroke:#287a3d,stroke-width:2px,color:#111
  classDef boundary fill:#dcecff,stroke:#316ca6,stroke-width:2px,color:#111

  Header["ContactEventHeader<br/>actor0/1 + collider0/1<br/>offset + count"]:::boundary
  MatchActor["actor pair照合<br/>両スロット対称"]:::changed
  MatchCollider["collider pair fallback<br/>articulation child prim対応"]:::changed
  Range["contact_data範囲をclamp<br/>不正範囲はfail-closed"]:::changed
  Sum["全contact pointの<br/>impulse norm合算"]:::changed
  Convert["force = impulse / physics dt"]:::changed
  Log["PhysicsObs<br/>seq / impL / impR / forceL / forceR"]:::changed
  Snapshot["SceneSnapshot<br/>left/right force N"]:::boundary

  Header --> MatchActor
  MatchActor -->|未一致| MatchCollider
  MatchActor -->|一致| Range
  MatchCollider --> Range --> Sum --> Convert
  Convert --> Log
  Convert --> Snapshot
```

### 実装内容

- actor0/actor1の順序に依存せずfingerとtomatoのpairを照合する。
- actor pathがarticulation rootでfinger linkを識別できない場合、collider0/collider1の子prim pathを使う。
- `contact_data_offset`と`num_contact_data`が示す全contact pointを合算する。配列外参照は行わない。
- `FingerContactImpulses [N s]`から`FingerContactForces [N]`への変換をphysics観測モジュールへ集約した。
- `PhysicsObs`へphysics sequence IDと左右のimpulse/forceを同一行で記録する。

## 案B: Servo終端Pose Trackingの変更箇所アーキテクチャ

```mermaid
sequenceDiagram
  participant B as behavior_planner
  participant M as motion_command
  participant A as servo_execution_adapter
  participant S as MoveIt Servo
  participant TF as TF panda_link0 to panda_link8
  participant G as gripper

  B->>M: MOVING_TO_PREGRASP
  M->>A: JointTrajectory endpoint
  A->>S: JOINT_JOG command type + JointJog
  S-->>A: joint feedback converged
  A-->>B: execution_status succeeded

  B->>M: MOVING_TO_GRASP
  M->>A: phase_goal_pose + JointTrajectory
  A->>G: keep open
  A->>S: POSE command type + PoseStamped
  loop 50 Hz
    A->>TF: current panda_link8 pose
    A->>S: absolute pose target
    A->>A: position <= 5 mm and orientation <= 0.03 rad
  end
  alt tolerance sustained for 3 samples
    A-->>B: execution_status succeeded
    B->>G: close at AT_GRASP
  else deadline exceeded
    A-->>B: servo_target_timeout
  end
```

### 実装内容

- Pose Tracking対象を`PhaseId.MOVING_TO_GRASP`へ限定した。
- runtime tool poseからMoveIt control link `panda_link8`への既存`58.4 mm` local offsetを姿勢込みで変換する。
- Servo command type serviceをphaseに応じてJOINT_JOG/POSEへ切り替え、非同期応答の競合を防ぐ。
- `panda_link0 -> panda_link8`のTF実測から位置・姿勢誤差を計算する。
- 位置`5 mm`以下かつ姿勢`0.03 rad`以下が3周期連続した場合のみ成功とする。
- Pose Tracking開始時はgripperをopenのまま維持し、閉爪を`AT_GRASP`以降へ遅延する。

## テスト

### Unit / repository test

| 項目 | 結果 |
|---|---|
| repository pytest | **PASS: 248件、2件skip** |
| contact actor順序反転 | PASS |
| articulation actor / collider子prim fallback | PASS |
| 複数contact point合算 | PASS |
| 不正contact data範囲のfail-closed | PASS |
| impulse / physics dtによるforce換算 | PASS |
| grasp phaseだけPose Trackingを選択 | PASS |
| runtime toolから`panda_link8` goalへの変換 | PASS |
| 6D tolerance判定 | PASS |
| tracking中の閉爪遅延 | PASS |

## Physics E2E条件

実行日: 2026-07-16

```bash
CI_HEADLESS_STEPS=3600 \
CI_GRASP_MODE=physics \
TOMATO_HARVEST_DEBUG_PHYSICS_GRASP=1 \
CI_E2E_TIMEOUT_SEC=2400 \
bash scripts/ci/run_e2e.sh
```

| 項目 | 値 |
|---|---|
| 初期姿勢 | `default` |
| 把持モード | `physics` |
| headless上限 | 3600 steps |
| Servo制御周期 | 50 Hz |
| Pose位置許容差 | 0.005 m |
| Pose姿勢許容差 | 0.03 rad |
| 必要連続到達 | 3 samples |
| 両指最小力 | 各1.0 N |
| friction継続 | 3 physics steps |

## Physics E2E結果

総合判定: **FAIL。pregraspは成功したが、grasp Pose Trackingがtimeoutし、接触評価へ到達しなかった。**

| 項目 | 結果 |
|---|---|
| planning | 成功、350.856 ms |
| phase | `idle -> detecting -> target_found -> moving_to_pregrasp -> moving_to_grasp` |
| pregrasp Servo | 成功、4373.285 ms、最大joint誤差0.005694 rad |
| Servo Pose mode切替 | 成功、command type `2` |
| grasp試行1 | `servo_target_timeout`、開始から約6.34 s |
| grasp再計画後試行2 | `servo_target_timeout`、開始から約5.16 s |
| `AT_GRASP`到達 | なし |
| gripper close | なし（tracking中open維持は設計どおり） |
| PhysX finger contact event | 0件 |
| 非zero contact impulse / force sample | 0件 |
| FrictionGraspStrategy評価 | 未到達 |
| 最終結果 | completion markerなし、E2E exit 1 |

### Gate判定

| Gate | 判定 | 根拠 |
|---|---|---|
| G0 planner / pregrasp復旧 | PASS | planning成功、pregrasp Servo到達 |
| G1 Pose command mode切替 | PASS | Servo command type `2` readyを記録 |
| G2 終端Pose収束 | **FAIL** | 2回とも`servo_target_timeout` |
| G3 観測整合 | BLOCKED | grasp contactが発生せず案AをE2E評価できない |
| G4 有効両指接触 | BLOCKED | `AT_GRASP`未到達 |
| G5 friction hold | BLOCKED | FrictionGraspStrategy未評価 |

## 解析

### 1. 案Bのcommand arbitrationまでは機能した

pregraspでは従来JointJogが収束し、`moving_to_pregrasp -> moving_to_grasp`へ遷移した。その直後にServo command type `2`への切替成功が記録されているため、phase選択とservice切替は動作している。

### 2. Pose targetの到達観測がなくtimeoutした

Pose Tracking中はjoint追従誤差ではなく6D TF誤差を使うため、現行`execution_status`には周期的なposition/orientation errorが出ていない。ログからは次の候補をまだ分離できない。

- PoseStampedがServoに受理されているがEEFが動いていない。
- EEFは動いているが`panda_link8` goal変換またはframe semanticsが一致していない。
- TF lookupが成立せず、到達判定だけが更新されていない。
- planned trajectoryの所要時間を基準にしたdeadlineがPose trackingには短い。

timeout延長だけを対策にはしない。まずcommand publish数、Servo status、TF lookup成否、目標/現在6D誤差を同一周期で記録し、停止箇所を確定する必要がある。

### 3. 案Aのunit境界は改善したが、E2E Gateは未評価

今回のE2Eはgripper closeと接触前に停止したため、actor/collider照合や左右force整合の実シーン検証には至っていない。`seq / impulse / force`形式が3600 stepを通して出力され、全sampleがzeroだったことは確認できたが、これは接触が無かったためであり案A成功の証拠にはしない。

## 次の改善

優先度P0でPose Tracking観測を追加する。

1. Pose command publishごとにsequence、frame、target 6Dを記録する。
2. TF lookup失敗をcountし、例外種別と最終成功時刻を記録する。
3. current `panda_link8` pose、position error、orientation error、Servo statusを同一sampleで記録する。
4. `panda_link8`とruntime toolのoffsetを同じTF snapshot上で照合する。
5. 原因修正後に同条件E2Eを再実行し、`AT_GRASP`到達後に案Aの左右actor/collider/force整合を評価する。

## 結論

案Aと案Bのコードおよびunit testは導入でき、repository testは全件成功した。統合E2Eでは既存pregrasp経路とPose mode切替までは成功したが、Pose Tracking終端収束がtimeoutし、把持接触へ到達しなかった。したがって今回の変更を「摩擦保持改善成功」とは判定しない。次のGateはPose commandとTF誤差の周期観測を追加してG2を通し、その後に案Aの実接触整合を検証することである。
