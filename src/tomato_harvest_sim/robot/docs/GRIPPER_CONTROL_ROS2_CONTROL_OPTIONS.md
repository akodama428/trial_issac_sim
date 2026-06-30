# ros2_control モードにおけるグリッパー制御 設計案

---

## 全体アーキテクチャ（変更前・現状）

`CENTRALIZED_PHASE_EXECUTION_POLICY.md` が対象とする `behavior_planner` / `trajectory_tracking` /
`ros2_control` / `simulator` の範囲において、**最新実装**を反映した全体構成図。

### コンポーネント構成図

```mermaid
flowchart TB
  subgraph AppProc["Python / Isaac Sim プロセス"]
    direction TB

    APP["TomatoHarvestApplication\n(app/application.py)\nstep() で全体を 1 tick 進める"]

    subgraph RobotPkg["HarvestRuntime  (robot/runtime.py)"]
      direction TB

      RT["HarvestRuntime.step()\n① EST を呼ぶ（task_phase=DETECTING 時のみ）\n② BP.step(estimate) を呼ぶ（上位計画 → current_phase 確定）\n③ PLN を呼ぶ（BP が TARGET_FOUND を確定した場合のみ）\n   TARGET_FOUND→PLANNING 遷移も Runtime が行う\n④ Coordinator.run_cycle(effective_snapshot) を呼ぶ"]

      EST["TomatoTargetEstimator\n(perception/target_estimator.py)\n→ TargetEstimate"]
      PLN["MotionPlanner\n(motion_planner/moveit_service_bridge.py)\nGetMotionPlan service 経由で軌道計画\n→ HarvestMotionPlan"]
      BP["BehaviorPlanner\n(behavior_planner/planner.py)\nphase state machine（上位計画）\nPLANNING 以降の phase を管理\nmotion command を bridge へ publish\n※ motion_plan は受け取らない\n  TARGET_FOUND→PLANNING 遷移は HarvestRuntime が担う"]

      subgraph TT["TrajectoryTrackingCoordinator  (trajectory_tracking/coordinator.py)"]
        direction TB
        ESS["ExecutionStateStore\njoint_trajectory / gripper_closed /\nblockedSignature を管理"]
        TRK["TrajectoryTracker\nIK フォールバック &\ngripper 補間\n(allow_direct_drive=False で no-op)"]
        MON["ExecutionMonitor\nacceptance / result を監視"]
        AC["FollowJointTrajectoryActionClient\n(trajectory_tracking/action_client.py)"]
        PORT["Ros2ActionTrajectoryPort\n(trajectory_tracking/ros2_action_trajectory_port.py)\nFollowJointTrajectory action client"]
        AC --> PORT
      end

      RT -->|"① call"| EST
      RT -->|"② call\nstep(estimate)"| BP
      RT -->|"③ call\nBP が TARGET_FOUND を確定した後のみ"| PLN
      RT -->|"④ call\nrun_cycle(effective_snapshot)"| TT
      EST -->|"TargetEstimate"| RT
      PLN -->|"HarvestMotionPlan\n→ Runtime が task_phase=PLANNING に更新"| RT
      BP -->|"shared state を更新\n（task_phase / last_phase_motion_plan など）\n→ effective_snapshot 経由で Coordinator へ"| RT
      ESS --> TRK
      ESS --> MON
      MON --> AC
    end

    subgraph SimPkg["simulator"]
      direction TB
      SCR["IsaacSceneRuntime\n(simulator/scene_runtime.py)\nscene 状態・tomato 物理・\ngripper_closed state"]
      DRV["IsaacFrankaDriver\n(simulator/isaac_franka_driver.py)\nArticulationView 操作\narm[0-6] + finger[7-8]"]
      HWPORT["Ros2JointStateHardwarePort\n(simulator/ros2_joint_state_hardware_port.py)\n/joint_states を subscribe して\n関節観測値を Coordinator へ渡す"]
      BRIDGE["IsaacJointRos2Bridge\n(simulator/isaac_joint_ros2_bridge.py)\n/isaac_joint_states publish\n/isaac_joint_commands subscribe\n/clock publish"]
    end

    BRIDGE --> DRV
    HWPORT --> DRV
    TT --> HWPORT

    subgraph BridgePkg["Ros2LoopbackBridge  (api/bridge.py)"]
      BRDG["SceneSnapshot / MotionCommand /\nJointState / ControlCommand\nの ROS2 topic 経由輸送"]
    end

    APP --> RobotPkg
    APP --> SimPkg
    APP --> BridgePkg
    BP --> BridgePkg
    BridgePkg --> SCR
  end

  subgraph ROS2IF["ROS2 Interface（プロセス境界）"]
    direction LR
    ACT["/joint_trajectory_controller\n/follow_joint_trajectory\n(FollowJointTrajectory Action)"]
    JS["/joint_states\n(sensor_msgs/JointState)\narm 7 joints のみ"]
    ISS["/isaac_joint_states\n(sensor_msgs/JointState)\narm 7 joints"]
    CMD["/isaac_joint_commands\n(sensor_msgs/JointState)\narm 7 joints のみ"]
    CLK["/clock\n(rosgraph_msgs/Clock)\nsim time"]
    MV["MoveIt2 move_group\nGetMotionPlan service"]
  end

  subgraph CPP["franka_ros2_control  (C++ / 別 ROS2 ノード)"]
    direction TB
    JTC["JointTrajectoryController\n軌道補間・open_loop 追従\npath tolerance 判定\n(panda_joint1–7 のみ)"]
    HWI["IsaacSimHardwareInterface\n(ros2_control HardwareInterface)\n/isaac_joint_states → position_state_\nposition_command_ → /isaac_joint_commands"]
    JTC <--> HWI
  end

  subgraph PhysX["Isaac Sim Physics (PhysX)"]
    ARTC["ArticulationView\npanda_joint1–7  arm\npanda_finger_joint1/2  finger\n⚠ finger は現状どこからも指令されない"]
  end

  PORT -->|"goal / feedback / result"| ACT
  ACT --> JTC
  HWI -->|"publish"| JS
  HWPORT -->|"subscribe"| JS
  BRIDGE -->|"publish"| ISS
  ISS -->|"subscribe"| HWI
  HWI -->|"publish"| CMD
  CMD -->|"subscribe"| BRIDGE
  BRIDGE -->|"publish"| CLK
  CLK -->|"subscribe（use_sim_time）"| JTC
  PLN -->|"GetMotionPlan\nservice call"| MV
  DRV -->|"apply_action / set_joint_positions\narm + finger 全関節対応"| ARTC
  BRIDGE -->|"_apply_pending_command\nindices 0–6 のみ"| DRV
```

### 1 tick シーケンス（ros2_control モード）

```mermaid
sequenceDiagram
    participant APP as TomatoHarvestApplication
    participant BR as Ros2LoopbackBridge
    participant RT as HarvestRuntime
    participant EST as TomatoTargetEstimator
    participant PLN as MotionPlanner
    participant BP as BehaviorPlanner
    participant CO as TrajectoryTrackingCoordinator
    participant PORT as Ros2ActionTrajectoryPort
    participant HWPORT as Ros2JointStateHardwarePort
    participant JB as IsaacJointRos2Bridge
    participant JTC as C++ JointTrajectoryController
    participant DRV as IsaacFrankaDriver
    participant IS as Isaac Sim / PhysX

    Note over APP: TomatoHarvestApplication.step()
    APP->>BR: spin_once()
    APP->>RT: observe_scene(snapshot)

    Note over RT: HarvestRuntime.step()
    Note over RT: ① task_phase=DETECTING の時のみ
    RT->>EST: estimate(camera, tf)
    EST-->>RT: TargetEstimate

    Note over RT: ② BP を先に実行（上位計画）
    Note over RT: estimate を渡し、current_phase を確定する
    RT->>BP: step(snapshot, bridge, estimate=estimate)
    BP-->>RT: logs（DETECTING→TARGET_FOUND 遷移など）
    Note over BP: state.task_phase = TARGET_FOUND に更新

    Note over RT: ③ BP の current_phase が TARGET_FOUND なら PLN を呼ぶ（下位計画）
    Note over RT: PLN が返した plan を Runtime が state に書き込み TARGET_FOUND→PLANNING へ遷移
    RT->>PLN: plan(last_target_estimate, joint_state, tf, snapshot)
    PLN->>PLN: GetMotionPlan (MoveIt2 service)
    PLN-->>RT: HarvestMotionPlan
    Note over RT: state.last_harvest_motion_plan = plan\nstate.task_phase = PLANNING

    Note over RT: ④ Coordinator に実行させる
    RT->>CO: run_cycle(effective_snapshot)
    Note over CO: ExecutionStateStore.normalize_snapshot\njoint_trajectory / gripper_closed を読み取り

    CO->>HWPORT: read_state()
    HWPORT->>HWPORT: spin_once() → /joint_states 受信
    HWPORT-->>CO: HardwareStateSample (arm 7 joints)

    CO->>PORT: send_goal(TrajectoryExecutionRequest)
    PORT->>JTC: FollowJointTrajectory action goal (panda_joint1–7)

    CO->>PORT: step() → spin_once()
    PORT->>CO: feedback / result callback

    Note over CO,DRV: ⚠ gripper 制御パス（現状バグ）
    CO->>CO: _apply_tracking_command()\nallow_direct_drive=False → RETURN no-op
    Note over DRV,IS: finger joints[7-8] への指令なし\n→ 初期位置(0.0 rad=閉)のまま固定

    Note over JB: IsaacJointRos2Bridge.step()（毎 Isaac tick）
    JB->>JB: _publish_clock() → /clock
    JB->>JB: _publish_state() → /isaac_joint_states
    JB->>JB: spin_once() → /isaac_joint_commands 受信
    JB->>DRV: set_joint_velocity_targets(arm[0-6])
    DRV->>IS: apply_action(arm positions + velocities)

    JTC->>JTC: 軌道補間 → /isaac_joint_commands publish (arm only)

    Note over APP: physics_bridge.begin_physics_step()
    Note over APP: physics_bridge.finalize_physics_step()
    IS->>IS: PhysX step
```

### グリッパー制御の詰まり箇所（問題の要約）

```mermaid
flowchart LR
    SS["SceneSnapshot\ngripper_closed: bool"]
    ESS["ExecutionStateStore\ngripper_closed を保持"]
    TRK["TrajectoryTracker\npositions 7-8 = 0.04 or 0.0\nを TrackingCommand に込める"]
    CO["Coordinator\n_apply_tracking_command()"]
    NOOP["❌ allow_direct_drive=False\n→ RETURN no-op"]
    DRV["IsaacFrankaDriver\n呼ばれない"]
    IS["Isaac Sim\nfinger joints = 0.0 rad 固定\n（グリッパー閉じたまま）"]

    SS --> ESS --> TRK --> CO --> NOOP -.->|"届かない"| DRV -.-> IS
```

---

## 問題背景

ros2_control バックエンド導入時に `TrajectoryTrackingCoordinator` へ
`allow_direct_drive=False` を設定した。これは C++ `JointTrajectoryController (JTC)` と
Python ドライバーの直接書き込みが arm joints (indices 0–6) で競合するのを防ぐための修正。

しかし finger joints (indices 7–8, `panda_finger_joint1/2`) は JTC の制御対象外
（URDF には含まれない）にもかかわらず、同じ no-op ブランチに入ってしまう。
結果として finger joints がゼロ位置（closed = 0.0 rad）のまま固定され、
把持フェーズでグリッパーが開かない。

---

## 関節インデックス割付

| index | joint name | 制御者 |
|---|---|---|
| 0–6 | panda_joint1 〜 panda_joint7 | C++ JTC |
| 7 | panda_finger_joint1 | **現在誰も制御していない** |
| 8 | panda_finger_joint2 | **現在誰も制御していない** |

---

## 現状データフロー（問題あり）

### グリッパー指令パス（詰まっている）

```mermaid
flowchart TD
    BP[BehaviorPlanner\ngripper_closed: bool]
    SS[StateStore\ngripper_closed]
    TR[Tracker\n_merge_gripper_targets_into_positions\npositions 7-8 = 0.04 or 0.0]
    CO[Coordinator\n_apply_tracking_command]
    NOOP["❌ allow_direct_drive=False\n→ RETURN no-op\nfingerへの指令が届かない"]
    IS[Isaac Sim ArticulationView\nfinger joints = 0.0 rad 固定\n= グリッパー閉じたまま]

    BP --> SS --> TR --> CO --> NOOP --> IS
```

### アームパス（正常）

```mermaid
flowchart TD
    BP2[BehaviorPlanner\njoint trajectory panda_joint1–7]
    RP[Ros2ActionTrajectoryPort\nFollowJointTrajectory action goal]
    JTC[C++ JointTrajectoryController\n/isaac_joint_commands arm only]
    BR[IsaacJointRos2Bridge\n_apply_pending_command\nindices 0–6 のみ]
    IS2[Isaac Sim ArticulationView\narm joints のみ動く]

    BP2 --> RP --> JTC --> BR --> IS2
```

---

## Option A: Coordinator 内で finger を直接ドライブ

### 概要

`allow_direct_drive=False` のままで arm joints[0–6] への書き込みは引き続き no-op にする。
Coordinator に `allow_gripper_direct_drive` フラグを追加し、
`_apply_tracking_command()` が TrackingCommand から finger 部分（indices 7–8）だけを
抜き出して `IsaacFrankaDriver` へ直接書き込む。

JTC は arm のみを管理するため、finger への直接書き込みと競合しない。

### アーキテクチャ図

```mermaid
flowchart LR
    subgraph Coordinator["TrajectoryTrackingCoordinator"]
        direction TB
        FLAG["allow_direct_drive = False\nallow_gripper_direct_drive = True ← 追加"]
        ATC["_apply_tracking_command(command)\n├ arm[0-6]: no-op\n└ finger[7-8]: driver へ直接書き込み ← 追加"]
        FLAG --> ATC
    end

    subgraph ArmPath["アームパス（変更なし）"]
        RP[Ros2ActionTrajectoryPort\nFollowJointTrajectory]
        JTC[C++ JointTrajectoryController]
        BR[IsaacJointRos2Bridge\n_apply_pending_command\nindices 0–6]
    end

    subgraph FingerPath["フィンガーパス（新規）"]
        DRV[IsaacFrankaDriver\nset_joint_positions\npositions 7:9]
    end

    IS[Isaac Sim ArticulationView\narm 0–6: JTC 経由\nfinger 7–8: Coordinator 直接]

    Coordinator -->|trajectory goal| RP
    RP --> JTC --> BR --> IS
    ATC -->|positions 7:9| DRV --> IS
```

### データフロー

```mermaid
sequenceDiagram
    participant BP as BehaviorPlanner
    participant SS as StateStore
    participant TR as Tracker
    participant CO as Coordinator
    participant DRV as IsaacFrankaDriver
    participant IS as Isaac Sim

    BP->>SS: gripper_closed = false
    SS->>TR: gripper_closed
    TR->>CO: TrackingCommand\npositions[7-8] = 0.04
    Note over CO: allow_gripper_direct_drive=True\nfingerのみ抽出
    CO->>DRV: set_joint_positions(positions[7:9])
    DRV->>IS: ArticulationView\npanda_finger_joint1/2 = 0.04 rad ✓
```

### 実装変更範囲

| ファイル | 変更内容 |
|---|---|
| `coordinator.py` | `allow_gripper_direct_drive: bool = True` パラメータ追加<br>`_apply_tracking_command()` に finger-only 書き込みパス追加 |
| `isaac_franka_driver.py` | 既存 `set_joint_positions_with_debug()` をそのまま利用（positions[7:9] だけ渡す） |
| `isaac_viewer.py` | 変更不要（デフォルト `True` を維持） |

### メリット・デメリット

| | 内容 |
|---|---|
| ✅ | 変更ファイルが最小（coordinator.py のみ実質変更） |
| ✅ | JTC との競合なし（finger は JTC 管理外） |
| ✅ | gripper_closed の真実が StateStore に一元化されたまま |
| ✅ | Tracker の step_toward_joint_positions 補間ロジックをそのまま再利用 |
| ⚠️ | Coordinator が Isaac Sim API を間接的に呼ぶ（レイヤー越え残存） |
| ⚠️ | `allow_direct_drive` / `allow_gripper_direct_drive` の 2 フラグが似た名前で混乱しやすい |

---

## Option B: IsaacJointRos2Bridge がグリッパーを管理

### 概要

グリッパー制御を Coordinator から完全に切り離し、`IsaacJointRos2Bridge` が
gripper_closed シグナルを受け取り、`step()` ごとに finger joints を直接ドライブする。

シグナルの受け渡しには `GripperStateHolder`（共有参照）を新設する（B-1案）。
または ROS2 topic `/gripper_command` で伝達する（B-2案）。

### アーキテクチャ図（B-1: 共有参照）

```mermaid
flowchart LR
    subgraph Coordinator["TrajectoryTrackingCoordinator（変更最小）"]
        ATC["_apply_tracking_command()\n完全 no-op のまま\n+ gripper_closed を Holder へ書き込む"]
    end

    subgraph Holder["GripperStateHolder（新規）"]
        GS["gripper_closed: bool\nthread-safe な共有参照"]
    end

    subgraph Bridge["IsaacJointRos2Bridge（変更）"]
        STEP["step()\n├ _publish_clock()\n├ _publish_state()\n├ spin_once()\n├ _apply_pending_command() arm\n└ _apply_gripper_command() finger ← 追加"]
    end

    subgraph ArmPath["アームパス（変更なし）"]
        RP[Ros2ActionTrajectoryPort]
        JTC[C++ JointTrajectoryController]
        BR_ARM["_apply_pending_command()\nindices 0–6"]
    end

    DRV[IsaacFrankaDriver\nset_joint_positions\npositions 7:9]
    IS[Isaac Sim ArticulationView]

    Coordinator -->|write| Holder
    Bridge -->|read| Holder
    STEP --> DRV --> IS
    Coordinator -->|trajectory goal| RP --> JTC --> BR_ARM --> IS
```

### データフロー（B-1: 共有参照）

```mermaid
sequenceDiagram
    participant BP as BehaviorPlanner
    participant SS as StateStore
    participant CO as Coordinator
    participant GH as GripperStateHolder
    participant BR as IsaacJointRos2Bridge
    participant DRV as IsaacFrankaDriver
    participant IS as Isaac Sim

    BP->>SS: gripper_closed = false
    SS->>CO: gripper_closed (TrackingCommand 経由)
    Note over CO: no-op のまま
    CO->>GH: gripper_closed = false（毎 tick 書き込み）

    loop 毎 step()
        BR->>GH: gripper_closed を読み取り
        BR->>DRV: set_joint_positions([0.04, 0.04])
        DRV->>IS: panda_finger_joint1/2 = 0.04 rad ✓
    end
```

### データフロー（B-2: ROS2 topic）

```mermaid
sequenceDiagram
    participant CO as Coordinator
    participant PUB as ROS2 pub\n/gripper_command (Bool)
    participant SUB as ROS2 sub\n/gripper_command
    participant BR as IsaacJointRos2Bridge
    participant IS as Isaac Sim

    CO->>PUB: publish(gripper_closed=false)
    PUB-->>SUB: Bool message
    SUB->>BR: _on_gripper_command(msg)

    loop 毎 step()
        BR->>IS: set_joint_positions([0.04, 0.04])
    end
```

### 実装変更範囲

| ファイル | 変更内容 |
|---|---|
| `coordinator.py` | `_apply_tracking_command()` から GripperStateHolder へ `gripper_closed` を書き込む処理追加 |
| `isaac_joint_ros2_bridge.py` | `GripperStateHolder` 参照を受け取り<br>`_apply_gripper_command()` メソッド追加<br>`step()` 末尾で呼び出し |
| `isaac_viewer.py` | `GripperStateHolder` インスタンスを生成し Coordinator と Bridge 両方へ渡す |
| `gripper_state_holder.py`（新規） | `GripperStateHolder` dataclass（1 ファイル） |

### メリット・デメリット

| | 内容 |
|---|---|
| ✅ | Coordinator が Isaac Sim API を一切呼ばない（レイヤー境界が明確） |
| ✅ | グリッパー制御が Bridge 内に集約される |
| ✅ | 将来 Franka gripper action server への差し替えが容易 |
| ⚠️ | 新しい通信チャネル（Holder または topic）が必要 |
| ⚠️ | 変更ファイル数が Option A より多い（3–4 ファイル） |
| ⚠️ | `gripper_closed` が StateStore と GripperStateHolder の 2 箇所に存在（同期ズレリスク） |
| ⚠️ | Tracker の step_toward_joint_positions 補間が使えない（Bridge 側で独自実装が必要） |

---

## 比較まとめ

```mermaid
quadrantChart
    title 実装コスト vs アーキテクチャ品質
    x-axis 実装コスト 低 --> 高
    y-axis アーキテクチャ品質 低 --> 高
    quadrant-1 理想
    quadrant-2 品質優先
    quadrant-3 避けるべき
    quadrant-4 速度優先
    Option A: [0.2, 0.35]
    Option B-1: [0.5, 0.75]
    Option B-2: [0.75, 0.65]
```

| 観点 | Option A | Option B-1 | Option B-2 |
|---|---|---|---|
| 変更ファイル数 | 1 | 3–4 | 3–4 |
| レイヤー境界 | finger のみ越える（残存）| 明確 | 明確 |
| gripper 補間ロジック再利用 | ✅ そのまま | ❌ 独自実装必要 | ❌ 独自実装必要 |
| gripper_closed の単一真実 | ✅ StateStore のみ | ⚠️ Holder と二重 | ⚠️ Holder と二重 |
| 将来 franka_gripper 差し替え | ❌ 難しい | ✅ 容易 | ✅ 容易 |
| 実装コスト | 低 | 中 | 高 |

**推奨**: 当面の動作確認を優先するなら **Option A**。
アーキテクチャの境界を重視するなら **Option B-1（共有参照）**。
