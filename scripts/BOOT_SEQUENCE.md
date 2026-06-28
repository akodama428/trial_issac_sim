# 起動シーケンス

`PYTHONPATH=src ./python.sh scripts/run_harvest_viewer.py --transport ros2` を実行したときの動線。

## 実行コマンドと主なオプション

```bash
PYTHONPATH=src ./python.sh scripts/run_harvest_viewer.py \
  --transport ros2        # ブリッジ種別: in_memory | ros2 | auto
  --grasp-mode success    # success=物理グラスプ有効, failure=失敗デモ
  --headless              # GUIなしで実行（--headless-steps でステップ数指定）
  --auto-start            # 起動直後に自動でSTARTボタンを押す
  --camera-view fixed     # 初期カメラ: fixed | hand
  --timeout-seconds 0     # GUI モードでの自動終了秒数（0=無制限）
```

---

## フェーズ 1: Isaac Sim プロセス起動

```
scripts/run_harvest_viewer.py
  └─ isaac_viewer.main()
       ├─ SimulationApp 生成          # Isaac Sim 物理エンジン・レンダラ起動
       ├─ build_review_scene_plan()   # シーン構成パラメータ読み込み
       └─ _build_scene()             # USD stage にオブジェクト配置
            ├─ stage 作成・重力設定 (PhysicsScene)
            ├─ 地面 (Cube)
            ├─ 照明 (DistantLight / SphereLight)
            ├─ Franka Panda USD ロード (公式アセット参照)
            ├─ branch / stem / tomato / tray 追加
            ├─ fixed camera / hand camera 追加
            ├─ SceneRuntimeDisplay セットアップ (デバッグ可視化)
            └─ IsaacPhysicsHarvestBridge (grasp-mode=success 時のみ)
                 └─ PhysX 関節・コンタクト制約を準備
```

## フェーズ 2: Isaac Sim タイムライン開始・初期フレーム待機

```
_start_timeline_playback()      # omni.timeline.play()
_pump_updates(frame_count=4)    # 物理初期化のためのフレーム送り
_wait_for_first_frame()         # ビューポートに最初のフレームが描画されるまで待機
```

## フェーズ 3: ロボットシステム全体の構築

Isaac 固有のハードウェア層と、それを注入されたロボットシステムをまとめて構築する。

```
# Isaac 固有の実装（simulator/ に留まる）
IsaacFrankaDriver              # Franka prim への関節読み書き
IsaacRos2ControlSystem         # HardwareControlPort 実装
TrajectoryTrackingCoordinator  # 軌道追従コーディネーター
  ├─ driver=IsaacFrankaDriver
  ├─ hardware_control_port=IsaacRos2ControlSystem
  └─ trajectory_execution_port=JointTrajectoryControllerBridge

# ロボットシステム構築 — coordinator を依存性注入
create_tomato_harvest_application(executor=franka_executor)
  ├─ IsaacSceneRuntime          # シミュレータ側ステート管理
  ├─ HarvestRuntime             # ロボットシステム全体のオーケストレーター
  │    ├─ TomatoTargetEstimator # perception
  │    ├─ MotionPlanner         # 運動計画
  │    ├─ BehaviorPlanner       # タスクフェーズ状態機械
  │    └─ executor              # 注入済み TrajectoryTrackingCoordinator
  └─ bridge                     # InMemoryBridge または Ros2Bridge

ControlPanelController.boot()
  └─ TomatoHarvestApplication.boot()
       ├─ IsaacSceneRuntime.boot()  → ScenePhase.READY
       ├─ HarvestRuntime.boot()     → RobotRuntimeState.READY / HarvestTaskPhase.IDLE
       └─ 初期スナップショット配信・受信
```

> **設計方針**: `IsaacFrankaDriver` と `IsaacRos2ControlSystem` は Isaac Sim 固有のため
> `simulator/` に留める。`TrajectoryTrackingCoordinator` だけを `HarvestRuntime` に
> 注入することで、テスト環境では `executor=None` のまま動作する。

---

## フェーズ 4: メインループ（1フレームの処理順）

ユーザーが **Start** を押すと `HarvestTaskPhase` が `DETECTING` に遷移し、以下のループが回り始める。

```
# Read → Compute → Actuate の制御ループ

┌─ 1フレーム ─────────────────────────────────────────────────────────────────┐
│                                                                             │
│  [Read]                                                                     │
│  ① robot.observe_scene()     # 前フレームの物理結果を受信                  │
│       └─ state.last_scene_snapshot を最新化                                │
│                                                                             │
│  [Compute]                                                                  │
│  ② control_controller.step_runtime()                                       │
│       └─ TomatoHarvestApplication.step()                                   │
│            └─ HarvestRuntime.step()  ← ロボット制御出力を生成              │
│                 │                                                           │
│                 ├─ [joint sync]  executor → bridge.publish_joint_state()   │
│                 │                                                           │
│                 ├─ [perception]  TomatoTargetEstimator.estimate()          │
│                 │                → TargetEstimate                          │
│                 │                                                           │
│                 ├─ [motion plan] MotionPlanner.plan()                      │
│                 │                → HarvestMotionPlan                       │
│                 │                                                           │
│                 ├─ [behavior]    BehaviorPlanner.step()                    │
│                 │                → state.last_phase_motion_plan            │
│                 │                → bridge.publish_motion_command()         │
│                 │                  (gripper コマンドなど一部は bridge 経由) │
│                 │                                                           │
│                 └─ [executor]    TrajectoryTrackingCoordinator.run_cycle() │
│                       ├─ 入力: state.last_phase_motion_plan  ┐ HarvestRuntime
│                       │         state.last_scene_snapshot    ┘ が直接渡す  │
│                       ├─ IsaacRos2ControlSystem.write_command()            │
│                       │    └─ IsaacFrankaDriver → Franka 関節制御          │
│                       ├─ consume_replan_request() → replan_motion()        │
│                       └─ log_post_update_debug_snapshot()  # デバッグログ  │
│                                                                             │
│  [Actuate]                                                                  │
│  ③ simulation_app.update()                                                 │
│       │                                                                     │
│       │  ── headless / GUI 共通 ──────────────────────────────────────     │
│       ├─ executor が write した関節コマンドを物理に反映                     │
│       └─ [grasp-mode=success 時のみ] IsaacPhysicsHarvestBridge             │
│               グラスプ成否・トマト落下を判定し SceneSnapshot に反映        │
│               → 次フレームの ① robot.observe_scene() で読まれる            │
│                                                                             │
│       │  ── GUI のみ ────────────────────────────────────────────────     │
│       ├─ _sync_runtime_visuals()   # デバッグ可視化 USD prim 書き込み      │
│       └─ レンダリング・ビューポート更新                                     │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

> **注**: メインループの分岐は 2 階層ある。
>
> **第1層 — executor の有無**（Isaac Sim か Python テストかを分ける）
>
> | | Isaac Sim（headless / GUI とも） | Python テスト（executor=None） |
> |---|---|---|
> | 物理ステップ | `simulation_app.update()` が担う | `advance()` が担う |
> | `apply_motion_command()` | 不要（coordinator が plan を直接受け取る） | 必要（`advance()` の目標記録） |
> | executor ブロック | 実行する | スキップ |
>
> **第2層 — headless / GUI**（`simulation_app.update()` 内のみ）
>
> | | GUI モード | headless モード |
> |---|---|---|
> | 物理演算 | 実行 | 実行 |
> | レンダリング・ビューポート更新 | 実行 | スキップ |
>
> headless でも executor は動く。headless/GUI の差は `simulation_app.update()` の内部に閉じる。

### データフローの設計方針

```
【Isaac Sim パス — executor あり（headless / GUI とも）】

  BehaviorPlanner → state.last_phase_motion_plan ──────────────────────┐
                                                                        ▼
                                           TrajectoryTrackingCoordinator
                                                  (HarvestRuntime 内)
                                                  IsaacRos2ControlSystem.write_command()
                                                       └─ IsaacFrankaDriver → 関節制御

  ※ apply_motion_command() / advance() は呼ばない
     coordinator が plan を直接受け取るため scene_runtime 経由不要


【Python テストパス — executor=None】

  BehaviorPlanner → bridge.publish_motion_command()
       ↓
  apply_motion_command()   # target_tool_pose 等を scene_runtime に記録
       ↓
  advance()                # Python 簡易物理アニメーション
       ↓
  robot.observe_scene()    # advance() 後のスナップショットを受信
```

---

## タスクフェーズ遷移図

```
IDLE
 │  (Start)
 ▼
DETECTING         ← TomatoTargetEstimator.estimate()
 │
 ▼
TARGET_FOUND      ← MotionPlanner.plan() → HarvestMotionPlan
 │
 ▼
PLANNING          ← bridge.publish_motion_command(pre-grasp)
 │
 ▼
MOVING_TO_PREGRASP ─── (TrajectoryTrackingCoordinator が軌道追従)
 │  (到達)
 ▼
PREGRASP_REACHED  ← bridge.publish_motion_command(grasp)
 │
 ▼
MOVING_TO_GRASP
 │  (到達)
 ▼
AT_GRASP          ← グリッパー閉じコマンド発行
 │
 ▼
GRASP_EVALUATION  ← tomato_status で成否判定
 │  (HELD)
 ▼
DETACHING         ← pull コマンド発行
 │  (完了)
 ▼
DETACHED          ← place コマンド発行
 │
 ▼
MOVING_TO_PLACE
 │  (完了)
 ▼
PLACED            ← グリッパー開き → return home コマンド発行
 │
 ▼
RETURNING_HOME
 │  (robot_home=True)
 ▼
COMPLETE
```

---

## モジュール責務の対応

| 層 | モジュール | 責務 |
|---|---|---|
| エントリポイント | `scripts/run_harvest_viewer.py` | `main()` 呼び出し |
| Isaac Sim 統合 | `simulator/isaac_viewer.py` | シーン構築・メインループ・Isaac 固有クラスの構築 |
| シミュレータ状態 | `simulator/scene_runtime.py` | SceneSnapshot 管理・Python 簡易物理（テスト用）|
| アプリケーション | `app/application.py` (TomatoHarvestApplication) | システム統合・Python sim パスの advance 呼び出し |
| **オーケストレーター** | `robot/runtime.py` (HarvestRuntime) | ①〜⑤ の順次実行（制御出力の生成） |
| 行動決定 | `robot/behavior_planner/` | タスクフェーズ状態機械 |
| 運動計画 | `robot/motion_planner/` | HarvestMotionPlan 生成 |
| 軌道追従 | `robot/trajectory_tracking/` | PhaseMotionPlan に基づく軌道追従状態管理 |
| ハードウェア制御 | `robot/ros2_control/` + `simulator/isaac_*` | Franka 関節への書き込み |
