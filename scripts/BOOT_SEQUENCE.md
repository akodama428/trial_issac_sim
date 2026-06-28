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

Isaac 固有のハードウェア層と、それを注入されたアプリケーション層をまとめて構築する。

```
# Isaac 固有の実装（simulator/ に留まる）
IsaacFrankaDriver              # Franka prim への関節読み書き
  └─ robot_prim_path           # Isaac Sim の USD prim パス

IsaacRos2ControlSystem         # HardwareControlPort 実装
  └─ driver=IsaacFrankaDriver

TrajectoryTrackingCoordinator  # 軌道追従コーディネーター (ros2_control 呼び出し層)
  ├─ driver=IsaacFrankaDriver
  ├─ hardware_control_port=IsaacRos2ControlSystem
  └─ trajectory_execution_port=JointTrajectoryControllerBridge

# ロボットシステム構築 — coordinator を依存性注入
create_tomato_harvest_application(executor=franka_executor)
  ├─ IsaacSceneRuntime          # シミュレータ側ステート管理
  ├─ HarvestRuntime             # ロボットシステム全体のオーケストレーター
  │    ├─ TomatoTargetEstimator # perception
  │    ├─ build_planner()       # MotionPlanner (MoveIt スタイル)
  │    └─ BehaviorPlanner       # タスクフェーズ状態機械
  ├─ bridge                     # InMemoryBridge または Ros2Bridge
  └─ executor=franka_executor   # 注入済み TrajectoryTrackingCoordinator

ControlPanelController.boot()
  └─ TomatoHarvestApplication.boot()
       ├─ IsaacSceneRuntime.boot()  → ScenePhase.READY
       ├─ HarvestRuntime.boot()     → RobotRuntimeState.READY / HarvestTaskPhase.IDLE
       ├─ bridge.publish_scene_snapshot()  # 初期スナップショット配信
       └─ robot.observe_scene()           # ロボットが最初のシーン状態を受信
```

> **設計方針**: `IsaacFrankaDriver` と `IsaacRos2ControlSystem` は Isaac Sim 固有のため
> `simulator/` に留める。`TrajectoryTrackingCoordinator` だけを依存性注入で
> `TomatoHarvestApplication` に渡すことで、テスト環境では `executor=None` のまま動作する。

---

## フェーズ 4: メインループ（1フレームの処理順）

ユーザーが **Start** を押すと `HarvestTaskPhase` が `DETECTING` に遷移し、以下のループが回り始める。

```
┌─ 1フレーム ─────────────────────────────────────────────────────────────────┐
│                                                                             │
│  ① control_controller.step_runtime()                                       │
│       └─ TomatoHarvestApplication.step()                                   │
│            ├─ [joint sync]  executor.current_joint_state_snapshot()        │
│            │                → bridge.publish_joint_state()                 │
│            │                                                                │
│            ├─ [②]  HarvestRuntime.step()                                  │
│            │    ├─ [DETECTING]    TomatoTargetEstimator.estimate()         │
│            │    ├─ [TARGET_FOUND] MotionPlanner.plan()                     │
│            │    └─ BehaviorPlanner.step()  ← タスクフェーズ状態機械        │
│            │         └─ bridge.publish_motion_command()                    │
│            │                                                                │
│            ├─ scene_runtime.apply_motion_command()  # 論理ステート更新      │
│            ├─ scene_runtime.advance()                                      │
│            │                                                                │
│            ├─ [④]  executor.run_cycle()  ← TrajectoryTrackingCoordinator  │
│            │    ├─ PhaseMotionPlan に基づき軌道追従状態を管理              │
│            │    ├─ IsaacRos2ControlSystem.write_command()                  │
│            │    │    └─ IsaacFrankaDriver  → Franka 関節制御               │
│            │    └─ consume_replan_request() → replan_motion() (必要時)     │
│            │                                                                │
│            └─ [⑤]  executor.current_end_effector_pose()                   │
│                     → sync_robot_tool_pose()  # エンドエフェクタ → scene   │
│                                                                             │
│  ② _sync_runtime_visuals()   # デバッグ可視化更新                          │
│                                                                             │
│  ③ simulation_app.update()   # Isaac Sim 物理ステップ実行                  │
│                                                                             │
│  ④ executor.log_post_update_debug_snapshot()  # 物理後デバッグログ         │
│                                                                             │
│  ⑤ IsaacPhysicsHarvestBridge (grasp-mode=success 時のみ)                  │
│       グラスプ成否・トマト落下を判定し SceneSnapshot に反映                │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
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
| シミュレータ状態 | `simulator/scene_runtime.py` | SceneSnapshot 管理 |
| オーケストレーター | `app/application.py` (TomatoHarvestApplication) | ①〜⑤ の順次実行 |
| ロボット行動 | `robot/runtime.py` (HarvestRuntime) | perception→behavior→motion planning |
| 行動決定 | `robot/behavior_planner/` | タスクフェーズ状態機械 |
| 運動計画 | `robot/motion_planner/` | HarvestMotionPlan 生成 |
| 軌道追従 | `robot/trajectory_tracking/` | PhaseMotionPlan に基づく軌道追従状態管理 |
| ハードウェア制御 | `robot/ros2_control/` + `simulator/isaac_*` | Franka 関節への書き込み |
