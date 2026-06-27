# Robot Architecture

## 目的
- `src/tomato_harvest_sim/robot` 配下の責務を `perception`、`planner`、`trajectory_tracking` に分離する。
- `robot` 内で閉じる境界契約は `src/tomato_harvest_sim/robot/api` に集約する。
- `robot -> simulator` の越境 API だけを `src/tomato_harvest_sim/api` に残す。

## ディレクトリ構成
```text
robot/
  api/
    perception.py
    planner.py
    trajectory_tracking.py
  docs/
    ARCHITECTURE.md
  perception/
    __init__.py
    target_estimator.py
  planner/
    __init__.py
    pregrasp_planner.py
    moveit_service.py
    moveit_service_bridge.py
    ros_python.py
  trajectory_tracking/
    __init__.py
    execution.py
    reference_tracking.py
  geometry.py
  motion.py
  runtime.py
  moveit_config/
```

## 責務
### `robot/api`
- `robot` 内部の層間契約を定義する。
- 実装は持たない。
- `Protocol`、軽量 dataclass、公開される追従状態などだけを置く。

### `robot/perception`
- camera / tf から harvesting target を推定する。
- 現在は `TomatoTargetEstimator` が `TargetEstimate` を生成する。
- simulator transport や planner 実装には依存しない。

### `robot/planner`
- pre-grasp 系の幾何 planner と MoveIt 連携を担当する。
- `pregrasp_planner.py` は pure な pose 生成ロジック。
- `moveit_service_bridge.py` は MoveIt planning scene と trajectory 生成の adapter。
- `moveit_service.py` は `move_group` 起動管理。
- `ros_python.py` は ROS Python module 解決だけを持つ。

### `robot/trajectory_tracking`
- `reference_tracking.py` は pure に近い trajectory 参照生成と速度指令計算を担当する。
- `execution.py` は driver readback と command apply を含む stateful executor。
- planner が出した `JointTrajectory` を time-based reference として評価し、abort / replan request を管理する。

### `robot/runtime.py`
- robot software の application service。
- `perception`、`planner`、`motion`、`trajectory_tracking` を組み合わせて収穫 state machine を進める。
- simulator 固有 API は直接呼ばず、`api.bridge.BridgeProtocol` 越しにやり取りする。

### `robot/motion.py`
- `HarvestMotionPlan` を `MotionCommand` へ変換する translator。
- planner と simulator contract の間だけをつなぐ。

### `robot/geometry.py`
- perception / planner で共有する幾何ユーティリティ。

## 依存ルール
### 許可する依存
- `runtime.py -> robot/api, perception, planner, motion`
- `perception/* -> robot/api, api/contracts, geometry`
- `planner/* -> robot/api, api/contracts`
- `trajectory_tracking/* -> robot/api, api/contracts`

### 禁止したい依存
- `perception -> planner`
- `planner -> trajectory_tracking`
- `trajectory_tracking -> planner`
- `robot/* -> simulator/*`

## API の切り分け
### `src/tomato_harvest_sim/robot/api`
- robot 内部の IF。
- 例:
  - `TargetEstimator`
  - `MotionPlanner`
  - `MoveIt2PlannerBridge`
  - `FrankaExecutionDriverProtocol`

### `src/tomato_harvest_sim/api`
- robot と simulator の境界契約。
- 例:
  - `SceneSnapshot`
  - `MotionCommand`
  - `JointTrajectory`
  - `BridgeProtocol`

## 実装上の狙い
- MoveIt 連携、trajectory 追従、perception 推定が別フォルダになることで責務が読みやすくなる。
- IF を `robot/api` に出すことで、`runtime` から見た依存先が実装ではなく契約になる。
- simulator 側 adapter の変更と robot 制御ロジックの変更を切り分けやすくする。
