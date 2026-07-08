# Tomato Harvest Simulator

Isaac Sim 6.0 と ROS 2 Jazzy を 1 コンテナにまとめ、Franka Panda でトマト収穫シナリオを検証するリポジトリです。

現行の正規起動入口は `scripts/run_ros2_components.sh --isaac --moveit` です。`scripts/run_harvest_viewer.py` はこのスクリプトから Isaac Sim viewer を起動する内部ヘルパーであり、利用者向けの単独起動入口としては扱いません。旧 PoC 実装は削除済みで、現行実装の起動には使いません。

## 設計文書（GitHub Pages）

`docs/` 配下の設計文書（HTML）は GitHub Pages で公開しており、ブラウザから直接閲覧できます。`main` ブランチへ push すると自動で最新化されます。

- 文書一覧（目次）: <https://akodama428.github.io/trial_issac_sim/>
- 解説書: <https://akodama428.github.io/trial_issac_sim/summary.html>
- テスト要件一覧: <https://akodama428.github.io/trial_issac_sim/test_list.html>
- 計画: 物理忠実度レベルアップ: <https://akodama428.github.io/trial_issac_sim/planning_levelup_tomato_sim.html>
- 計画: MoveIt2 再計画改善: <https://akodama428.github.io/trial_issac_sim/planning_movit2_improvements.html>

新しい設計文書を `docs/` に追加した場合は、`docs/index.html` の目次にもリンクを追記してください。

## 前提

- Linux
- NVIDIA GPU と NVIDIA Container Toolkit
- Docker
- X11 で GUI 表示する場合は `xhost +local:root`
- Isaac Sim ベースイメージ `nvcr.io/nvidia/isaac-sim:6.0.0` を pull できること

## ファイル

- `docker/Dockerfile`
  - Isaac Sim 公式イメージに ROS 2 Jazzy と MoveIt 2 を追加する本番用 Dockerfile
- `build.sh`
  - 現行実装用 Docker イメージをビルドする
- `into.sh`
  - 現行実装用のデバッグコンテナを作成し、そのまま `docker exec -it` で入る
- `python.sh`
  - ROS 2 Jazzy を source したうえで `/isaac-sim/python.sh` を起動する

## ビルド

```bash
./build.sh
```

環境変数で上書きできます。

```bash
IMAGE_NAME=my-tomato-sim ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:6.0.0 ./build.sh
```

## GitHub Actions CI

このリポジトリの CI は GitHub-hosted runner ではなく、GPU と Docker を持つ `self-hosted` runner を前提とする。workflow は [`.github/workflows/ci.yml`](/mnt/ssd2/trial_issac_sim/.github/workflows/ci.yml:1) にあり、`runs-on: [self-hosted, linux, x64, gpu]` を要求する。

- unit テスト: Python `pytest` と `franka_ros2_control` の `colcon test`
- E2E テスト: Isaac Sim を実際に headless 起動し、`scripts/run_ros2_components.sh --isaac --moveit --headless --auto-start` を実行
- NGC login が必要な場合は repository secret `NGC_API_KEY` を設定する
- commit ごとにフル app image を作り直さず、依存入りの `ci-base` image をローカル Docker cache で再利用する。checkout 済みソースは container に bind mount して、その上で build/test を実行する

## デバッグコンテナに入る

GUI を使う場合は先に X11 を許可します。

```bash
xhost +local:root
./into.sh
```

`into.sh` は次を行います。

- コンテナ名 `tomato-harvest-sim-debug` を作成または再利用する
- リポジトリルート全体を `/workspace/tomato-harvest` にマウントする
- ホストの `/tmp/tomato-harvest-sim-tmp` をコンテナ内 `/tmp` にマウントする
- `PYTHONPATH=/workspace/tomato-harvest/src` を設定する

## コンテナ内で実行する代表コマンド

### 1. フル ROS2 構成で起動する

```bash
./scripts/run_ros2_components.sh --isaac --moveit
```

期待する結果:

- Isaac Sim の 3DView が開く
- `franka_ros2_control`、MoveIt2、robot ノード群、SimulatorNode がまとめて起動する
- `Tomato Harvest Controls` パネルが表示される
- `Start / Stop / Reset` と `Fixed Camera / Hand Camera` を操作できる

### 2. 自動開始で一連のシナリオを確認する

```bash
./scripts/run_ros2_components.sh --isaac --moveit --auto-start
```

### 3. headless で E2E 実行する

```bash
./scripts/run_ros2_components.sh --isaac --moveit --headless --headless-steps 600 --auto-start
```

## よくある間違い

- `./python.sh` が無いと言われる
  - 現行の `./into.sh` で起動したコンテナではない可能性があります
- `scripts/run_harvest_viewer.py` を単独で正規起動だと思って実行する
  - 現行構成では `run_ros2_components.sh` が正規入口です。viewer 単独では ros2_control / MoveIt / robot ノード群が揃いません
- GUI が真っ暗になる
  - `DISPLAY` と X11 ソケットが正しく渡っているかを確認してください
  - `xhost +local:root` 実行漏れを確認してください

## 現行実装でよく使う起動例

GUI あり:

```bash
./into.sh
cd /workspace/tomato-harvest
./scripts/run_ros2_components.sh --isaac --moveit
```

自動開始:

```bash
./into.sh
cd /workspace/tomato-harvest
./scripts/run_ros2_components.sh --isaac --moveit --auto-start
```

headless:

```bash
./into.sh
cd /workspace/tomato-harvest
./scripts/run_ros2_components.sh --isaac --moveit --headless --headless-steps 600 --auto-start
```
