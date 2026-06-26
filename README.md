# Tomato Harvest Simulator

Isaac Sim 6.0 と ROS 2 Jazzy を 1 コンテナにまとめ、Franka Panda でトマト収穫シナリオを検証するリポジトリです。

現行の正規 viewer 起動入口は `scripts/run_harvest_viewer.py` です。`poc_code/` は旧 PoC の退避先であり、現行実装の起動には使いません。

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

### 1. 3D Viewer を起動する

```bash
PYTHONPATH=src ./python.sh scripts/run_harvest_viewer.py --transport ros2
```

期待する結果:

- Isaac Sim の 3DView が開く
- `Tomato Harvest Controls` パネルが表示される
- `Start / Stop / Reset` と `Fixed Camera / Hand Camera` を操作できる

### 2. 自動開始で一連のシナリオを確認する

```bash
PYTHONPATH=src ./python.sh scripts/run_harvest_viewer.py --auto-start --timeout-seconds 90 --transport ros2
```

### 3. MoveIt サービスだけを単独起動する

通常は viewer 側が自動起動しますが、切り分け時は単独起動もできます。

```bash
PYTHONPATH=src ./python.sh scripts/run_moveit_service.py
```

## よくある間違い

- `poc_code/into.sh` に入ってから現行の `scripts/run_harvest_viewer.py` を実行する
  - 旧 PoC フォルダしかマウントされないため失敗します
- `./python.sh` が無いと言われる
  - `poc_code` コンテナに入っている可能性があります
- GUI が真っ暗になる
  - `DISPLAY` と X11 ソケットが正しく渡っているかを確認してください
  - `xhost +local:root` 実行漏れを確認してください

## 現行実装でよく使う 2 ターミナル運用

ターミナル 1:

```bash
./into.sh
PYTHONPATH=src ./python.sh scripts/run_harvest_viewer.py --transport ros2
```

ターミナル 2:

```bash
docker exec -it tomato-harvest-sim-debug bash
cd /workspace/tomato-harvest
PYTHONPATH=src ./python.sh scripts/run_moveit_service.py
```

ただし通常はターミナル 1 の viewer 起動だけで MoveIt サービスも自動起動します。
