---
title: USERS_GUIDE.md
version: 0.1.0
status: draft
owner: atsushi
created: 2026-06-18
updated: 2026-06-20
---

# 目的
Isaac Sim を知らない利用者でも、トマト収穫シミュレータを起動し、3DView 上でロボットの探索、把持、搬送、配置までを確認できるようにするための利用者ガイドである。

このシミュレータでは Web ブラウザは使わない。利用者は Isaac Sim の 3DView と、Isaac Sim 内に表示される操作ウィンドウだけを使う。

# 想定する利用者
- Isaac Sim を触ったことがない
- ROS 2 や MoveIt2 の詳細は知らない
- まずはトマト収穫の流れを 1 回動かして理解したい

# このシミュレータでできること
- Franka Panda とトマトを 3DView で確認する
- `fixed` 視点と `hand` 視点を切り替える
- `Start` を押して探索から収穫完了までのシナリオを実行する
- `Stop` で途中停止する
- `Reset` で初期状態へ戻して再試行する
- ターミナルに出る `Target is Found!` とトマト座標を確認する

# 利用前に知っておくこと
## 最小用語
- `3DView`
  - ロボット、トマト、配置先を見る画面である
- `fixed camera`
  - ロボット全体を見る固定外部視点である
- `hand camera`
  - `panda_hand` に取り付けた手先視点カメラである
- `Start / Stop / Reset`
  - シナリオ開始、途中停止、初期化である

## シナリオの前提
- 実行環境は `Isaac Sim + ROS 2 Jazzy` の 1 コンテナである
- 知覚に使うカメラは `hand camera` だけである
- トマトの高さは既知である
- 収穫動作は `探索 → IK 接近 → 把持 → 搬送 → 配置` である
- 今の PoC では detach は本格物理ではなく、把持後にトマトを手先へ追従させる最小実装で検証する

# 起動方法
## 通常の起動
推奨コマンドは次の 2 つである。

```bash
./build.sh
./run.sh
```

`./run.sh` は、`DISPLAY` が有効な X11 または VNC デスクトップ上で実行することを前提にしている。起動すると Isaac Sim のネイティブウィンドウが開き、その中に 3DView と `Tomato Harvest Controls` ウィンドウが表示される。

Linux の X11 環境では、必要に応じて先に次を実行する。

```bash
xhost +local:root
```

## デバッグ用の起動
コンテナに入って手動で試す場合は次を使う。

```bash
./into.sh
```

コンテナ内に入ったあと、シミュレータ本体は次で起動する。

```bash
/isaac-sim/python.sh scripts/run_poc.py --mode isaac
```

headless で完走確認だけしたい場合は次を使う。

```bash
/isaac-sim/python.sh scripts/run_poc.py --mode isaac --headless --test
```

# 起動すると何が見えるか
起動が成功すると、利用者は次の 3 つを見る。

- Isaac Sim の 3DView
  - Franka、トマト、枝、配置先マーカーが見える
- `Tomato Harvest Controls` ウィンドウ
  - `Start`
  - `Stop`
  - `Reset`
  - `Fixed Camera`
  - `Hand Camera`
- ターミナルログ
  - 現在フェーズ
  - `Target is Found!`
  - トマトの camera 座標
  - トマトの world 座標

# 最初の体験シナリオ
利用者が最初に確認する流れは次の 6 段階である。

1. `./run.sh` でシミュレータを起動する
2. 3DView に Franka とトマトが見えることを確認する
3. `Fixed Camera` と `Hand Camera` を押して視点が切り替わることを確認する
4. `Start` を押す
5. 探索、把持、搬送、配置が順に進むことを確認する
6. `Reset` を押してもう一度試す

# 操作方法
## 1. 起動する
利用者は `./run.sh` を実行する。

期待する結果:
- Isaac Sim ウィンドウが開く
- 3DView が表示される
- `Tomato Harvest Controls` が表示される
- 状態表示が `Ready` になる

## 2. 3DView を確認する
最初は `fixed` 視点で全体を確認する。

期待する結果:
- ロボットの全体配置が分かる
- 黄色いトマトが見える
- 緑の枝と青い配置先マーカーが見える

必要に応じて、マウスで orbit、pan、zoom を行う。

## 3. 視点を切り替える
`Fixed Camera` または `Hand Camera` を押す。

期待する結果:
- `Fixed Camera`
  - ロボット全景の固定外部視点になる
- `Hand Camera`
  - ロボット手先の `panda_hand/HandCamera` 視点になる

使い分け:
- 全体挙動を見たいときは `Fixed Camera`
- 探索と認識の見え方を確認したいときは `Hand Camera`

## 4. Start を押す
`Start` を押すと、1 回分の収穫シナリオが始まる。

シナリオは次の順で進む。

1. 探索
2. 収穫
3. 搬送と配置

### 4-1. 探索
- ロボットは hand camera だけを使う
- ロボット周囲を 360 度スキャンするように複数姿勢を順に取る
- トマトの高さは既知として扱う
- hand camera 内でトマトが見つかると探索終了になる

期待する結果:
- 3DView 上でロボット姿勢が段階的に変わる
- ターミナルに次が出る

```text
Target is Found!
Tomato camera xyz: ...
Tomato world xyz: ...
```

### 4-2. 収穫
- 発見した world 座標をもとに IK を解く
- 手先を pre-grasp 位置へ移動する
- 続いて grasp 位置へ移動する
- グリッパを閉じる

期待する結果:
- 状態表示が `Approaching`、`Grasping` と進む
- ロボット手先がトマトへ寄る
- グリッパが閉じる

### 4-3. 搬送と配置
- 把持後、トマトを所定の配置先へ搬送する
- 配置位置に到達したらグリッパを開く
- シナリオ完了になる

期待する結果:
- 状態表示が `Placing`、`Complete` へ進む
- トマトが青い配置先マーカー付近へ移る

## 5. Stop を押す
途中で止めたい場合は `Stop` を押す。

期待する結果:
- 現在のモーションが停止する
- 状態表示が `Stopped` になる
- そのままでは自動再開しない

再開したい場合:
- `Reset` を押してから、もう一度 `Start` を押す

## 6. Reset を押す
再試行したい場合は `Reset` を押す。

期待する結果:
- ロボットが初期姿勢に戻る
- トマトが元の位置へ戻る
- 視点が `fixed` に戻る
- 状態表示が `Ready` になる

# 利用者が確認すべき成功条件
- `Start` を押すと探索が始まる
- トマト発見時に `Target is Found!` が出る
- camera 座標と world 座標が出る
- IK により手先がトマトへ移動する
- 把持後にトマトが配置先へ移動する
- `Reset` で同じ手順を繰り返せる

# 初心者向けの注意
- 起動直後は shader warm-up のため重いことがある
- `Hand Camera` は手先視点なので、全景確認には向かない
- 視点が見づらい場合は `Fixed Camera` に戻す
- GUI が開かない場合は、`DISPLAY` と X11/VNC セッションを確認する

# 今の PoC でまだ簡略化している点
- トマト検出は画像認識ではなく、stage 上の既知ターゲットを hand camera 座標へ変換して可視判定している
- 収穫計画は MoveIt2 本実装ではなく、Isaac Sim 内 IK と段階モーションで構成している
- detach は本格物理 break ではなく、PoC の操作体験確認を優先した簡略実装である
