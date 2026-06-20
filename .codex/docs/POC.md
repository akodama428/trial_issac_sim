---
title: POC.md
version: 0.1.0
status: draft
owner: atsushi
created: 2026-06-18
updated: 2026-06-20
---

# PoC の目的
この PoC は、ブラウザ UI を作り込むことではなく、Isaac Sim の 3DView だけでトマト収穫シナリオを初学者が理解しながら試せるかを検証するためのものである。

検証対象の利用者体験は次の通りである。

1. 1 コマンドで Isaac Sim が起動する
2. 3DView と最小操作ウィンドウだけで使い方が分かる
3. `Start` を押すと探索から配置までの一連動作が進む
4. `Stop` と `Reset` で迷わず再試行できる
5. `fixed` 視点と `hand` 視点を切り替えながら状態を理解できる

# 現在の PoC 方針
## 利用者操作
- 画面は `Isaac Sim 3DView + Tomato Harvest Controls` のみとする
- ブラウザは使わない
- 操作は `Start / Stop / Reset / Fixed Camera / Hand Camera` の 5 つに絞る

## シナリオ
- `Start` を押すと 1 回分の収穫シナリオを実行する
- 知覚カメラは `hand camera` のみとする
- 探索時はロボットが周囲を 360 度走査する
- トマトの高さは既知とする
- 発見後は IK で接近し、把持し、所定位置へ搬送して配置する

## 今回の最小実装
- scene は `Franka + branch + tomato + place target` の最小構成
- 検出は hand camera 座標への投影と閾値判定で行う
- 収穫は deterministic な段階シナリオで行う
- 搬送後の配置完了までを PoC の完了条件とする

# 5 スプリント計画
## Sprint 1: 起動基盤
- 目的:
  - 1 コンテナで Isaac Sim と ROS 2 Jazzy を起動できるようにする
- 実装対象:
  - `docker/Dockerfile`
  - `build.sh`
  - `run.sh`
  - `into.sh`
  - container entrypoint
- 完了条件:
  - `./build.sh` が通る
  - `./run.sh` で GUI 起動経路に入れる
  - `./into.sh` でコンテナ内デバッグに入れる
- 現状:
  - 実装済み

## Sprint 2: 3DView 利用性
- 目的:
  - 3DView が表示され、利用者が scene を操作できるようにする
- 実装対象:
  - Franka の実体表示
  - hand camera の prim 階層化
  - fixed camera と hand camera の切替
  - ライト、床、トマト、枝、配置先の可視化
- 完了条件:
  - 3DView が表示される
  - マウスで視点操作できる
  - `Fixed Camera` と `Hand Camera` の切替が分かる
- 現状:
  - 実装済み

## Sprint 3: 探索から発見まで
- 目的:
  - `Start` 押下で探索が始まり、トマト発見でログが出るようにする
- 実装対象:
  - `Start` ボタン
  - 360 度探索用の scan pose 群
  - hand camera 座標による可視判定
  - `Target is Found!` と座標ログ出力
- 完了条件:
  - scan pose を順に実行する
  - トマト発見時に terminal に camera/world 座標が出る
- 現状:
  - 実装済み

## Sprint 4: 収穫と搬送
- 目的:
  - 発見後に IK で接近し、把持し、配置先へ運べるようにする
- 実装対象:
  - pre-grasp と grasp の 2 段 IK
  - グリッパ開閉
  - 把持後のトマト追従
  - place target への搬送
- 完了条件:
  - `Approaching`、`Grasping`、`Placing`、`Complete` が進む
  - トマトが配置先へ移る
- 現状:
  - 実装済み

## Sprint 5: 再試行性
- 目的:
  - 初心者が停止、初期化、再試行を理解できるようにする
- 実装対象:
  - `Stop`
  - `Reset`
  - 初期視点への復帰
  - 端末ログの明確化
- 完了条件:
  - 途中停止できる
  - `Reset` で初期姿勢とトマト位置に戻る
  - 同じ手順を繰り返せる
- 現状:
  - 実装済み

# 実装順序
1. 起動経路を安定させる
2. 3DView を操作可能にする
3. hand camera を本当に `panda_hand` 配下へ置く
4. Start/Stop/Reset の最小 UI を Isaac Sim 内に置く
5. 探索シナリオを入れる
6. IK 接近と把持を入れる
7. 配置先への搬送を入れる
8. Reset と再試行を確認する

# 実装・デバッグ記録
## 3DView 表示の解決策
- 学び:
  - `Isaac Sim Python 6.0.0` のウィンドウが出ても、すぐに「応答なし」と見えることがあった
  - 主因は初回 shader warm-up の重さで、即クラッシュではなかった
  - `3DView が出る` だけでなく、`マウス操作を受け付ける` ことを別条件として確認する必要があった
- 採用した対策:
  - X11 を有効にしたコンテナで GUI 起動する
  - `./into.sh` でデバッグコンテナへ入り、必要時に `/isaac-sim/python.sh ...` を直接実行できるようにした
  - hang detector を長めにし、初回 warm-up 完了まで待つ前提にした
- PoC への反映:
  - Sprint 2 の完了条件に `視点操作できること` を含める

## hand camera の学び
- 学び:
  - 手先近傍に camera を置いただけでは `hand camera` と呼べない
  - `panda_hand` 配下に camera prim を持たせ、手先と一緒に動く必要がある
  - `hand camera` が stage 上に存在しても、手先メッシュに近すぎると viewport では真っ黒に見えることがある
  - 特に local offset が短すぎる場合、camera が手先ジオメトリや near clipping 条件に埋もれて、利用者からは「視点切替できていない」ように見えた
- 採用した対策:
  - `HandCamera` を `panda_hand` の子として生成する
  - `Fixed Camera` と `Hand Camera` を明示的に切り替える
  - hand camera の local offset を `0.10 m` まで前に出し、手先メッシュの内側から外す
  - hand camera に `clippingRange = (0.01, 1000.0)` を明示設定して、近接時でも黒画面になりにくくした
- PoC への反映:
  - 利用者ガイドでは `全景確認` と `認識確認` の役割を分けて説明する
  - camera 要件には「prim 階層」だけでなく「local offset / clipping が実際に可視化できる値であること」を含める
  - 今後 camera を増やす場合も、`見える camera` として成立する最小距離と clipping 条件をテスト観点に入れる

## Web UI をやめた学び
- 学び:
  - PoC 段階では、ブラウザ連携よりも 3DView だけで操作が閉じる方が理解しやすかった
  - 画面が分かれると、初心者は `どちらを見ればよいか` に迷いやすかった
- 採用した対策:
  - Web UI を削除し、Isaac Sim 内の最小 control window へ集約した
- PoC への反映:
  - `USERS_GUIDE.md` を 3DView 中心の体験へ更新した

## 探索シナリオの学び
- 学び:
  - hand camera だけを使う場合、最初から target を画面中央へ置く前提は成立しない
  - そのため、まず scan pose を定義し、可視判定で見つかった時点から接近へ移る構成が自然だった
- 採用した対策:
  - 周囲探索用の複数 pose を定義した
  - トマト高さ既知の条件を visibility 判定へ入れた
- PoC への反映:
  - REQUIREMENTS へ進む際は `探索フェーズ` を独立した要求として扱う

## トレー配置の学び
- 学び:
  - `place target` に到達しただけでは、利用者が期待する `トレーに置いた` 状態にはならない
  - 把持中のトマトを最後に固定座標へ上書きすると、見た目と実際のロボット挙動が一致しなくなる
  - 配置は `トレー上方へ移動`、`トレー面まで下降`、`グリッパ開放`、`手先退避` の 4 段階で扱う方が理解しやすかった
- 採用した対策:
  - place 動作を 1 段の IK ではなく、上方進入と下降の 2 段 IK に分けた
  - トマト開放時は、その瞬間の実座標を保持し、後から別座標へテレポートしないようにした
  - 開放後はロボット手先を少し上へ退避させ、配置完了後の不安定挙動を避けた
- PoC への反映:
  - `搬送完了` の条件は、単なる `place target 到達` ではなく、`実際に置いて手先が離脱できること` として扱う
  - REQUIREMENTS へ進む際は、配置動作を `進入 / 下降 / 開放 / 退避` の段階要求に分けて整理する

# 現在の実行コマンド
## 通常起動
```bash
./build.sh
xhost +local:root
./run.sh
```

## デバッグコンテナ
```bash
./into.sh
```

## コンテナ内から GUI 起動
```bash
/isaac-sim/python.sh scripts/run_poc.py --mode isaac
```

## headless 完走確認
```bash
/isaac-sim/python.sh scripts/run_poc.py --mode isaac --headless --test
```

## 物理把持 PoC の学び
- 学び:
  - `fruit を dynamic rigid body`、`tray を fixed collider`、`hand-fruit を fixed joint` とする最小物理把持構成までは PoC として成立した
  - 一方で、`fruit-stem を PhysX fixed joint の break で素直に表現する` 方式は、現状の簡易 scene と URDF import ベースの Franka 構成では安定しなかった
  - 特に `FruitStemJoint` は Isaac Sim 6.0 headless 実行時に `disjoint body transforms` warning を出しやすく、探索開始前の fruit 落下や、Reset 後に元位置へ戻らない原因になった
  - そのため PoC では、`収穫前は fruit を kinematic hold で枝位置に保持し、把持成立後だけ dynamic に戻して hand-fruit joint へ引き渡す` 方式に切り替えた
  - この方式により、`Reset 時だけ fruit を初期位置へ戻す`、`Start では fruit を勝手に戻さない`、`把持前に片指が当たっただけで fruit が落ちない` という利用者体験は安定した
  - ただし、その代償として `把持前の fruit-stem 接続` は厳密な breakable joint 物理ではなく、`PoC 用の強い保持モデル` になっている
  - また、ロボットの最終的な grasp 成功率は `運動計画の精度` に強く依存し、PoC の手書き IK と固定オフセットだけで最後まで詰めるのは効率が悪いことが分かった
- PoC への反映:
  - 本番設計では、fruit / stem / branch を asset 側で分離し、`fruit-stem joint anchor を asset ローカル座標で厳密定義した breakable joint` に置き換える必要がある
  - PoC の `kinematic hold` は、利用者体験確認のための暫定実装として扱い、本番の物理モデルそのものとは見なさない
  - REQUIREMENTS / ADR へ進む際は、`PoC では物理保持を段階導入し、fruit-stem detach の完全物理再現は未完` であることを前提にする
  - ロボットの接近姿勢、grasp center、collision 回避、再現性ある把持は、PoC の手書き IK で深追いせず、MoveIt2 と grasp pose 設計のフェーズへ分離して進める

# 観察項目
- `Ready` まで到達する時間
- `Start` 押下後に探索が始まるか
- `Target is Found!` が出るか
- hand camera と fixed camera の切替が理解しやすいか
- トマト搬送後に `Complete` まで進むか
- `Reset` 後に同じ操作を繰り返せるか

# REQUIREMENTS へ反映する事項
- 固定する事項:
  - Web UI は使わず、3DView 中心の操作にする
  - 操作は `Start / Stop / Reset / camera switch` に絞る
  - 探索は hand camera のみで行う
  - トマト発見時に camera/world 座標を出す
- 今後詳細化する事項:
  - 検出アルゴリズムの本実装
  - detach の物理 break モデル
  - place 位置の本番運用条件
  - ROS 2 / MoveIt2 連携の本格化
