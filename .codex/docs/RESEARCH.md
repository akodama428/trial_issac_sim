---
title: RESEARCH.md
version: 0.1.0
status: draft
owner: atsushi
created: 2026-06-17
updated: 2026-06-18
---

# 調査目的
以下の構成で、トマトをロボットハンドで収穫するシミュレータを構築できるかを、公式一次情報を中心に整理する。

```text
Custom Docker Container
  ├─ Isaac Sim
  │  ├─ Franka Panda
  │  ├─ eye-to-hand Camera
  │  ├─ Tomato / Leaf / Branch scene
  │  └─ ROS2 Bridge
  └─ ROS2 Jazzy
     ├─ image topic
     ├─ joint_state
     ├─ tf
     └─ MoveIt2 / rule-based node
```

# 固定した前提
- Camera は `eye-to-hand`
- 収穫動作は `把持して引く`
- トマトの detach は `物理で扱う`
- ROS は `Docker コンテナ上の Jazzy`
- ROS 2 Jazzy は `自前 Dockerfile` で導入する
- Isaac Sim と ROS 2 Jazzy は `1コンテナ化` する
- detach の物理モデルは `固定 joint の break` とする
- 外部 asset は `fruit / stem / branch が別階層` を必須条件とする

# 調査条件
- 調査日: 2026-06-18
- 優先ソース:
  - NVIDIA Isaac Sim 公式ドキュメント
  - NVIDIA NGC / Isaac Sim 公式 GitHub
  - Isaac Lab 公式ドキュメント
  - Franka Robotics 公式ドキュメント
  - ROS / OSRF 公式 Docker 情報
  - 外部アセット提供元の現行ページ
- 今回の重点観点:
  - Isaac Sim を Docker コンテナで動かせるか
  - 公開されている公式コンテナイメージがあるか
  - ROS 2 Jazzy を使う前提が妥当か
  - Franka Panda、Camera、ROS 2 topic、MoveIt 2 がこの方針に乗るか
  - Tomato / Leaf / Branch の外部 asset に適切な候補があるか

# 確認できた事実
## 1. Isaac Sim には公開されている公式 Docker コンテナがある
- Isaac Sim の `Container Installation` では、NVIDIA Isaac Sim container の利用が公式手順として案内されている。
- 公式の pull コマンドは `docker pull nvcr.io/nvidia/isaac-sim:6.0.0` である。
- ドキュメント上、コンテナ実行は Linux Docker 前提で、deployment on remote headless servers or the Cloud に向くとされている。
- WebRTC 用の Docker Compose でも、`ISAAC_SIM_IMAGE=nvcr.io/nvidia/isaac-sim:6.0.0 docker compose ...` で prebuilt NGC image を使える。
- したがって、Isaac Sim 用の「公開されている公式コンテナ」は存在すると判断してよい。
  - 根拠は NVIDIA 公式ドキュメントが NGC 上の prebuilt image を直接案内している点。

## 2. Docker 版 Isaac Sim には明確な運用制約がある
- 公式ドキュメントでは、Isaac Sim container は headless mode with livestreaming を主目的として説明されている。
- コンテナ版は Python apps と standalone examples を headless mode only で実行する前提と明記されている。
- `--network=host` は WebRTC livestreaming に必須で、通常の Docker bridge + `-p` では映像配信が成立しないと説明されている。
- コンテナは rootless user で動く。
- 一部チュートリアルは Nucleus 未接続だと Content Browser を前提にうまく動かないことがある。
- よって、このリポジトリでは GUI 操作中心ではなく、headless 実行と standalone script 中心の構成に寄せる方が整合的である。

## 3. Docker まわりの公式補助資産もある
- Isaac Sim 公式 GitHub の `isaac-sim/IsaacSim` には `tools/docker/` があり、Dockerfile、docker-compose.yml、README、build scripts が公開されている。
- 公式 docs からも `Isaac Sim Dockerfiles` へのリンクがある。
- そのため、NGC の prebuilt image をそのまま使う選択肢と、公式 Dockerfiles をベースに custom image を作る選択肢の両方がある。

## 4. ROS 2 Jazzy は Isaac Sim 6.0 の公式推奨構成である
- `ROS 2 Installation (Default)` では、Isaac Sim の ROS 2 bridge は Jazzy と Humble に対応している。
- Ubuntu 24.04 では Jazzy が recommended と明記されている。
- `MoveIt 2` チュートリアルでも、Humble で planning / execution failure が出る場合は Jazzy を検討するよう書かれている。
- したがって、今回の方針を `Dockerコンテナ上でJazzy` に切り替えるのは、公式推奨とも整合する。

## 5. ROS 2 側にも公式系 Docker 資産がある
- OSRF の `docker_images` リポジトリは、Official Library と OSRF Organization の Docker images を管理している。
- README では、Official Library の ROS images は production and general downstream use 向けで、distribution name でタグ付けされると説明されている。
- これは Jazzy を含む正式リリース系 ROS 2 をコンテナ化する方針と整合する。
- 一方で、Isaac Sim と ROS 2 Jazzy を 1 つにまとめた NVIDIA 公式の combined image は、今回確認した公式 docs では見つからなかった。
  - これは「見つからなかった」のであって「存在しない」とは断定しない。

## 6. 公式調査だけを見ると「2コンテナ構成」は自然である
- 公式に確認できたのは `nvcr.io/nvidia/isaac-sim:6.0.0` という Isaac Sim 専用コンテナである。
- ROS 2 Jazzy は別コンテナとして用意し、同一ホスト上で同じ DDS 到達性を持たせる構成が自然である。
- この結論は、Isaac Sim 側 docs が「ROS 2 を source した同じ terminal から Isaac Sim を起動する」ことを要求している点と、コンテナ側 docs が host networking を前提にしている点からの設計推論である。
- 実装方針としては、少なくとも次の 2 案がある。
  - `Isaac Sim official container` + `ROS 2 Jazzy container` を `--network=host` で並列起動する
  - 公式 Isaac Sim image をベースに ROS 2 Jazzy を追加した custom image を作る

## 7. ただし今回の採用方針は「自前 Dockerfile による 1 コンテナ化」である
- その一方で、今回のユーザー指定は `ROS 2 Jazzy は自前 Dockerfile`、`Isaac Sim と ROS 2 Jazzy は 1 コンテナ化` である。
- このため、このリポジトリでは `nvcr.io/nvidia/isaac-sim:6.0.0` をベースに Jazzy と必要な ROS 2 パッケージを追加する custom image を前提に進める。
- この構成では、Isaac Sim 起動環境と ROS 2 実行環境の source 順を 1 イメージ内で固定できる。
- また、PoC 段階では 1 本のコンテナ起動手順で再現できるため、利用者視点の試行には扱いやすい。
- 代償として、依存衝突や image build 失敗時の切り分けは 2 コンテナより難しくなる。
- この判断は、公式の safest default というより、今回の開発運用方針として確定したプロジェクト判断である。

## 8. Franka Panda はそのまま Docker 方針に乗る
- `Robot Assets` には `FrankaPanda` が掲載されており、USD Path は `FrankaRobotics/FrankaPanda/franka.usd` である。
- Panda は 9 DOF として整理されている。
- 付属アクセサリとして `AlternateFinger`、`Default`、`Robotiq_2F_85` がある。
- したがって、収穫 PoC は Franka Panda 標準グリッパで始め、必要に応じてハンドを差し替える進め方が取れる。

## 9. Camera と ROS 2 topic publish も Docker 方針に乗る
- `ROS 2 Cameras` チュートリアルでは `ROS 2 Camera Helper` を使って、publish するデータ種別と topic 名を設定できる。
- サンプルでは `type=rgb`、`topicName=rgb`、`frameId=turtle` を設定している。
- 同ページでは RGB 以外に `Depth` と `Point Cloud` も publish 可能である。
- `ROS 2 Bridge in Standalone Workflow` には `camera_periodic.py` があり、image と camera info publisher の実行周期を制御できる。
- したがって、Docker 版 Isaac Sim で standalone script を回す方針は、今回のカメラ配信要件と相性が良い。

## 10. `/joint_states` と `/tf` も Docker 方針に乗る
- `ROS2 Joint Control` チュートリアルでは `ROS2 Publish Joint State` が `/joint_states` に publish し、`ROS2 Subscribe Joint State` が `/joint_command` を subscribe する。
- 例では `/panda` articulation を targetPrim に設定している。
- `ROS2 Transform Trees and Odometry` では、Isaac Sim 6.0 以降は `Isaac Compute Transform Tree` から `ROS 2 Publish Transform Tree` へつなぐ流れになっている。
- したがって、Franka Panda を含む scene から `/joint_states` と `/tf` を外部 ROS 2 Jazzy コンテナへ流す構成は成立する。

## 11. MoveIt 2 は Jazzy 方針の方が明確に相性が良い
- `MoveIt 2` チュートリアルには `ROS2 > MoveIt > Franka MoveIt` のサンプル環境がある。
- 起動コマンドは `ros2 launch isaac_moveit isaac_moveit.launch.py` である。
- `isaac_moveit` と moveit config は Isaac Sim の `humble_ws` または `jazzy_ws` に含まれている。
- Humble は既知の intermittent failure がある一方、Jazzy は推奨側に置かれているため、MoveIt 2 を早期に使いたいなら Jazzy 優先が妥当である。

## 12. eye-to-hand + 把持して引く + 物理detach は asset 要件を厳しくする
- ここから先は、ユーザーが固定した前提にもとづく設計推論である。
- `eye-to-hand` にすると、camera の最小撮影距離は hand-mounted より緩くなるため、固定 RGB-D camera を採用しやすい。
- 一方で `把持して引く` と `物理detach` を成立させるには、少なくとも次の asset 条件が必要になる。
  - fruit と stem / peduncle / branch が別パーツ、または後から分離しやすい topology であること
  - fruit 側と stem 側に collision を入れやすいこと
  - detach 部に fixed joint あるいは breakable connection を author しやすいこと
  - pivot / 原点が peduncle 近傍に置き直せること
- つまり、単に見た目が良い植物 asset よりも、「部位分割しやすい asset」の方が今回の要件には適している。

## 12.5. camera の初期配置は greenhouse row 斜め上方からの固定視点とする
- ここもユーザー指定を受けた実装前提の具体化である。
- 初期 PoC では、camera はトマト房を正面から少し見下ろす `eye-to-hand` 固定 RGB-D camera とする。
- 初期の推奨配置は、収穫対象トマト基準で次のように置く。
  - 位置: `x=0.80 m, y=0.00 m, z=1.35 m`
  - 姿勢: `pitch=-30 deg, yaw=180 deg, roll=0 deg`
- 意図は次の通りである。
  - Franka の前方作業空間とトマト房の両方を 1 視野に収めやすい
  - 手先接近時の self-occlusion を hand-mounted camera より減らせる
  - peduncle 近傍の上下関係を depth で取りやすい
- この配置は greenhouse row に対して正対する 1 本目の仮置きであり、PoC では FOV、死角、遮蔽率を見て微調整する。

## 13. 外部 asset 候補はあるが、そのまま物理detachに使える保証は弱い
- NVIDIA の `Third-Party USD Assets` では、Isaac Sim 互換の外部アセット源として `Lightwheel SimReady store`、`Synthesis Asset pack`、`imagine.io` などが案内されている。
- ただし、公式ページ上では tomato plant 専用 asset の有無までは確認できなかった。
- 現在の marketplace 上では、TurboSquid の tomato 検索で `Tomato Plant with Fruits and Flowers 3ds Max`、`Tomatoes Fruits and Flowers 3ds Max`、tomato plant 検索で `Tomato Plant Blender` が見つかる。
- greenhouse 側も TurboSquid で `Multi-Span Greenhouse With Soil Beds 3ds Max`、`Industrial Agricultural Greenhouse Frame 3ds Max`、`Vegetables Greenhouse Tent with Gardener 3ds Max` のような候補が見つかる。
- TurboSquid の検索ページでは exchange formats として `FBX`、`glTF`、`OBJ`、`USD` フィルタが提示され、また real-time models の対象に `Omniverse` が含まれている。
- ただし、今回確認できたのは検索結果ページまでで、各商品の mesh hierarchy や「果実が別メッシュか」は確認できていない。
- 今回のプロジェクトでは、`fruit / stem / branch が別階層` であることを asset 採用の必須条件にする。
- したがって、これらは「候補」ではあるが、商品詳細で階層分離を確認できるまでは採用確定にしない。

## 14. 物理detach前提なら、scan ベースが最も要件に合いやすい
- `Third-Party USD Assets` では `XGrid Scan to Simulation Tutorial` が案内されており、3D scan を Isaac Sim scene に変換する経路がある。
- 研究側でも、トマト植物の 3D point cloud を open-source で扱う例がある。
  - `Look how they have grown...` では、50+ の tomato plant 3D point cloud files を含む open-sourced datasets があるとされている。
- ただし point cloud dataset は、そのまま物理シミュレーション用 mesh にはならない。
- それでも、peduncle 位置や枝葉の密度、葉の重なり方を再現したいなら、marketplace asset より scan / point cloud を元に USD を作り直す方が、最終的には要件適合性が高い可能性がある。

# 外部 asset の評価
## 候補 A: Marketplace tomato plant を導入する
- 候補例:
  - TurboSquid `Tomato Plant Blender`
  - TurboSquid `Tomato Plant with Fruits and Flowers 3ds Max`
  - TurboSquid `Tomatoes Fruits and Flowers 3ds Max`
- 長所:
  - 立ち上がりが速い
  - 見た目をすぐ作れる
  - greenhouse asset も同じ系統で揃えやすい
- 短所:
  - fruit / stem / branch の分離保証が弱い
  - 物理detach用に DCC での分割、pivot 修正、collision authoring が必要な可能性が高い
- 判断:
  - 最初の画作り用には適切
  - 物理detachまで見据えると、そのまま本命 asset にするのはリスクがある

## 候補 B: greenhouse shell だけ marketplace から入れる
- 候補例:
  - TurboSquid `Multi-Span Greenhouse With Soil Beds`
  - TurboSquid `Industrial Agricultural Greenhouse Frame`
  - TurboSquid `Vegetables Greenhouse Tent with Gardener`
- 長所:
  - 環境構築が速い
  - 植物本体と切り分けて購入・差し替えできる
- 短所:
  - 植物 asset は別途必要
- 判断:
  - greenhouse 背景は marketplace、植物は別調達、という分離は合理的

## 候補 C: scan / reconstruction ベースで植物を作る
- 候補例:
  - XGrid scan-to-simulation workflow
  - open tomato point-cloud datasets を参照して自前再構築
- 長所:
  - peduncle / branch / fruit の分割を要件に合わせて設計できる
  - `把持して引く` と `物理detach` の条件に最も合わせやすい
- 短所:
  - 最初の工数が高い
  - mesh cleanup と physics authoring が必要
- 判断:
  - 長期的には最も適切
  - まず marketplace で PoC、次に scan ベースへ移る二段構えが現実的

# 確定した実装方針
1. Docker は `nvcr.io/nvidia/isaac-sim:6.0.0` をベースにした `自前 Dockerfile` とし、Isaac Sim と ROS 2 Jazzy を `1コンテナ化` する。
2. Camera は `eye-to-hand` で固定し、初期配置は `x=0.80 m, y=0.00 m, z=1.35 m, pitch=-30 deg, yaw=180 deg, roll=0 deg` を採用する。
3. 収穫動作は `把持して引く` 前提とし、peduncle / stem の扱いを scene と要件に含める。
4. detach の物理モデルは `固定 joint の break` を採用する。
5. 植物 asset は `fruit / stem / branch が別階層` を必須条件とし、これを満たさない market asset は採用しない。
6. greenhouse 環境は marketplace から先に入れてよいが、植物本体は階層分離条件を満たすものだけに限定する。
7. 階層分離を満たす市販 asset が見つからない場合は、scan / custom rebuild に移行する。

# 推奨する最小 PoC 構成
## Phase 1
- Custom container
  - Base image: `nvcr.io/nvidia/isaac-sim:6.0.0`
  - Added by custom Dockerfile:
    - ROS 2 Jazzy
    - MoveIt 2
    - image subscriber / monitor tools
    - `/joint_states` monitor
    - `/tf` monitor
    - rule-based node
  - Runtime contents:
    - Franka Panda
    - eye-to-hand static RGB-D camera
    - 単純化した枝 + トマト 1 個の scene
    - ROS 2 Bridge
    - 色または既知座標で対象トマトを選ぶ最小ロジック
    - predefined pose sequence で接近し、把持後に引き方向へ移動する制御
    - fixed joint break の detach 条件観察

## Phase 2
- greenhouse shell asset を導入する
- market asset の植物を 1 本導入し、`fruit / stem / branch が別階層` かを点検する
- fruit / peduncle / branch の階層に対して collision と fixed joint break 条件を author する
- 必要なら scan ベースへ移行する
- `isaac_moveit` を使って MoveIt 2 を接続する

# 残課題
- `自前 Dockerfile` に Jazzy と MoveIt 2 をどのレイヤ順で追加するか
- greenhouse row の寸法を決めた後に、camera の FOV と遮蔽率をどう再調整するか
- fixed joint break の閾値を、トマトサイズと把持力に対してどの値から試すか
- market asset の商品詳細を確認し、`fruit / stem / branch が別階層` を満たす候補を具体的に確定できるか

# ソース
- NVIDIA Isaac Sim Container Installation
  - https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html
- NVIDIA Isaac Sim ROS 2 Installation (Default)
  - https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html
- NVIDIA Isaac Sim MoveIt 2
  - https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_moveit.html
- NVIDIA Isaac Sim ROS 2 Cameras
  - https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_camera.html
- NVIDIA Isaac Sim ROS2 Joint Control: Extension Python Scripting
  - https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_manipulation.html
- NVIDIA Isaac Sim ROS2 Transform Trees and Odometry
  - https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_tf.html
- NVIDIA Isaac Sim Robot Assets
  - https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_robots.html
- NVIDIA Isaac Sim Third-Party USD Assets
  - https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_third_party.html
- NVIDIA Isaac Sim Camera and Depth Sensors
  - https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_camera_depth_sensors.html
- isaac-sim/IsaacSim `tools/docker`
  - https://github.com/isaac-sim/IsaacSim/tree/main/tools/docker
- OSRF Docker Images
  - https://github.com/osrf/docker_images
- TurboSquid tomato plant search
  - https://www.turbosquid.com/Search/3D-Models/tomato-plant
- TurboSquid tomato search
  - https://www.turbosquid.com/Search/3D-Models/tomato
- TurboSquid greenhouse search
  - https://www.turbosquid.com/Search/3D-Models/greenhouse
- Tomato plant 3D point-cloud dataset reference
  - https://arxiv.org/abs/2304.03610
