---
title: RESEARCH.md
version: 0.1.0
status: draft
owner: atsushi
created: 2026-06-17
updated: 2026-06-27
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

## 16. MOVING_TO_PLACE suffix replan の current state 境界

- 調査日: 2026-07-11
- 対象バージョン: MoveIt 2 Rolling documentation（2026-07-11閲覧）
- 確認済みの事実:
  - MoveIt公式の Planning Scene Monitor は、最新のplanning sceneを維持する推奨インタフェースである。
  - PlanningSceneはworld collision objectsだけでなくRobotStateも含むsnapshotである。
  - CurrentStateMonitorはJointStateを購読し、最新のRobotStateを維持する。
  - MoveIt Hybrid Planningのonline motion planningは、global solutionを反復更新しつつ、local plannerが更新trajectory segmentを既存軌道へblendする構成を説明している。
  - `move_group` はcontrollerと `FollowJointTrajectory` actionで接続するため、更新trajectoryを無条件送信するとcontroller側goal差し替えに波及する。
- Issue #11への設計判断:
  - `MOVING_TO_PLACE` suffix replanは、集約済みの最新JointStateをstart stateとして使う。
  - place以外のpregrasp/grasp/pull trajectoryは再計画しない。
  - Hybrid Planningのblend機構はまだ導入しないため、候補trajectoryの終端差分が小さい場合はpublishせず、既存goalを維持する。
  - planner多重起動はcoordinatorのin-flight gateで抑止し、ROS callbackの実行モデルだけに安全性を依存させない。
- 一次情報:
  - https://moveit.picknik.ai/main/doc/examples/planning_scene_monitor/planning_scene_monitor_tutorial.html
  - https://moveit.picknik.ai/main/doc/concepts/hybrid_planning/hybrid_planning.html
  - https://moveit.picknik.ai/main/doc/concepts/move_group.html

## 15. MoveIt の joint trajectory 実行で最も一般的なのは `FollowJointTrajectory` + `ros2_control` `joint_trajectory_controller`
- 調査日: 2026-06-26
- 確認できた事実:
  - MoveIt の公式 `Low Level Controllers` では、MoveIt は通常 `JointTrajectoryController` へマニピュレータの motion command を publish すると説明している。
  - 同ページでは、MoveIt 側で controller interface type として `FollowJointTrajectory` を設定し、別途 `ros2_control` の `JointTrajectoryController` を起動すると、MoveIt がその action interface に自動接続すると説明している。
  - 同ページでは、実運用では `99% of users choose ros2_control` と明記されている。
- 推論ではなく確認済みの解釈:
  - MoveIt 自身が独自の速度比例制御を毎周期回すのが一般形ではない。
  - 一般形は、MoveIt が時間付き `JointTrajectory` を作り、実行は low-level controller に委譲する構成である。

## 16. `joint_trajectory_controller` は waypoint 直狙いではなく、時間補間した目標状態を追従する
- 調査日: 2026-06-26
- 確認できた事実:
  - `joint_trajectory_controller` は、trajectory point 間を時間補間して実行する controller であり、waypoint 間隔は疎でもよい。
  - trajectory は「特定時刻に到達すべき waypoint 群」として扱われ、controller は機構が許す範囲でそれを実行しようとする。
  - `MoveIt` の trajectory processing では、planner は通常 path だけを出し、その後に time parameterization を行う。MoveIt 公式 docs では `TimeOptimalTrajectoryGeneration (TOTG)` が推奨され、joint の velocity / acceleration limit は `joint_limits.yaml` から読む。
- 推論ではなく確認済みの解釈:
  - 現在の executor のように「次 waypoint だけを残り時間で割って直接速度化する」方式は、MoveIt 標準系より単純化された独自実装である。
  - MoveIt の時間情報を活かすには、絶対時刻に対する参照状態 `q_ref(t), qd_ref(t)` を使う設計へ寄せる方が自然である。

## 17. velocity command を使う場合の一般形は、trajectory tracking error を PID で速度へ写像する方式である
- 調査日: 2026-06-26
- 確認できた事実:
  - `joint_trajectory_controller` は `position`、`velocity`、`acceleration`、`effort` の各 command interface をサポートする。
  - `velocity` command interface の場合、position と velocity の trajectory following error を PID loop で velocity command へ写像すると公式 docs に記載されている。
  - `velocity` command interface を使う場合、state interface には少なくとも `position` と `velocity` が必要である。
- 推論ではなく確認済みの解釈:
  - 速度指令ベースにしたい場合でも、一般的な方式は単純な `qdot = (q_target - q_now) / remaining_time` だけではなく、参照速度と追従誤差を使う PID 型の追従制御である。
  - 今回の simulator executor を改善するなら、最終的には `joint_trajectory_controller` 相当の時間基準追従へ寄せるのが最も一般的である。

## 18. `segment_timeout` を一次判定にして waypoint IK へ即フォールバックするのは、一般的な MoveIt 実行方式ではない
- 調査日: 2026-06-27
- 確認できた事実:
  - `FollowJointTrajectory` action の goal には `path_tolerance`、`goal_tolerance`、`goal_time_tolerance` があり、実測 joint 値が path tolerance を外れた場合は goal abort、最終時刻 + goal_time_tolerance までに goal tolerance へ入らない場合も goal abort である。
  - MoveIt の `Trajectory Execution Manager` には `execution_duration_monitoring`、`allowed_execution_duration_scaling`、`allowed_goal_duration_margin`、`allowed_start_tolerance` があり、低レベル controller 側の expected duration 超過や開始点不整合を監視する。
  - `joint_trajectory_controller` は waypoint を「特定時刻に到達すべき点列」として内部に保持し、時間補間しながら追従する。
- 推論ではなく確認済みの解釈:
  - 一般的な MoveIt 実行系の失敗判定は、`segment 単位のローカル timeout` より、`trajectory 全体の expected duration` と `path/goal tolerance` を基準にする。
  - したがって、現行の `segment_timeout -> waypoint IK fallback` は MoveIt 標準の失敗意味論ではなく、このリポジトリ固有の簡略化である。

## 19. 追従失敗時の改善方針としては、まず action 相当の abort と current-state replanning を行い、waypoint IK は劣化モードに下げるのが妥当である
- 調査日: 2026-06-27
- 確認できた事実:
  - MoveIt は low-level controller を直接実装するのではなく、`FollowJointTrajectory` のような controller interface を通じて既存 controller と連携する。
  - `joint_trajectory_controller` は cancel 時に即 hold だけでなく smooth deceleration を選べる設計を持つ。
  - `allowed_start_tolerance` は trajectory 先頭点と current state の整合を実行前に確認するためのパラメータである。
- 設計推論:
  - simulator 内の executor でも、segment ごとに「止まったら別方式へ逃がす」より、まず trajectory 実行を abort 扱いにして停止し、現在 joint state から同じ phase 目標へ再計画する方が MoveIt の設計意図に近い。
  - waypoint IK は、`MoveIt trajectory unavailable`、`replan failed`、`retry budget exceeded` のときだけ使う劣化モードに下げる方が、phase 間の stale trajectory 再利用や急なホーム復帰を避けやすい。
  - この結論のうち「replan を優先する」は設計推論であり、`FollowJointTrajectory` action 自体が再計画方針を規定しているわけではない。

## 20. `ros2_control` の `joint_trajectory_controller` は `position, velocity` command interface を同時に扱え、MoveIt はその action interface へ接続するのが標準構成である
- 調査日: 2026-06-27
- 確認できた事実:
  - `joint_trajectory_controller` は command interface として `position`、`position, velocity`、`position, velocity, acceleration`、`velocity` などをサポートする。
  - `position, velocity` command interface の場合、desired position はそのまま forward され、velocity 側は trajectory following の position/velocity error を使う PID で補助される。
  - MoveIt の low-level controller 構成では、`move_group` は通常 `FollowJointTrajectory` controller interface を使い、別途起動した `ros2_control` の `JointTrajectoryController` action へ接続する。
  - `joint_trajectory_controller` の action interface は execution monitoring を伴う主要経路であり、topic interface は fire-and-forget で、監視が必要なら action を優先すべきとされている。
- 推論ではなく確認済みの解釈:
  - 今回の `trajectory_tracking` が独自に持っている `q_ref(t)` 評価、追従ゲイン、timeout / tolerance 判定の多くは、`joint_trajectory_controller` と `FollowJointTrajectory` の組み合わせへ委譲できる。
  - したがって、独自 PD executor の安定化をゲイン調整で詰めるより、`ros2_control` の `position, velocity` interface を使う標準構成へ寄せる方が、意味論と責務分離の両面で妥当である。

## 21. 外乱にロバストな MoveIt2 計画には「常時フル再計画」より「グローバル計画 + ローカル追従」の分離が推奨される
- 調査日: 2026-07-08
- 対象:
  - MoveIt Documentation: Rolling
  - `Hybrid Planning`
  - `Planning Scene Monitor`
  - `Realtime Servo`
- 確認できた事実:
  - MoveIt 公式の `Hybrid Planning` では、従来の `Sense-Plan-Act` は既知の静的環境には有効だが、不安定または動的な環境には適用しづらいと説明している。
  - 同ページでは、その対策として `global planner` と `local planner` を並列かつ反復的に動かす `Hybrid Planning` を提示している。
  - 同ページでは、global planner は比較的遅く完備性寄り、local planner は連続実行され、現在状態・世界状態・参照軌道を見ながらロボットコマンドを逐次生成すると説明している。
  - 同ページでは、local planner が近接衝突などの局所問題を検出した場合、event-based logic により global planner を再起動して replan できると説明している。
  - `Planning Scene Monitor` では、最新の planning scene を維持する推奨インタフェースは `PlanningSceneMonitor` であり、`CurrentStateMonitor` は `JointState` と TF を subscribe して内部 `RobotState` を更新すると説明している。
  - `Realtime Servo` では、MoveIt Servo はリアルタイム制御向けであり、joint velocity / end-effector velocity / end-effector pose を入力として受け、visual servoing や closed-loop position control に使えると説明している。
  - `Realtime Servo` では、collision checking、singularity checking、motion smoothing、joint position / velocity limits enforcement を備えると説明している。
- 推論ではなく確認済みの解釈:
  - MoveIt2 の一次情報だけを見ると、「低周期で現在状態から再計画する」要求は妥当である。ただし、推奨アーキテクチャは「実行中に毎回フルOMPL再計画して controller goal を置換し続ける」より、「グローバル計画」と「ローカル追従 / ローカル補正」を分離する形である。
  - したがって、このリポジトリの改善も、まずは `現在状態を起点にした低周期の suffix replan` を導入しつつ、将来的には `Hybrid Planning` または `MoveIt Servo` を使った local planner へ寄せる方が、MoveIt2 の公式整理と整合する。
- 現時点の推奨方針:
  1. 近短期は、現行 `trajectory_planner_node` に低周期 timer を導入し、`aborted` 時だけでなく、移動フェーズ中は最新 `joint_states` と `scene_snapshot` から再計画する。
  2. ただし再計画対象は「全フェーズ固定」ではなく、現在フェーズ以降の `suffix` に限定する。特に `MOVING_TO_PLACE` では既存の `plan_place_trajectory()` を使う。
  3. `DETACHING` のような接触を伴う区間は、低周期OMPL再計画の主対象にせず、停止 / 保持 / 退避といった局所挙動、または将来の `Servo` / local planner 適用候補として扱う。
  4. 中長期は、`Hybrid Planning` の `global planner + local planner` へ移行し、global planner は疎に replan、local planner は高頻度に追従補正する構成を検討する。
- 未解決の確認事項:
  - 現行の `motion_command_executor_node` の cancel-and-replace 実装で、低周期再計画をどこまで許容できるか。
  - `DETACHING` 中に必要なのはグローバル経路再探索なのか、それとも contact-aware な局所補正なのか。
  - `MoveIt Servo` をこの構成へ入れる場合、`joint_trajectory_controller` とどう役割分担するか。
- ソース:
  - https://moveit.picknik.ai/main/doc/concepts/hybrid_planning/hybrid_planning.html
  - https://moveit.picknik.ai/main/doc/examples/hybrid_planning/hybrid_planning_tutorial.html
  - https://moveit.picknik.ai/main/doc/examples/planning_scene_monitor/planning_scene_monitor_tutorial.html
  - https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html

## 22. GitHub Actions でローカルマシンを実行基盤にする場合は self-hosted runner を使い、`runs-on` でラベル指定する
- 調査日: 2026-07-09
- 対象:
  - GitHub Docs `Self-hosted runners`
  - GitHub Docs `Choosing the runner for a job`
  - GitHub Docs `Workflow syntax`
- 確認できた事実:
  - GitHub 公式では、GitHub Actions の job をローカルマシンやオンプレ環境で実行するには `self-hosted runner` を使う。
  - self-hosted runner は、ハードウェア、OS、インストール済みソフトウェアを自分で管理する前提である。
  - `jobs.<job_id>.runs-on` にはラベル配列を指定でき、self-hosted runner を対象にする場合は `runs-on: [self-hosted, linux, x64, gpu]` のように複数ラベルを並べられる。
  - 配列指定の場合、job は指定したすべてのラベルに一致する runner 上でのみ実行される。
- 推論ではなく確認済みの解釈:
  - Isaac Sim の GPU 実行、Docker、NVIDIA Container Toolkit を前提にしたこのリポジトリの CI は、GitHub-hosted runner ではなく self-hosted runner 前提で構成するのが妥当である。
  - したがって workflow では `self-hosted`, `linux`, `x64`, `gpu` のラベルを要求し、runner 側の前提条件を固定した方が運用しやすい。
- ソース:
  - https://docs.github.com/en/actions/concepts/runners/self-hosted-runners
  - https://docs.github.com/en/actions/how-tos/write-workflows/choose-where-workflows-run/choose-the-runner-for-a-job
  - https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-syntax

# ソース
- NVIDIA Isaac Sim Container Installation
  - https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html
- NVIDIA Isaac Sim ROS 2 Installation (Default)
  - https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html
- NVIDIA Isaac Sim MoveIt 2
  - https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_moveit.html
- MoveIt Documentation: Low Level Controllers
  - https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html
- MoveIt Documentation: Time Parameterization
  - https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html
- MoveIt Documentation: Trajectory Processing
  - https://moveit.picknik.ai/main/doc/concepts/trajectory_processing.html
- ROS 2 Control Documentation: joint_trajectory_controller
  - https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
- ROS 2 Control Documentation: Trajectory Representation
  - https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html
- control_msgs `FollowJointTrajectory.action`
  - https://raw.githubusercontent.com/ros-controls/control_msgs/master/control_msgs/action/FollowJointTrajectory.action
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
