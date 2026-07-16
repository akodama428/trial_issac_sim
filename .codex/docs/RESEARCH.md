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

## Step 3-6 GRASP状態機械リファクタリング（2026-07-17）

### 確認済みの事実

- Python公式の`dataclasses` failed-value semanticsでは、`frozen=True`により生成された`__setattr__`/`__delattr__`が更新を拒否する。完全な不変性ではないが、状態遷移を新しい値の返却として表す用途に適合する。
- Python公式の`StrEnum`は`Enum`であると同時に文字列として扱え、`str()`はmember値を返す。JSON契約に定義済みの実行意図を追加する用途に適合する。
- ROS 2 Jazzy公式`rclpy`ではsubscription、timer、service等のcallbackがexecutorの実行単位である。そのためcallbackはROS I/Oを受け、ROS非依存の遷移関数へ値を渡し、返値をpublish/logするshellとして構成できる。

### 設計への反映（推論）

- `PhaseMachineState`とevent/resultをfrozen dataclassで表し、`advance`が次状態を返すことで、ROS callbackとGRASP遷移ロジックを分離する。
- `MotionKind`は`StrEnum`で定義し、旧JSONの欠落fieldはdeserialize時の既定値で受理する。phase IDから実行意図を推測しない。
- node callbackはparse、pure transition呼び出し、publish/logに限定し、timer/counterの更新は状態機械だけが所有する。

### 一次情報

- Python `dataclasses`: https://docs.python.org/3/library/dataclasses.html
- Python `enum.StrEnum`: https://docs.python.org/3/library/enum.html#enum.StrEnum
- ROS 2 Jazzy `rclpy` Execution and Callbacks: https://docs.ros.org/en/jazzy/p/rclpy/api/execution_and_callbacks.html

## Step 3-5 グリッパ指令の確定と評価（2026-07-16）

### 調査目的と条件

- 目的: Step 3-4 E2EでPose Tracking通過後もgripperがopenのままになった原因を特定し、simと実機に共通する指令ライフサイクルを決める。
- 対象: Franka公式`franka_ros2` humble branch、ROS 2 `control_msgs`、Isaac Sim 6.0.1公式資料、現行repositoryとGPU E2Eログ。
- Web調査日は2026-07-16。確認済みの事実と設計推論を分離した。

### 確認済みの事実

- Franka公式`franka_gripper_node`は`move`、`grasp`、MoveIt用`gripper_action`を提供する。`grasp`はwidth、speed、forceを受け、実finger間距離がepsilon範囲内に入った場合に成功する。
- ROS 2公式`control_msgs/GripperCommand` resultはposition、effort、stalled、reached_goalを返す。指令発行と物理到達は別イベントである。
- Isaac Sim 6.0.1公式Articulation Controllerはjoint name/indexとposition commandを対応させて適用する。同じjointをpositionとeffortなど複数方式で同時制御できない。
- 現行sim経路は`servo_execution_adapter -> /tomato_harvest/gripper_closed -> IsaacSimHardwareInterface -> finger position target -> Isaac articulation`である。
- Step 3-4ログでは`hold_at_grasp`と`hold_grasp_eval`が開始してPose Trackingは成功したが、全PhysicsObsで`grip=0`、finger gap 0.0800 mだった。
- 直接原因は、Pose Tracking開始時にcloseを安全のためopenへ遅延する一方、成功時に元のclose指令を確定publishしていなかったことである。両hold commandも内部`PhaseId.MOVING_TO_GRASP`を使うため同じ遅延規則が再適用された。

### 現時点の推奨方針（設計推論）

- phase plannerが決めたgripper指令をadapterが再解釈しない。`MOVING_TO_GRASP`は上流がopen、`AT_GRASP`以降は上流がcloseを指定するため、各command開始時にその値を適用し、Pose成功時に冪等に再確定する。
- adapterは指令タイミングだけを所有し、simのfinger target変換はHardwareInterface、実機のwidth/speed/force実行と到達判定はFranka gripper action adapterへ分離する。
- 今回は既存boolean契約を最小修正し、E2Eでclose後のfinger gap減少、`gripper_closed=true`、両指接触、`HELD`を確認した。pull中もcloseは維持されたが、実接触保持が崩れて`FALLEN`となったため、次の課題はgripper指令ではなく摩擦保持とpull動作である。
- 将来の実機対応ではboolean topicをFranka/MoveIt gripper actionへ置換し、action resultの`reached_goal`と実測widthをgrasp評価へ渡す。

### 未解決の確認事項

- simのposition target方式を、実機相当のwidth/speed/force action契約へ昇格する時期。
- tomatoとの接触後にfingerが目標0 mへ到達しない正常な把持を、gap、force、stalled相当値のどの組合せで成功判定するか。
- 現行physics grasp joint生成を実接触だけで成立させられるか、geometry補助を残すか。

### 一次情報

- Franka Robotics公式`franka_gripper`: https://github.com/frankarobotics/franka_ros2/blob/humble/franka_gripper/doc/index.rst
- ROS 2 `control_msgs/GripperCommand`: https://github.com/ros-controls/control_msgs/blob/master/control_msgs/action/GripperCommand.action
- Isaac Sim 6.0.1 Articulation Controller: https://docs.isaacsim.omniverse.nvidia.com/latest/robot_simulation/articulation_controller.html

## Step 3-4 実機互換TF配信責務（2026-07-16）

### 調査目的と条件

- 目的: `panda_link0 -> panda_link8`が配信されない直接原因を特定し、sim/実機で共通化できるTF配信責務を決める。
- 対象: ROS 2 Jazzy、Isaac Sim 6.0、Franka Panda/Franka ROS 2の2026-07-16時点の公式情報、および現行repository実装。
- 確認済み事実と設計推論を分離した。

### 確認済みの事実

- ROS 2公式`robot_state_publisher`はURDFと`/joint_states`を入力に、可動jointを`/tf`、fixed jointをtransient-localの`/tf_static`へ配信する。TF計算をhardware driverやapplication nodeが個別実装する必要はない。
- ros2_control公式`joint_state_broadcaster`はhardware state interfaceから標準`/joint_states`を配信する。
- Franka公式はrobot model生成元として`franka_description`を独立packageで提供している。
- Isaac Sim 6.0公式にも`Isaac Read Joint State -> ROS2 Publish Joint State`と`Isaac Compute Transform Tree -> ROS2 Publish Transform Tree`があるが、これはsimulatorから直接TFを配信できる機能であり、実機共通の責務配置を要求するものではない。
- 現行repositoryは`scripts/run_ros2_components.sh`から`robot_state_publisher`を既に起動し、`joint_state_broadcaster`も`/joint_states`を配信している。
- 現行`franka_ros2_control.urdf`は`panda_link0`から`panda_link7`、`panda_hand`を持つが、`panda_link8`を定義していない。このため`robot_state_publisher`が正常でも`panda_link8`はTF treeに現れない。
- Step 3-3 fallbackが使うのはSceneSnapshotの関節角ではなく`robot_tool_pose`であり、58.4 mm offsetからcurrent `panda_link8` poseを導出している。

### 現時点の推奨方針（設計推論）

- TF配信の所有者は`franka_ros2_control`のbringupとし、hardware sourceだけをsim/実機で差し替える。
- `joint_state_broadcaster -> /joint_states -> robot_state_publisher -> /tf,/tf_static`を唯一のrobot kinematic TF経路にする。
- robot modelはMoveIt、ros2_control、robot_state_publisherで同じcanonical descriptionを使い、`panda_link8`とtool/hand fixed jointsを含める。
- Isaac Simからのdirect TF publishは二重authorityを避けるため既定では無効にする。SceneSnapshot fallbackは移行期間の診断・縮退経路に限定する。
- `robot_state_publisher`はcontroller plugin内部へ入れず、launch/bringupで独立nodeとして構成する。これによりhardware I/O、joint state公開、kinematic TF生成の単一責務を保つ。

### 未解決の確認事項

- 採用中のPanda modelと公式`franka_description` releaseのlink8/hand/tool frame名・fixed transformの完全一致。
- Isaac Sim USD articulation joint名とcanonical URDF joint名の対応、およびfinger mimicの扱い。
- 実機Frankaで使う`franka_ros2`/`franka_description`のROS 2 Jazzy対応版をどのrevisionで固定するか。
- base/world固定TFをrobot bringupとcell/environment bringupのどちらが所有するか。robot内部TFとは別の判断が必要。

### 一次情報

- ROS 2 robot_state_publisher: https://docs.ros.org/en/ros2_packages/rolling/api/robot_state_publisher/index.html
- ros2_control Jazzy JointStateBroadcaster: https://control.ros.org/jazzy/doc/api/classjoint__state__broadcaster_1_1JointStateBroadcaster.html
- Franka公式description: https://github.com/frankarobotics/franka_description
- Franka公式ROS 2 integration: https://github.com/frankarobotics/franka_ros2
- Isaac Sim 6.0 ROS 2 OmniGraph migration: https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/ros2_omnigraph_migration.html

## Issue #46-4 MoveIt Servo速度調整（2026-07-14）

- MoveIt Servo公式仕様では、`command_in_type: speed_units`のJointJog速度はrad/sとして扱われ、`scale.joint`はunitless入力の場合だけ適用される。このため今回の調整対象はadapterの比例gainと速度clampであり、`scale.joint`の変更ではない。
- Franka公式のPanda推奨矩形速度上限は関節ごとに1.0〜3.0 rad/sで、最小はjoint 2の1.0 rad/sである。また実機では位置と移動方向に依存する速度上限がさらに適用される。
- PoCの調整値は全関節共通0.8 rad/sとし、最小公称上限に20%の余裕を残す。比例gainは1.5から3.0へ上げるが、MoveIt ServoのURDF joint limit処理とButterworth smoothingは維持する。
- 参照: https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html
- 参照: https://frankarobotics.github.io/docs/robot_specifications.html

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

## 22. Issue #46 safety-constrained online local solver

- 調査日: 2026-07-14
- 対象: MoveIt 2 Rolling公式文書・公式実装
- 確認済みの事実:
  - MoveIt Servoはcollision、singularity、joint limitを監視し、collisionまたはsingularityへの接近時に速度をscale downする。
  - Servoの公式parameter例はsingularityにlower thresholdとhard-stop threshold、joint limit marginを持つ。
  - `AccelerationLimitedPlugin`は実行可能な範囲で加速度制限を適用する。MoveItの通常planning pathはkinematicであり、実行前にtime parameterizationが必要である。
  - MoveItのTime-Optimal Trajectory Generationは速度・加速度制限をtrajectoryへ付与し、Ruckig smoothingはjerk制限を追加できる。
- 設計への反映:
  - Issue #46ではServoの責務をproducer境界内の純粋Python solverで先行検証する。collision clearanceとJacobian由来singularity measureはadapter入力、joint position/velocity/accelerationはsolver内のhard constraintとする。
  - 現行linear solverは比較baselineとして明示選択時だけ残し、既定をsmoothstep time-scaling付きsafe online solverへ切り替える。
  - hard stop時はtrajectoryを生成せず、local publisherから下流へunsafe planを渡さない。実機安全認証を意味するものではない。
- 一次情報:
  - https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html
  - https://github.com/ros-planning/moveit2/blob/main/moveit_ros/moveit_servo/config/servo_parameters.yaml
  - https://moveit.picknik.ai/main/api/html/acceleration__filter_8hpp_source.html
  - https://moveit.picknik.ai/main/doc/examples/time_parameterization/time_parameterization_tutorial.html
  - https://moveit.picknik.ai/main/doc/concepts/trajectory_processing.html

## 23. Issue #46-2 PlanningScene / Jacobian safety observation adapter

- 調査日: 2026-07-14
- 対象バージョン: MoveIt 2 Jazzy実行環境、MoveIt 2 main公式API（2026-07-14閲覧）
- 確認済みの事実:
  - `PlanningScene::distanceToCollision(robot_state)`は、Allowed Collision Matrixを考慮したrobotとworldの最近接距離を返す。公式APIはこの関数がself-collisionを含まないことも明記している。
  - `PlanningScene::isStateColliding()`はenvironment collisionとself-collisionの双方を判定する。
  - `RobotState::getJacobian()`は指定JointModelGroupとtip linkについてJacobianを計算できる。
  - PlanningSceneMonitorはscene monitorとcurrent state monitorを開始し、monitored planning sceneとjoint statesから最新snapshotを維持できる。
- 設計への反映:
  - adapter nodeは`distanceToCollision()`をworld proximity、`isStateColliding()`を衝突時の0 m化、translational Jacobianのcondition numberをsingularity指標に用いる。
  - condition number 17を減速開始、30をhard stopとするMoveIt Servo既定値の区間を0〜1へ正規化する。
  - global MoveIt planningは従来のgeometryなしURDFを維持し、adapterだけに保守的primitive collision modelを渡す。把持の意図的接触をglobal plannerが拒否する回帰を避けるためである。
- 一次情報:
  - https://github.com/moveit/moveit2/blob/main/moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.hpp
  - https://moveit.picknik.ai/main/doc/examples/visualizing_collisions/visualizing_collisions_tutorial.html
  - https://github.com/frankarobotics/franka_description

## 19. Step 6 local planner初期導入の境界

- 調査日: 2026-07-12
- 対象バージョン: MoveIt 2 Rolling documentation（2026-07-12閲覧）
- 確認済みの事実:
  - MoveIt Hybrid Planningは、global plannerが参照軌道を生成し、local plannerが現在状態と参照軌道から逐次コマンドを生成する役割分担を採る。
  - MoveIt Servoはjoint velocity、end-effector velocity、end-effector poseを入力にでき、collision・singularity・joint limitを監視するclosed-loop補正向け機能である。
  - 本リポジトリのexecutor境界は時間付き`JointTrajectory`であり、Servo commandを直接受ける契約ではない。
- Issue #14への設計判断:
  - 初回導入ではexecutor契約を変えず、現在JointStateからglobal planの既存終端関節構成へ接続する短いjoint-space correction trajectoryをlocal producerが生成する。
  - `MOVING_TO_PREGRASP` / `MOVING_TO_GRASP` / `MOVING_TO_PLACE`を対象とし、接触支配の`DETACHING`は除外する。
  - 把持直前はlocal correction採用後に同phaseのglobal suffixを再採用しない。global plannerが返す別IK解によるgrasp trajectory差し替えを防ぐためである。
  - これはHybrid Planningの責務分離を既存契約上で先行導入する最小実装であり、Servoの速度commandや高頻度closed-loop制御そのものはStep 7候補として残す。
- 一次情報:
  - https://moveit.picknik.ai/main/doc/concepts/hybrid_planning/hybrid_planning.html
  - https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html

## 18. plan producer複線化におけるadoption / arbitrationの責務分離

- 調査日: 2026-07-11
- 対象バージョン: MoveIt 2 Rolling documentation（2026-07-11閲覧）
- 確認済みの事実:
  - MoveIt Hybrid Planning は global planner と local planner を別コンポーネントとして並走させ、local planner が global の解を参照しながら実行時補正を行うアーキテクチャである。plan の受け手側は両者の成果物を扱う必要がある。
  - Hybrid Planning の local planner はコールバックベースで global trajectory の更新を受け取り、実行中の trajectory へ blend する。すなわち「複数の計画生成主体が同じ実行系へ流れ込む」構造が前提になっている。
- Issue #13 への設計判断:
  - consumer 側の判定を2層へ分離する。producer 種別を問わない共通契約検証（metadata fail-closed / phase整合 / revision・生成時刻の順序付け）は adoption policy（Step 1 で導入済み）へ残し、producer 種別ごとの受け入れ裁定は新設の arbitration policy へ置く。consumer は arbitration だけを窓口にする。
  - local plan の裁定規則は「採用済み plan の土台があること」「planned_from_phase が現在 phase と一致すること」の2つとし、優先度制御は導入しない。producer 間の主導権交代は Step 1 の順序規則（instance 間は generated_at_sec 比較）へ委ねることで、local 採用後も新しい global plan が自然に主導権を取り戻せる。
  - 実 local planner (Step 6) 導入前に、no-op refinement の dummy producer（local_planner_stub_node）で配管だけを先に実証する。stub は採用済み global plan を土台として再刻印するのみで、軌道補正は行わない。
- 一次情報:
  - https://moveit.picknik.ai/main/doc/concepts/hybrid_planning/hybrid_planning.html

## 17. 自由空間phaseへのsuffix replan一般化とDETACHING除外

- 調査日: 2026-07-11
- 対象バージョン: MoveIt 2 Rolling documentation（2026-07-11閲覧）
- 確認済みの事実:
  - MoveIt Hybrid Planning は、低頻度で経路全体を解くglobal plannerと、高頻度でセンサ入力へ反応するlocal plannerを分離するアーキテクチャを説明している。local plannerの役割は global trajectory への追従と反応的な微修正である。
  - MoveIt Servo は、接触や終端補正のような高頻度・低遅延の補正を twist / joint jog として行う仕組みで、global replan とは別系統である。
  - OMPL のサンプリングベース計画は非決定的で、同じ goal に対して毎回異なる経路を返し得る（本リポジトリの計画書 §11 でも経路 jitter リスクとして整理済み）。
- Issue #12 への設計判断:
  - suffix replan の対象は自由空間の移動 phase（`MOVING_TO_PREGRASP` / `MOVING_TO_GRASP` / `MOVING_TO_PLACE`）に限定する。
  - `DETACHING` は茎からの引き剥がしという接触支配区間で、経路形状よりも接触力と終端の微修正が支配的なため、周期的な global suffix replan の対象にしない。OMPL 非決定性による経路差し替えは接触区間では逆効果になり得る。
  - `DETACHING` の失敗救済は従来どおり abort 起点の full-chain replan と JTC の成果ベース遷移に任せ、高頻度補正は Step 6 の local planner（Servo / Hybrid Planning）候補として残す。
  - phase ごとの残区間選択と planning scene 差（トマト把持前/後）は planner adapter 側（`plan_from_phase()` / `plan_suffix_trajectory()`）へ寄せ、node 側の phase 分岐を増やさない。
- 一次情報:
  - https://moveit.picknik.ai/main/doc/concepts/hybrid_planning/hybrid_planning.html
  - https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html

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

## 23. Step 7 は event-driven manager により global / local planner を排他的に配送する
- 調査日: 2026-07-12
- 対象: MoveIt Rolling `Hybrid Planning`、`Realtime Servo`
- 確認できた事実:
  - Hybrid Planning は、計算時間を限定しない比較的低速な global planner と、実行中に反復してセンサ入力へ応答する高速・決定的な local planner を組み合わせる。
  - Hybrid Planning Manager の planning logic は event-driven であり、planner の開始・停止・制約切替へ event を対応付ける。
  - Local Planner は global reference trajectory、現在状態、world を入力に逐次コマンドを生成し、局所解で回復できない場合に global replan を要求できる。
  - MoveIt Servo は joint jog / twist / pose command を扱い、collision、singularity、joint bound に対する減速・停止状態を備える。
- Step 7 への適用:
  - tracking error は local planner だけへ、abort は global planner だけへ配送し、同じ event から両 planner を起動しない。
  - 周期 timer による global replan は廃止し、global planner を初期計画と重大 event に限定する。
  - local event には重複排除、0.25秒の rate limit、2秒の stale timeout、phase 整合を設ける。
  - 現実装の joint-space local planner は ROS 2 message 境界と裁定を検証する段階であり、本番採用には Servo または同等 solver、collision / singularity safety の接続が必要である。
- ソース:
  - https://moveit.picknik.ai/main/doc/concepts/hybrid_planning/hybrid_planning.html
  - https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html

## 24. 初期姿勢ロバスト性には関節制限内の特異姿勢近傍を独立ケースとして含める
- 調査日: 2026-07-12
- 確認済み事項:
  - Franka公式仕様は7軸それぞれのjoint position limitを定義している。
  - MoveIt Servo公式資料はsingularity checkingを安全機能として持ち、特異点への接近時に減速、十分近い場合に停止することを説明している。
- Issue #28への適用:
  - 10姿勢は固定ID・固定関節角とし、全件をFranka関節制限内に置く。
  - `near_singularity_extended`は肩・肘・手首軸が整列する伸展特異姿勢近傍として明示し、通常ケースと区別できるflagを持たせる。
  - 特異姿勢は成功を保証する入力ではなく、planner/controllerの回復能力と失敗理由を継続計測する評価入力である。
- ソース:
  - https://frankarobotics.github.io/docs/robot_specifications.html
  - https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html

## 25. Issue #45 tracking error配信責務の簡素化
- 調査日: 2026-07-14
- 対象: ROS 2 Control Jazzy `joint_trajectory_controller`、ROS 2 Jazzy QoS
- 確認済み事項:
  - `FollowJointTrajectory` actionはexecution monitoringを必要とする用途の主要interfaceである。
  - JTCの`action_monitor_rate`既定値は20Hzであるため、action feedbackごとの瞬時誤差転送は概ね20Hz、現行250ms window配信は約4Hzとなる。
  - ROS 2のsensor data向けQoSはKeep Last / Best Effortを標準とするが、現在の`execution_status`契約はReliable depth 10であり、Issue #45ではQoS変更を混在させない。
- 設計判断:
  - executorは各feedbackの瞬時最大誤差とlimiting jointを転送し、goal全体peakはabort診断専用として維持する。
  - 短い閾値超過の保持はplanner側のpending peakで行い、local eventを受理した後にclearする。
  - threshold、minimum interval、phase guardはplanner側へ集約する。
- ソース:
  - https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
  - https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html
  - https://docs.ros.org/en/ros2_packages/jazzy/api/rclcpp/generated/classrclcpp_1_1SensorDataQoS.html

## 26. Issue #41 JTC abort後hold・新goal受理・tolerance設計
- 調査日: 2026-07-14
- 対象version: `ros-jazzy-joint-trajectory-controller 4.40.1`、`control_msgs 5.9.0`
- 確認済み事項:
  - JTCはpath/goal tolerance違反時、action feedbackへ`actual`、`desired`、`error`を設定してからabortし、現在位置のhold trajectoryへ切り替える。
  - tolerance違反と新goalの競合時は`rt_has_pending_goal_`を確認し、新goalがpendingなら古いholdで上書きしない実装である。
  - `interpolate_from_desired_state=false`では新trajectoryをstate interfaceの実測値から補間する。古い`open_loop_control`はJazzyでdeprecatedであり、互換値を残さず設定から削除する。
  - `constraints.goal_time=0`は無効化ではなく無限待ち、joint goal toleranceの`0`は未指定値を意味する。現在の5.0秒、停止速度0.05 rad/sは維持し、位置goalをplannerのtracking error閾値と同じ0.10 radにする。
  - `JointTrajectoryControllerState`はaction goal外でもreference（desired）とfeedback（actual）を提供するため、action feedbackが欠けたabort診断の補完元にできる。
- 設計判断:
  - JTCは実測state起点へ変更し、abort後の古いcommandを新goal補間seedにしない。
  - executorはgoal generationで非同期callbackを識別し、置換済みgoalの遅延resultが現行goal handleを消さないようにする。このcallbackは非同期action APIが実際に呼ぶ現役経路であり、dead codeではない。
  - abort診断はaction feedbackを正本とし、不足時だけcontroller stateでdesired/actualを補完する。
  - controller restartは新goal反映が確認できない場合の最終手段とし、JTC標準のpending-goal保護で成立する限り導入しない。
- ソース:
  - https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
  - https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html
  - https://github.com/ros-controls/ros2_controllers/blob/master/joint_trajectory_controller/src/joint_trajectory_controller.cpp
  - https://docs.ros.org/en/ros2_packages/jazzy/api/control_msgs/action/FollowJointTrajectory.html
  - https://docs.ros.org/en/jazzy/p/control_msgs/msg/JointTrajectoryControllerState.html

## 27. Issue #46-3 MoveIt Servo node比較時の接続境界
- 調査日: 2026-07-14
- 対象version: MoveIt 2 Jazzy
- 確認済み事項:
  - Servo ROS nodeは`JointJog`、`TwistStamped`、`PoseStamped`を入力し、`JointTrajectory`または`Float64MultiArray`を周期出力する。
  - Servoはcollision、singularity、joint limit、smoothingを内部で扱う。Jazzyの標準設定では100 Hz出力、singularity condition number 17から減速、30で停止する。
  - `command_out_topic`はcontroller command topicへ直接接続する設計であり、現在の`FollowJointTrajectory` actionと同時に同じJTCへ接続すると、2つのcommand producerが競合する。
  - `is_primary_planning_scene_monitor=false`により、既存`move_group`をPlanningSceneの正本として利用できる。
- 設計判断:
  - Issue #46-3ではlaunchの`servo_mode`を`off`、`jtc`から選ぶ。`jtc`ではServo出力をJTC command topicへ接続し、既存FollowJointTrajectory executorを停止してcommand producerを排他化する。実制御比較に寄与しないshadow modeは持たない。
  - E2E収穫の置換判定には、phase lifecycle、Servo commandの連続供給、完了判定を担うadapterが必要である。JTC接続だけでは非劣化を証明できないため、既存solverは削除しない。
  - 実装Gateでは既存`motion_command`のtrajectory終端を関節目標として再利用し、`ServoExecutionAdapter`が現在関節誤差から`JointJog`速度を50 Hzで連続生成する。Servoは公式ROS APIどおりJTC向け`JointTrajectory`へ変換し、collision、singularity、joint limit、smoothingを適用する。
  - CI imageは`--no-install-recommends`を使うため、`ros-jazzy-moveit-servo`を明示依存にする。
- ソース:
  - https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html
  - https://github.com/ros-planning/moveit2/blob/jazzy/moveit_ros/moveit_servo/config/servo_parameters.yaml
  - https://github.com/ros-planning/moveit2/blob/jazzy/moveit_ros/moveit_servo/config/panda_simulated_config.yaml

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
