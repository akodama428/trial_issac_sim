# Step 3-12 grasp系phaseのdirect JTC化 A/B実験 — Servo完全撤去可否の検証 (Issue #59)

**ステータス**: 実験完了 / B条件 4/10 (40%) — **Servo撤去の既定化は見送り**
**作成日**: 2026-07-18
**対象issue**: [#59](https://github.com/akodama428/trial_issac_sim/issues/59)
**前提レポート**: `step3-11_direct_jtc_trajectory_execution.md` §6.3

## 0. 実験目的

step3-11でDETACHING以降の搬送・退避系をdirect JTC化した結果、Servo経路はgrasp系
3 phase（MOVING_TO_GRASP / AT_GRASP / GRASP_EVALUATION）のみに残った。静止物体把持の
現仕様ではTF直接追従（pose tracking）が不要である可能性があり、grasp系もdirect JTC化して
Servoを実行経路から完全に撤去できるかをA/B実験で検証する。

- **A条件**: 現状（grasp系はServo pose tracking）。基準データは step3-11 の10ケースmatrix
  実測 10/10（2026-07-17、同一コードベース直前run、n=1）
- **B条件**: grasp系3 phaseも `terminal_pose_tracking=False` としてdirect JTC実行

## 1. 実装

環境変数 `TOMATO_HARVEST_GRASP_DIRECT_JTC=1` で切り替えるA/B実験フラグを実装した。

- `motion_command.py`: `GRASP_SERVO_PHASES` を定義し、フラグ有効時に該当phaseの
  `PhaseCommandSpec.terminal_pose_tracking` を `False` へ置換（pure関数、env読み取りは
  ノード初期化時の境界に限定）。起動時に `grasp_direct_jtc_flag` メトリクスを出力
- `scripts/ci/run_e2e.sh`: docker への env pass-through 追加
- テスト4件追加（デフォルト不変 / B条件でgrasp系のみFalse / 他phase不変 / env解釈）。
  全体 324 passed

`terminal_pose_tracking=False` により `pose_tracking_goal` が None となり、既存の
`should_execute_direct_trajectory()` 判定でdirect JTC経路（bridge retime + 一度きり
dispatch + 実測監視）に乗る。**フラグOFF時の挙動は完全に不変**。

## 2. 実行条件の注記: step予算900→1200

最初のrun（`CI_HEADLESS_STEPS=900`、A条件と同一予算）では default / elbow_left の2ケースが
「摩擦把持・搬送成功 → releasing の settle 判定中に step 予算切れ」で失敗した
（角速度1.0〜2.4 rad/sが減衰中、閾値0.5まであと1〜2秒）。settle完了自体の可否を観測する
ため、予算を1200へ拡張して全10ケースを再実行した。900予算runの証跡は
`.artifacts/issue59-grasp-direct-b-steps900/` に保全。

## 3. 結果: B条件 4/10 (40%)

成果物: `.artifacts/issue59-grasp-direct-b/`（summary: `initial-pose-summary.md`）

| Case | 結果 | grasp精度 [mm] | settle | failure mode分類 |
|---|---|---:|---|---|
| default | FAIL | 6.4 | 1.84s (seq1092) | 設置成功→home帰還で実行乖離 |
| elbow_left | FAIL | (900run: 9.3) | - | grasp chainのmissing_trajectoryループ (abort×4) |
| elbow_right | FAIL | - | - | 起動ストール（phase進行なし、B非起因の疑い） |
| shoulder_high | PASS | 15.3 | 0.18s (seq827) | - |
| shoulder_low | FAIL | - | - | grasp失敗→home stallループ (triggered_stall×31) |
| wrist_left | FAIL | - | - | grasp chainのmissing_trajectoryループ (abort×2) |
| wrist_right | FAIL | 14.4 | 2.12s (seq1054) | 設置成功→home帰還未完 |
| folded_near | PASS | 8.5 | 1.12s (seq940) | - |
| extended_far | PASS | 8.0 | 0.71s (seq892) | - |
| near_singularity_extended | PASS | 15.1 | 1.13s (seq932) | - |

A条件（step3-11）との比較:

| 指標 | A (grasp系Servo) | B (全phase direct) |
|---|---|---|
| 成功率 | **10/10 (100%)** | 4/10 (40%) |
| grasp精度 (position_error_norm) | pose tracking収束（6D許容内） | 6.4〜15.3mm（計画+JTC実行のopen-loop精度） |
| grasp到達ケースでの摩擦把持成立 | 10/10 | **6/6 (100%)** |
| E2E時間（PASSケース） | 88〜134s | 82〜117s（同等〜やや高速） |
| 予算公平性 | 900 steps | 1200 steps（900では4 PASSのうち2のみ相当） |

## 4. failure mode分析

### 4.1 grasp chainのmissing_trajectoryループ（2件: elbow_left, wrist_left）

moving_to_grasp のdirect実行がabortすると、replanが `success=true` だが対象区間の軌道を
含まないplanを返し、`missing_trajectory` abort→replan→…のループに入る。
**issue #58（DETACHING縮退replanのレジリエンス欠陥）と同一機序がgrasp chainでも再現**した
初の実証データとなる。A条件ではgrasp系のabortをpose trackingの誤差フィードバックが
吸収していたため顕在化しなかった。

### 4.2 設置成功後のhome帰還乖離（2件: default, wrist_right）

摩擦把持→搬送→`settled_in_tray` まで**完全に成功**した後、returning_home のdirect実行中に
実測がreference から+0.52 rad/sの一定レートで乖離し（誤差0.9rad超）、予算内に完了しない。
default では home dispatch 後1.5秒は正常追従（誤差≤0.017rad）した後に乖離が始まり、
finger_z が place 高さへ降下した。接触力はゼロで **issue #52のwedgeとは別機序**。
placed hold（停止軌道dispatch）とhome dispatchが0.32秒差で連続する境界での軌道置換の
競合が疑われるが、本実験では原因未特定（要追加調査）。

#### 4.2.1 追加取得bagの健全性確認（2026-07-18）

`.artifacts/issue59-home-divergence/e2e/home_divergence_bag` を解析した結果、このbagには
§4.2のhome帰還乖離区間が含まれていなかった。

- 記録時間は8.4486秒、846 messageで、保存されたtopicは
  `/joint_trajectory_controller/controller_state` だけだった。
- recorderは`/tomato_harvest/phase`と
  `/joint_trajectory_controller/joint_trajectory`にもsubscribeしたが、両topicの
  message数は0だった。そのためphase境界、placed hold、home dispatchの順序は復元できない。
- 7軸のJTC referenceとfeedbackは全期間で静止していた。最大絶対誤差はjoint2の
  0.002373 radで、0.02 radを超えるsampleは0件だった。§4.2で観測した0.9 rad超の乖離や
  +0.52 rad/sの一定レート移動は再現していない。
- bag時刻（1784325543.77〜1784325552.22）と同じrunの`robot_node.log`ではphaseは
  `moving_to_grasp`だった。`franka_controller.log`では直前の
  `moving_to_pregrasp`成功後に`missing_trajectory`でabortしており、設置成功後の
  `returning_home`へ到達していない。
- `/tomato_harvest/moveit_servo/status`は同名topicに複数typeが存在するためrecorderが
  errorを出しているが、これは上記3topicのうちcontroller stateしか保存されなかった
  直接原因とは断定できない。

したがって、このbagから「placed holdとhome dispatchの軌道置換競合」を支持または反証する
ことはできない。ファイル名に反してhome divergenceの証跡ではなく、
`moving_to_grasp`の`missing_trajectory`終了runの静止区間である。次回はhome乖離が発生した
runを同定したうえで、少なくともphase、trajectory dispatch、controller stateを各1件以上
含むことをmetadataで確認してから解析する必要がある。

#### 4.2.2 recorder同居での再E2E評価（2026-07-18）

sidecar containerからの記録はtopic型だけを発見してmessageを受信できなかったため、
recorderをE2Eと同一container内でrobot stackより先に起動した。非同期jobへSIGINTを送る
方式ではbashがsignalをignoreしてflushできなかったため、SIGTERMで正常終了させた。
最終成果物は
`.artifacts/issue59-home-divergence/rerun-20260718-final/e2e/home_divergence_bag`。

実行条件:

- initial pose: `default`
- `TOMATO_HARVEST_GRASP_DIRECT_JTC=1`
- `CI_HEADLESS_STEPS=1600`
- grasp mode: `physics`
- 結果: `complete`、1209/1600 step

bagの健全性:

- duration 54.4300秒、全5465 message
- controller state 5444件
- trajectory dispatch 9件
- phase 12件（`detecting`から`complete`まで）
- recorderは終了時にcacheをflushし、metadataを自動生成した

placed/home境界では、`hold_placed`の2点停止軌道dispatchから0.3191秒後に、33点・
3.363秒のhome軌道がdispatchされた。home phaseは3.728秒で完了し、最終追従誤差は
0.005052 radだった。一方、home開始2.15秒後（home dispatchから約1.85秒後）に大きな
過渡乖離が再現した。

| joint | home中の最大絶対誤差 |
| --- | ---: |
| joint5 | 2.0373 rad |
| joint7 | 0.8671 rad |
| joint6 | 0.1336 rad |
| joint1 | 0.0934 rad |

JTC referenceは全軸で連続しており、home区間の最大差分速度は0.595 rad/sだった。
これに対してfeedbackの差分速度はjoint5で57.5 rad/s、joint7で30.6 rad/sに達した。
joint5 referenceが約-0.04 radのまま滑らかに進む間に、feedbackだけが約2.00 radまで
急増してから収束している。従って、少なくとも今回のrunではtrajectory置換によって
JTC referenceが不連続になったのではなく、hardware feedback側で急激な関節運動が
発生した。

この結果から、placed holdとhome dispatchの0.32秒間隔だけを直接原因とする仮説は
支持されない。乖離開始まで約1.85秒の遅延があり、referenceも正常だったためである。
主な次の調査対象はIsaac articulation/hardware interface側の物理的不安定化であり、
同時刻のarticulation実速度、applied target、effort、contact、physics step時間を
同じ時系列で記録する必要がある。今回のrunはfeedback急変後に回復して`complete`したため、
§4.2の「予算内に完了しない」は確率的な回復時間またはstep予算との組合せと考えられる。

#### 4.2.3 moving_to_placeのFranka drive属性・速度上限・推定トルク調査（2026-07-18）

手動GUI E2Eで取得した
`.artifacts/manual-gui/home_divergence_bag_20260718_010824`を用い、moving_to_place中の
JTC目標速度、実関節速度、Franka USDのdrive属性を照合した。bagは51.9207秒、5862 message
（controller state 5193、trajectory 657、phase 12）を含み、moving_to_placeは3.1962秒、
320 sampleだった。

##### 実行時Franka USDのdrive属性

Isaac Sim 6.0公式
`Assets/Isaac/6.0/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd`を実際にロードし、
各arm joint primの属性を直接取得した。

| joint | `maxJointVelocity` | rad/s換算 | stiffness | damping | maxForce | drive type |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| joint1〜4 | 124.618 deg/s | 2.175 | 400 | 80 | 87 Nm | force |
| joint5〜7 | 149.542 deg/s | 2.610 | 400 | 80 | 12 Nm | force |

この速度上限は`franka_ros2_control/config/joint_limits.yaml`のMoveIt上限と一致する。
現行`IsaacFrankaDriver`は同じ`ArticulationAction`へ`joint_positions`と
`joint_velocities`を同時設定している。Isaac Sim公式資料が示す標準PDの概念式は次である。

```text
effort =
    stiffness * (target_position - actual_position)
  + damping   * (target_velocity - actual_velocity)
```

従ってvelocity targetは独立した速度上限制御ではなく、driveのD項の目標でもある。

##### moving_to_placeの目標速度と実速度

| joint | JTC目標速度最大 [rad/s] | USD上限比 | 実速度最大 [rad/s] |
| --- | ---: | ---: | ---: |
| joint1 | 0.195 | 9.0% | 0.790 |
| joint2 | 0.533 | 24.5% | 0.962 |
| joint3 | 0.340 | 15.6% | 1.119 |
| joint4 | 0.521 | 23.9% | 1.039 |
| joint5 | 0.102 | 3.9% | 0.243 |
| joint6 | 0.276 | 10.6% | 0.586 |
| joint7 | 0.522 | 20.0% | 1.608 |

全関節のJTC目標速度はUSD上限の25%未満であり、MoveIt/JTC速度上限がFranka USDに対して
大きすぎる仮説は支持されない。実速度が目標速度を上回る関節もあり、速度上限飽和による
単純な追従遅れとは逆の特徴である。可視化は
`.artifacts/manual-gui-analysis/moving_to_place_velocity_limits.png`に保存した。

##### PD式からのトルク需要推定

USDのstiffness/dampingとrosbagのreference/feedbackを上記PD式へ代入した。
これはcontact constraintを含むPhysX solverの実effort readbackではなく、drive属性と
観測誤差から求めた非飽和需要の推定値である。

| joint | 非飽和トルク需要最大 [Nm] | maxForce [Nm] | 推定飽和sample率 |
| --- | ---: | ---: | ---: |
| joint1 | 139.7 | 87 | 30.6% |
| joint2 | 35.1 | 87 | 0% |
| joint3 | 23.7 | 87 | 0% |
| joint4 | 39.1 | 87 | 0% |
| joint5 | 6.9 | 12 | 0% |
| joint6 | 18.2 | 12 | 2.5% |
| joint7 | 33.2 | 12 | 45.6% |

joint1は位置誤差0.282 radだけでもP項が約113 Nmとなり、87 NmのmaxForceを超える。
joint7も最大12 Nmに対して推定需要33.2 Nmである。従って、速度上限よりも、接触・拘束等で
位置誤差が増えた後にdriveがmaxForceへ飽和し、追従回復余裕を失う機序が有力な増幅要因で
ある。ただしトルク飽和だけでは、負方向指令中にjoint1が正方向へ動き始める外力源を
説明できない。逆方向運動の起点としてtray/gripper接触等を別途観測する必要がある。
可視化は`.artifacts/manual-gui-analysis/moving_to_place_inferred_drive_effort.png`に保存した。

##### 判定と次の比較条件

- **速度上限過大仮説**: 棄却。MoveIt上限とUSD上限は一致し、実目標は上限の25%未満。
- **driveトルク不足仮説**: joint1/joint7の回復を妨げる増幅要因として有力。ただし逆方向運動の
  発生原因そのものとは未確定。
- **stiffness/damping仮説**: `Kp=400`により大誤差時は直ちにmaxForceへ達するため要比較。
- **position/velocity同時指定仮説**: Isaac標準PD契約には適合するが、position-onlyとのA/Bで
  D項の寄与を分離する価値がある。

原因確定の比較優先順は、同一軌道に対して
`tray collision有無`、`position+velocity対position-only`、`velocity scaling 0.2対0.1`
とする。各runで`trayF`、実articulation effort、applied target、実位置・実速度、
maxForce到達時間を同一時系列に記録する。速度scalingだけを先に下げると、接触起因でも
見かけ上誤差が減り、根本原因を取り違える可能性がある。

一次情報:

- Isaac Sim 6.0 Joint Gains Tuning:
  https://docs.isaacsim.omniverse.nvidia.com/6.0.0/openusd_tuning_tutorials/tutorial_06_joint_gains_tuning.html
- Isaac Sim Articulation Controller API:
  https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/deprecated/isaacsim.core.api/docs/index.html
- Isaac Sim Motion Policy PD target semantics:
  https://docs.isaacsim.omniverse.nvidia.com/4.2.0/concepts/motion_generation/motion_policy.html
- Franka Robotics joint limits:
  https://frankarobotics.github.io/docs/control_parameters.html

### 4.3 settle予算の余裕不足（900予算runの2件で観測）

リリース時のトマト残留スピンは確率的（0.001〜2.4 rad/s）で、大きい場合はsettleに
2秒前後を要し、900 step予算の残量では間に合わない。A条件はgrasp系phaseの滞留が
数秒長く、settleまでの減衰時間の余裕が結果的に大きかった。B条件はサイクルが速い分、
settle完了がstep予算の限界に近づく。

### 4.4 起動ストール（1件: elbow_right、B非起因の疑い）

trajectory_planner が全期間 `phase: unknown / suppressed_incomplete_state` のまま
サイクルが開始しなかった。Issue #40系の起動flakeの様相であり、B条件のロジックとは
独立の可能性が高い。

## 5. 判断: Servo撤去の既定化は見送り

issue #59の判断基準「BがAと同等以上の成功率・把持精度ならServo撤去を既定化」に対し、
**B (40%) < A (100%)** のため見送り。ただし内訳は仮説を部分的に支持する:

- **grasp のdirect化そのものは機能する**: graspに到達した6ケース全てで摩擦把持が成立し
  （6.4〜15.3mm）、detach・搬送まで完走した。「静止物体把持ならTF直接追従は不要」という
  仮説自体は反証されていない
- 失敗の主因はgrasp直接化の把持性能ではなく、(1) Servoの誤差フィードバックが暗黙に
  吸収していた**replan縮退へのレジリエンス欠如（#58）**、(2) **home帰還のdirect実行乖離**
  （新観察・要調査）、(3) settle予算の余裕不足、に分解される

## 6. 次ステップ

1. **issue #58 の修正を先行させる**: 本実験で#58の発生頻度・影響がB条件下で大きいことが
   実証された。#58修正後にB条件を再評価する価値がある
2. **home帰還乖離の調査**（§4.2）: 設置成功後の失敗であり、A条件でも潜在しうる。
   placed hold と home dispatch の連続境界の観測を追加する
3. settle予算: リリース時スピンの上振れを考慮した予算またはリリース前の短い静定待ちを検討
4. フラグ `TOMATO_HARVEST_GRASP_DIRECT_JTC` は**デフォルトOFFのまま維持**し、
   上記対応後の再実験に備える（OFF時の挙動は不変、テストで担保済み）
5. effort command interface化とmaxForceを考慮したトルク制御の検証は、本実験から分離して
   [Issue #61](https://github.com/akodama428/trial_issac_sim/issues/61)で扱う
