# Step 3-15 Issue #53 / #54 / #58 マージ後ステータス確認

**ステータス**: 調査完了

**確認日**: 2026-07-20

**対象**: `main` / `334e8d8120e3bdce79cdc8facc10f76f02f6db58`（PR #65 マージ後）

**対象issue**:

- [#53 moving_to_place固着: servo_target_timeout後にreplanが二度と評価されないデッドロック](https://github.com/akodama428/trial_issac_sim/issues/53)
- [#54 一過性FALLEN誤検出が終端FAILEDへラッチされリリース前に固着する](https://github.com/akodama428/trial_issac_sim/issues/54)
- [#58 DETACHING縮退replanのレジリエンス欠陥](https://github.com/akodama428/trial_issac_sim/issues/58)

---

## 1. 結論

| Issue | 現在も元の問題か | 判定 | 根拠 |
| --- | --- | --- | --- |
| #53 | いいえ | 恒久デッドロックは解消済み | 0.3秒周期timerがabortを再評価する。既存10ケースmatrixでabort後のsuffix replanと完走を2件確認済み。 |
| #54 | いいえ（ただし根本誤検出は残る） | 終端FAILEDラッチは対策済み | FALLENを30連続tick確認してからFAILEDへ遷移し、非FALLENでcounterをresetする。誤検出そのものは`FrictionGraspStrategy`側に残る。 |
| #58 | はい | 未解決 | DETACHINGはabort後のsuffix replan対象外で、`missing_trajectory`連続回数上限・終端FAILED・要求phase trajectory存在検証のいずれもない。fault injection testもない。 |

GitHub上では3件ともOPENだが、実装と既存検証証跡に基づく実質状態は
「#53 解消済み、#54 実害対策済み、#58 未解決」である。

---

## 2. 確認範囲

### 2.1 マージ状態

- PR #60「Step 3-9〜3-11: physics E2E安定化統合」に#53/#54対応が含まれ、mainへマージ済み。
- PR #65「Issue #52: Add staged tray retreat for RETURNING_HOME」が2026-07-20にmainへマージ済み。
- ローカルHEAD `4cad1c8`とマージ後main `334e8d8`のtree差分は0件であり、本確認対象コードはマージ後mainと一致する。
- PR #65の変更対象はRETURNING_HOMEのtray退避であり、#53/#54の対策を削除していない。
- #58を修正するマージまたは現行コード差分は確認できない。

### 2.2 実行したテスト

```text
python3 -m pytest -q
347 passed, 2 skipped in 0.51s
```

重点関連テスト:

```text
test_phase_machine.py
test_replan_trigger.py
test_phase_suffix_replan.py
test_motion_command.py
test_servo_execution_adapter.py

109 passed in 0.15s
```

テスト全通過は回帰がないことを示す。ただし#58のfault injection testは存在しないため、
全テスト成功を#58解消の根拠にはできない。

---

## 3. Issue #53

### 3.1 元の故障

`servo_target_timeout`のabortがreplan minimum intervalに抑止された後、
execution adapterがstatus publishを停止するため、replan triggerが二度と評価されず
永久停止していた。

### 3.2 現行実装

`robot/motion_planner/node.py`は
`_REPLAN_TRIGGER_POLL_INTERVAL_SEC = 0.3`のtimerを生成し、
`/trajectory_status`受信とは独立に`_evaluate_replan_trigger()`を周期実行する。
一度`minimum_interval`に阻まれてもabort generationは未処理のまま残るため、
次回以降のtimerがabortを拾い、対象の自由空間phaseを再計画する。

### 3.3 既存E2E証跡

Issueコメントに記録された10ケースmatrixでは、次の2件で修正経路を実証済みである。

- `default`: RETURNING_HOME abort後0.52秒で`triggered_abort`、suffix replan採用後に完走。
- `near_singularity_extended`: 同じabort→timer再評価→suffix replan→完走。

当時の結果は8/10 PASSで、残る2失敗は物理把持滑落であり#53の永久沈黙ではなかった。

### 3.4 判定

**元issueの構造的な永久デッドロックは現在の問題ではない。**

Issue本文の未完了項目であるUI手動実行は、#54発見時の実運転で
`abort → 1.06秒後にreplan採用 → 到達成功`まで確認されている。
この証跡をissueへ反映してclose可能な状態である。

---

## 4. Issue #54

### 4.1 元の故障

搬送中に一過性の`TomatoStatus.FALLEN`を1 tick受けただけで、
phase machineが終端`FAILED`へ遷移していた。実把持が直後に`HELD`へ回復しても
phaseだけがFAILEDにラッチされ、サイクルを再開できなかった。

### 4.2 現行実装

`robot/behavior_planner/phase_machine.py`は次を実装している。

- `PhaseMachineState.fallen_steps`
- `FALLEN_CONFIRM_STEPS = 30`
- DETACHING / MOVING_TO_PLACE / RELEASING共通の`_confirm_fallen()`
- 非FALLEN snapshot受信時の`fallen_steps = 0`
- phase遷移時のcounter初期化

したがって、一過性FALLENは現phaseに留まって回復でき、実落下に相当する30連続FALLENだけが
FAILEDへ遷移する。

unit testは一過性FALLEN、counter reset、30連続FALLEN、対象3 phaseを固定している。

### 4.3 既存E2E証跡

実装後の10ケースmatrixは9/10 PASSで、直前runでFALLEN失敗した
`wrist_left`と`folded_near`はいずれもPASSした。ただしこのrunでは
`friction_grasp_slipped`自体が0件だったため、実E2Eでデバウンスが発火して
ライドスルーした直接証跡ではない。

### 4.4 判定

**元issueである「一過性FALLENが即時に終端FAILEDへラッチする問題」は現在の問題ではない。**

ただし次は残存する。

- `FrictionGraspStrategy`の5 mm滑落watchdogによる一過性FALLEN誤検出そのもの。
- 実E2EでFALLEN誤検出が発生し、30 tick以内にHELDへ戻って継続した直接artifact。

したがって、#54は「ラッチ欠陥の解消」としてcloseできるが、誤検出根絶を同じissueの
受け入れ条件にするなら未完了である。現在のissue目的と実装範囲からは、誤検出改善は
別issueへ分離するのが妥当である。

---

## 5. Issue #58

### 5.1 元の故障

DETACHINGでpull実行が失敗した後、再計画結果が要求phaseのpull trajectoryを含まないまま
成功扱いされると、executorが`missing_trajectory`でabortする。
同じ欠落planが再publishされ続ける場合、次の永久ループになる。

```text
missing_trajectory abort
  → trajectoryを欠いたplanを採用
  → motion command生成
  → missing_trajectory abort
  → ...
```

### 5.2 現行コードで残っている条件

- `servo_execution_adapter._on_command()`はtrajectory欠落時に
  `abort_reason="missing_trajectory"`をpublishするだけで、連続回数を保持しない。
- `phase_machine.advance(ExecutionAborted)`はDETACHINGを同phaseに留め、
  終端FAILEDへ遷移する上限を持たない。
- `SUFFIX_REPLAN_PHASES`はDETACHINGを明示的に除外している。
- `trigger_starts_planner()`はDETACHING abortでplannerを起動しない。
- plan採用境界に「現在phaseのtrajectoryが必ず存在する」という検証がない。
- `missing_trajectory`を注入して有限回でFAILEDまたは回復することを確認するtestがない。

PR #65のRETURNING_HOME退避変更はこれらの条件を変更していない。

### 5.3 影響範囲

既定のgrasp系Servo経路ではpullが正常実行される限り顕在化しにくいが、
pull計画・実行失敗時の復旧経路は依然として保証されない。
Step 3-12では同じ機序がMOVING_TO_GRASPのdirect JTC A/Bでも観測されているため、
DETACHINGだけの局所問題ではなく「要求phase trajectoryを欠いたplanの採用境界」の問題である。

### 5.4 判定

**#58は現在も問題であり、受け入れ条件は未達である。**

修正時は少なくとも次を必要とする。

1. 現在phaseに必要なtrajectoryを欠くcandidate planを採用しない。
2. `missing_trajectory`の連続abortに有限回上限を設け、回復不能時は明示的FAILEDへ遷移する。
3. DETACHINGとMOVING_TO_GRASPの両方についてfault injection testを追加する。
4. 可能なら欠落planが`success=true`になるplanner側縮退条件を修正する。

第一防御線はplan採用境界の必須trajectory検証、第二防御線は連続abort上限とする。
前者だけではplannerが同じ無効candidateを返し続ける負荷ループ、後者だけでは回復可能な
一時失敗を早期終端するため、両方が必要である。

---

## 6. 推奨するissue操作

| Issue | 推奨 |
| --- | --- |
| #53 | E2E証跡と現行347 test通過をコメントし、completedでcloseする。 |
| #54 | 「FAILEDラッチ解消」と「FALLEN誤検出自体は別課題」を明記し、completedでcloseする。必要なら誤検出改善issueを新規化する。 |
| #58 | OPENを維持し、次の実装対象にする。受け入れ条件へMOVING_TO_GRASPも追加する。 |

本レポートではGitHub issueの状態変更は行っていない。
