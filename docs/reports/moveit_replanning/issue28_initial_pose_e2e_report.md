# Issue #28 初期姿勢10ケースE2E CIレポート

## 目的

初期関節姿勢による把持成否のばらつきを、同じ10姿勢で継続測定する。プラン生成だけでなく、Isaac Sim上で把持・detach・placeを経て`complete`へ到達した場合だけ成功とする。

## 初期姿勢

関節順はpanda_joint1からpanda_joint7、単位はradである。

| ID | 関節角 | 狙い |
|---|---|---|
| default | 0,-0.4,0,-2.1,0,1.7,0.8 | 標準 |
| elbow_left | 0.35,-0.55,-0.25,-2.0,0.20,1.55,0.55 | 左肘 |
| elbow_right | -0.35,-0.55,0.25,-2.0,-0.20,1.55,1.05 | 右肘 |
| shoulder_high | 0.10,0.15,-0.15,-1.65,0.10,1.85,0.70 | 高い肩 |
| shoulder_low | -0.10,-0.85,0.15,-2.35,-0.10,1.45,0.90 | 低い肩 |
| wrist_left | 0.15,-0.45,-0.10,-2.05,0.65,1.65,0.25 | 左手首 |
| wrist_right | -0.15,-0.45,0.10,-2.05,-0.65,1.65,1.35 | 右手首 |
| folded_near | 0,-0.15,0,-2.65,0,2.45,0.8 | 折畳み |
| extended_far | 0.20,-0.25,-0.15,-1.05,0.10,0.75,0.50 | 伸展 |
| near_singularity_extended | 0,-0.05,0,-0.10,0,0.15,0 | **伸展特異姿勢近傍** |

全ケースは固定値で、7軸・有限値・Panda関節制限内をunit testで検証する。自己衝突と実到達性はMoveIt/Isaac E2Eの結果として記録する。特異姿勢ケースは成功を前提に除外せず、同じ成功率の分母へ含める。

## 成功判定と許容誤差

MoveItのgrasp pose constraintは位置`0.01 m`、各軸姿勢`0.10 rad`を用いる。API成功だけでは合格にせず、実行後に`AT_GRASP`を経てstable graspが成立し、最終的に`returning_home -> complete`となり、途中にfailed phaseがないことを要求する。suffix replanの有無は成功条件に含めない。

## CI設計と履歴

通常PR CIを10倍にしないため、専用workflowをmain push、週次、手動で実行する。10件はGPU競合を避けて直列実行し、1件失敗しても残りを続行する。各caseのrobot/controller/MoveIt/Isaacログ、JSON集計、Markdown集計をartifact `initial-pose-history-<sha>-<run-id>`として90日保存し、Job Summaryにも表示する。commit SHA付きJSONにより複数commitを比較できる。

初期閾値は成功率`70%`（7/10）とする。根拠は、標準を含む通常9姿勢の大半を守りながら、特異姿勢を含む最大3件の既知弱点をまず計測可能にするためである。ベースラインが3回蓄積した後、flakeを確認して閾値を引き上げる。

## 実行方法

全件:

```bash
CI_HEADLESS_STEPS=3600 bash scripts/ci/run_initial_pose_matrix.sh
```

単一ケース:

```bash
INITIAL_POSE_CASE_IDS=near_singularity_extended CI_HEADLESS_STEPS=3600 bash scripts/ci/run_initial_pose_matrix.sh
```

結果は`.artifacts/initial-pose-e2e/initial-pose-summary.md`とJSONに出力される。

## 特異姿勢ケースの先行実測

`near_singularity_extended`を単独実行し、初期姿勢がIsaac Simへ設定されたことをログで確認した。MoveIt初期planningは`340.301 ms`で成功し、`moving_to_pregrasp`、`moving_to_grasp`、`at_grasp`までは到達したが、安定把持が成立せず`grasp_evaluation -> failed`となった。E2E所要時間は`133秒`、結果は`0/1`である。

これはIssue #28の計測基盤が「plan API成功」を収穫成功と誤判定せず、特異姿勢からの実行結果を失敗phase付きで記録できたことを示す。Issueの非スコープに従い、このPRでは特異姿勢向けplanner改善は行わない。
