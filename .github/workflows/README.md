# GitHub Actions CI 実装解説

対象: [ci.yml](ci.yml) とそれが呼び出す [scripts/ci/](../../scripts/ci/) 一式、[docker/Dockerfile](../../docker/Dockerfile)、[docker/entrypoint.sh](../../docker/entrypoint.sh)

作成日: 2026-07-10(`/explain-code` による実装からの逆起こし)

## 1. 入出力、振る舞い

### 入力信号

**トリガー(ci.yml:3-6)**

- `push`: 全ブランチへの push で起動(ブランチフィルタなし)
- `pull_request`: 全 PR で起動。push と併発するため、同一 ref の重複は concurrency で抑制
- `workflow_dispatch`: 手動起動

**Secrets**

- `secrets.NGC_API_KEY`: nvcr.io(NGC レジストリ)へのログイン用。**未設定でも失敗せずスキップ**する設計(ci.yml:51-54)。ただしその場合、Isaac Sim ベースイメージの初回 pull は失敗しうる

**環境変数(ワークフロー定義値、ci.yml:20-30)**

- `CI_IMAGE_REPOSITORY` / `CI_IMAGE_TAG`: CI ベースイメージ名。`tomato-harvest-sim-ci-base:cached` 固定
- `CI_HEADLESS_STEPS="3600"`: ヘッドレス Isaac Sim が消化するシミュレーションステップ数。コメントに明記の通り「完了検知ではなく固定ステップ消化」であり、キャッシュの温まり具合による速度変動を見込んで収穫サイクル完走に余裕を持たせた値
- `CI_E2E_TIMEOUT_SEC="2400"`: E2E 全体の強制打ち切り時間(40分)
- `CI_ARTIFACT_ROOT`: 成果物集約先(`$GITHUB_WORKSPACE/.artifacts/ci`)
- `ACCEPT_EULA` / `PRIVACY_CONSENT`: Isaac Sim の EULA・テレメトリ同意

**ランナー側の暗黙依存(スクリプトが参照)**

- `CI_CACHE_ROOT`(未設定時 `/tmp/tomato-harvest-sim-cache-github-actions`): ジョブをまたいで永続するランナーローカルのキャッシュディレクトリ
- self-hosted ランナーに `docker`、`nvidia-smi`(= GPU + NVIDIA Container Toolkit)が存在すること

### 出力信号

- **ジョブ成否**: unit テスト・E2E のいずれかが失敗すると赤
- **アーティファクト `ci-artifacts-<run_id>`**(ci.yml:69-75): `if: always()` で失敗時も必ずアップロード。内容は:
  - `runner_prereqs.txt`(ランナー環境スナップショット)、`docker-build.log`、`image_ref.txt`
  - `unit/`: `docker-unit-console.log`、`colcon-build.log`、`pytest.log`、`pytest-results.xml`、`colcon-test.log`、`colcon-test-result.log`
  - `e2e/`: `docker-e2e-console.log`、`run_ros2_components.log`、`franka_controller.log`、`robot_node.log`
- **ランナーローカルの副作用**: CI ベースイメージ(docker image)と `CI_CACHE_ROOT` 配下のキャッシュ(unit-colcon / franka-ws / ci-home / kit-cache)が次回実行のために残る

### モジュール内の処理概要

`unit-and-servo-e2e`の単一ジョブで、**「unit test → MoveIt Servo E2E」**を実行する。ServoがJTCを排他所有し、収穫サイクル完走とJTC feedback由来のtracking error周期配信を検証する。

各ステップの要点:

1. **Clean workspace residue**(ci.yml:32-39): checkout 前に alpine コンテナで workspace とキャッシュを `chown` する。過去のコンテナ実行が root 所有ファイルを残すと `actions/checkout` が削除できず失敗するため(コミット e04f8b2 で入った CI 修復)。`|| true` で失敗しても続行
2. **Check runner prerequisites**([check_runner_prereqs.sh](../../scripts/ci/check_runner_prereqs.sh)): `docker`/`nvidia-smi` の存在確認と、`timeout 20s docker ps` によるデーモン応答確認(`docker info` がハングする環境対策のコメントあり)。環境情報を `runner_prereqs.txt` に記録
3. **Prepare cached CI base image**([build_ci_image.sh](../../scripts/ci/build_ci_image.sh)): 「Isaac Sim イメージ名 + Dockerfile + entrypoint.sh の sha256」を合成したフィンガープリントを計算し、既存イメージのラベル `com.tomato_harvest.ci_base_fingerprint` と一致すれば**ビルドをスキップして再利用**。不一致なら `--target ci-base` で再ビルド。GitHub のキャッシュ機構ではなくランナーローカルの docker イメージ自体をキャッシュとして使う自作キャッシュ機構
4. **Run unit tests**([run_unit_tests.sh](../../scripts/ci/run_unit_tests.sh) → [in_container_unit_tests.sh](../../scripts/ci/in_container_unit_tests.sh)): GPU なしのコンテナで `colcon build`(franka_ros2_control まで)→ `pytest`(tests / robot / simulator)→ `colcon test` を実行。3つの終了コードを個別に保持し、**全部走らせてから** build → pytest → colcon の優先順で失敗を報告する(1つ失敗しても残りのログが取れる)
5. **Run Isaac Sim E2E**([run_e2e.sh](../../scripts/ci/run_e2e.sh) → [in_container_e2e.sh](../../scripts/ci/in_container_e2e.sh)): `--gpus all --network host --shm-size=1g` のコンテナで `run_ros2_components.sh --isaac --moveit --headless --headless-steps 3600 --auto-start` を `timeout --signal=INT 2400` 付きで実行。成否は終了コードに加えて**ログの文字列マーカーで判定**:
   - `Headless simulator node setup completed.` がスタックログにあること
   - `Phase: returning_home .* complete` がロボットログにあること(収穫サイクル完走)
   - `Phase: .* failed` がロボットログに**ないこと**
   - `timeout` の終了コード 124 は「E2E タイムアウト」として明示的にエラー化

**非 root 実行の仕組み(unit / E2E 共通)**: コンテナは `--user "$(id -u):$(id -g)"`(ホストユーザー)で実行し、root 所有ファイルの残骸を作らない。Isaac Sim イメージ内の `/isaac-sim` は `isaac-sim:isaac-sim` の 750 のため、`docker run` で `id -g isaac-sim` を取得(失敗時 1234 にフォールバック)して `--group-add` で読み取り権を得る。`HOME` はコンテナ内の書込可能パスに差し替え、リポジトリは `:ro` マウント、書き込みはすべてアーティファクト/キャッシュ用ボリュームへ向ける。

## 2. モジュール内の構成

```mermaid
flowchart TD
  T[pull_request / workflow_dispatch] --> C{concurrency<br>同一refの旧実行をキャンセル}
  C --> S1[Servo job<br/>Clean workspace residue]
  S1 --> S2[actions/checkout@v4]
  S2 --> S3[Prepare artifact root]
  S3 --> S4[Login to NGC<br>キー未設定ならスキップ]
  S4 --> S5[check_runner_prereqs.sh<br>docker/nvidia-smi/デーモン応答確認]
  S5 --> S6[build_ci_image.sh<br>フィンガープリント一致なら再利用<br>不一致なら docker build --target ci-base]
  S6 --> S7[run_unit_tests.sh<br>GPUなしコンテナ起動]
  S7 --> S7i[in_container_unit_tests.sh<br>colcon build → pytest → colcon test]
  S7i --> S8[run_e2e.sh<br>GPU+host networkコンテナ起動]
  S8 --> S8i[in_container_e2e.sh<br>timeout付きで run_ros2_components.sh<br>--headless-steps 3600 実行<br>ログマーカーで成否判定]
  S8i --> S9[Servo artifact upload]
  S9 --> L1[Legacy job<br/>explicit off mode]
  L1 --> L2[local and suffix disturbance E2E]
  L2 --> L3[Legacy artifact upload]

  subgraph ランナーローカル状態
    IMG[(tomato-harvest-sim-ci-base:cached)]
    CACHE[(CI_CACHE_ROOT<br>unit-colcon / franka-ws<br>ci-home / kit-cache)]
  end
  S6 -.読み書き.-> IMG
  S7 -.マウント.-> CACHE
  S8 -.マウント.-> CACHE
```

- **[ci.yml](ci.yml)**: default Servo検証とlegacy local-planner検証を別ジョブとして定義し、`needs`で直列化する
- **[check_runner_prereqs.sh](../../scripts/ci/check_runner_prereqs.sh)**: ランナー前提条件の検証と環境スナップショット記録
- **[build_ci_image.sh](../../scripts/ci/build_ci_image.sh)**: フィンガープリントベースの CI イメージキャッシュ管理
- **[run_unit_tests.sh](../../scripts/ci/run_unit_tests.sh) / [run_e2e.sh](../../scripts/ci/run_e2e.sh)**: ホスト側。`docker run` の引数組み立て(ユーザー・グループ・マウント・環境変数)が責務
- **[in_container_unit_tests.sh](../../scripts/ci/in_container_unit_tests.sh) / [in_container_e2e.sh](../../scripts/ci/in_container_e2e.sh)**: コンテナ内。テスト実行と成否判定が責務
- **[docker/Dockerfile](../../docker/Dockerfile)**: `ci-base`(Isaac Sim 6.0.0 + ROS 2 Jazzy + MoveIt + colcon。CI はこちらだけ使用)と `app`(ソースを COPY して colcon build 済み。CI では未使用)の2ステージ
- **[docker/entrypoint.sh](../../docker/entrypoint.sh)**: ROS 2 と colcon install の setup.bash を source し、GUI 起動ハングの原因になる `HUB__ARGS__DETECT_ONLY` を unset してからコマンドを exec

## 3. モジュールの要件

実装から逆起こしした要件:

- push / PR / 手動のいずれでも同一の unit + E2E 検証を実行できること。同一 ref に対する古い実行は自動キャンセルされること
- GPU 付き self-hosted ランナーで動作し、実行前に docker・GPU・デーモン応答を検証して前提不足を早期に検出できること
- 前回実行が root 所有ファイルを残していても checkout が失敗しないこと(実行前の残骸掃除)
- コンテナ内の全プロセスをホストユーザー権限で実行し、workspace とキャッシュに root 所有ファイルを新たに作らないこと
- Dockerfile / entrypoint / ベースイメージが変わらない限り CI イメージを再ビルドせず、変わったときのみ自動で再ビルドすること
- colcon ビルド成果物・Franka ワークスペース・Isaac Sim の kit キャッシュ・HOME キャッシュを実行間で再利用し、ウォームキャッシュ時の実行時間を短縮できること
- unit テストで build / pytest / colcon test のどれかが失敗しても残りを実行し切り、全ログを成果物として残した上で失敗を報告すること
- E2E は 2400 秒で強制打ち切りされ、タイムアウトを通常失敗と区別して報告すること
- E2E の成功判定は終了コードだけでなく、「ヘッドレスセットアップ完了」「収穫サイクル完走(returning_home complete)」「失敗フェーズ遷移なし」の3マーカーで担保すること
- 成否にかかわらず、全ステップのログを 1 つのアーティファクトとしてダウンロードできること
- NGC API キーがなくてもワークフロー自体は破綻しないこと(ローカルにイメージがあれば動く)

**不明点 / リファクタリング時の注意**

- `CI_HEADLESS_STEPS=3600` は「完了検知なしの固定ステップ消化」への時間ベースの当て付けであり、マシン性能・キャッシュ状態に依存する暗黙の結合がある(ci.yml のコメント自体がこれを認めている)。なお feature/headless-early-exit ブランチで早期終了機構が入ったため、この値は「上限」としての意味に変わっている可能性が高い — `run_ros2_components.sh` 側の仕様と合わせて確認が必要
- `run_e2e.sh` の `--network host` は、ランナー上で他の ROS 2 プロセスが動いているとトピック混線しうる(concurrency 設定で同時実行は抑制されているが、CI 以外のプロセスは防げない)
- E2E の成否判定がログの文言(`Headless simulator node setup completed.` など)に密結合しており、アプリ側のログメッセージ変更で CI が壊れる
- `CI_CACHE_ROOT` が `/tmp` 配下のため、ランナー再起動で消える前提(消えても動くが初回は遅い)
