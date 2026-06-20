この文書の冒頭には、リポジトリ直下から `codex` フォルダまでの相対パスを必ず記載すること。
`codex_relative_path: ./.codex`

この文書中の `docs/`、`agents/`、`skills/` は、すべて上記の相対パス配下、すなわち `codex` フォルダ内に配置されるものとして扱う。

この文書は、このリポジトリ全体を進めるためのオーケストレーション方針を定義する。

## 基本方針
- このリポジトリでは、`<codex_relative_path>/docs/` 配下の文書に仕様を記載する。
- 仕様変更前、実装前には、必ず `<codex_relative_path>/docs/MASTER.md` を読む。
- 今回の作業では新規開発を主目的とするため、設計・実装・レビュー前には、必ず `<codex_relative_path>/docs/RESEARCH.md` を読んで外部調査と前提条件を揃える。
- 与えられた課題に対しては、まず Web の一次情報を調査し、その結果を `<codex_relative_path>/docs/RESEARCH.md` へ整理してから、要件定義・設計・実装へ進む。
- 今回の作業では、`<codex_relative_path>/docs/USERS_GUIDE.md` で利用者視点の操作と期待体験を先に定義し、その内容を最小実装で試す `<codex_relative_path>/docs/POC.md` を作成して、使い勝手を先に検証する。
- 今回の作業では、`<codex_relative_path>/docs/REQUIREMENTS.md` まで整理した後、期待挙動と受け入れ条件をテストで担保するために `<codex_relative_path>/docs/TESTING.md` を先に整備してから実装へ進む。
- タスク実行時は、`<codex_relative_path>/skills/` 配下のスキルを確認し、該当するスキルがあればそれを使う。
- `<codex_relative_path>/docs/REQUIREMENTS.md` を新規作成または大きく更新する場合は、利用者視点の操作手順を記載する `<codex_relative_path>/docs/USERS_GUIDE.md` も合わせて作成または更新する。

## 「次のステップを教えて」と指示された場合の必須手順
ユーザーから「次のステップを教えて」と指示された場合は、必ず最初に以下を実施すること。

1. 最低限必要な文書の存在確認を行う。
2. `<codex_relative_path>/skills/docs-status-sync/SKILL.md` が存在する場合はそれを用い、存在しない場合は各文書の frontmatter を手動確認して、`<codex_relative_path>/docs/MASTER.md` の `## 文書索引` を最新化する。
3. `RESEARCH.md → USERS_GUIDE.md → POC.md → REQUIREMENTS.md → TESTING.md → ADR.md → ARCHITECTURE.md → PATTERNS.md` の順で、最初に `draft` の文書を特定する。
4. `draft` の文書があれば、そのレビューまたは更新を次のステップとしてユーザーに提案する。

上記を実施する前に、次の設計・実装・レビュー作業へ進んではいけない。

## 各ワークフロー詳細

### 1. 最低限必要な文書の存在確認
最初に、最低限以下の文書が存在することを確認する。

- `<codex_relative_path>/docs/MASTER.md`
  - 全体の状態管理を担う
- `<codex_relative_path>/docs/RESEARCH.md`
  - 新規開発前の外部調査、参考実装、技術制約を記載する
- `<codex_relative_path>/docs/USERS_GUIDE.md`
  - 利用者向けの利用手順・操作方法を記載する
- `<codex_relative_path>/docs/POC.md`
  - USERS_GUIDE に記載した操作と体験を最小実装で試し、使い勝手を検証する
- `<codex_relative_path>/docs/REQUIREMENTS.md`
  - 新規開発で満たすべき要件を整理する
- `<codex_relative_path>/docs/TESTING.md`
  - 期待挙動を担保し、実装前後で確認すべきテスト戦略を記載する
- `<codex_relative_path>/docs/ADR.md`
  - アーキテクチャ設計書
- `<codex_relative_path>/docs/ARCHITECTURE.md`
  - 最終的に決定したアーキテクチャと選定根拠
- `<codex_relative_path>/docs/PATTERNS.md`
  - 実装パターン、コーディング規則

これらの文書が存在しない場合は、以下のように frontmatter 付きの文書を新規作成すること。

```md
---
title: ARCHITECTURE.md
version: 0.1.0
status: draft
owner: atsushi
created: 2026-06-17
updated: 2026-06-17
---
```

### 2. 文書 status の確認と同期
- `<codex_relative_path>/skills/docs-status-sync/SKILL.md` が存在する場合はそれを使い、存在しない場合は各文書の frontmatter を直接確認して `status` を確認する。
- `<codex_relative_path>/docs/MASTER.md` の `## 文書索引` を、各文書の最新 `status` に更新する。

### 3. 文書の進行順序
文書は、基本的に以下の順番で進める。

1. `RESEARCH.md`
2. `USERS_GUIDE.md`
3. `POC.md`
4. `REQUIREMENTS.md`
5. `TESTING.md`
6. `ADR.md`
7. `ARCHITECTURE.md`
8. `PATTERNS.md`

上の文書が承認済みになるまで、次の文書のステップには進まないこと。

### 4. draft 文書の扱い
- 対象文書の `status` が `draft` の場合は、その文書をレビューするよう提案すること。
- 必要に応じて、文書の記載不足、要件不足、設計不足を指摘すること。
- `draft` のまま次のステップへ進めないこと。

## 各ワークフローの実行方法
### 事前調査
- 新規開発の事前調査を行う場合や、`<codex_relative_path>/docs/RESEARCH.md` を作成または更新する場合は、まず `<codex_relative_path>/agents/web-researcher.md` の方針に従って Web の一次情報を収集する。
- 調査では、確認済みの事実と推測を分離し、参照した URL、確認日、対象バージョンを記載する。
- 特に Isaac Sim と Franka を扱う場合は、NVIDIA 公式ドキュメント、Isaac Lab 公式ドキュメント、Franka Robotics 公式ドキュメントと公式 GitHub を優先する。

### 既存モジュール理解が必要な場合
- 新規開発であっても、既存モジュールの実装内容を解説する場合は、`<codex_relative_path>/skills/explain_code/SKILL.md` を使う。
- 解説は、入出力と各信号の意味、振る舞い、サブモジュール構成、フローチャート、実装から逆起こしした要件を含めて整理する。

### 要件定義レビュー
- 要件定義レビューを依頼された場合は、`<codex_relative_path>/skills/requirements-review/SKILL.md` を基準本体としつつ、独立 reviewer として `<codex_relative_path>/agents/requirements-reviewer.md` の方針で実施する。
- `<codex_relative_path>/docs/REQUIREMENTS.md` を作成または更新する場合は、`<codex_relative_path>/docs/USERS_GUIDE.md` も合わせて作成または更新し、要件と利用手順の整合を確認する。
- 要件定義レビュー後の指摘のうち、人間確認が不要な軽微修正を行う場合は、`<codex_relative_path>/skills/requirements-supporter/SKILL.md` を使う。毎回のレビュー後に必ず実施して、修正内容、未対応案件を伝えてください。

### アーキテクチャ設計
- ADR 設計を依頼された場合は、`<codex_relative_path>/skills/adr-design/SKILL.md` を使う。
- ADR で決定した案の詳細アーキテクチャ設計を依頼された場合は、`<codex_relative_path>/skills/architecture-design/SKILL.md` を使う。

### PoC 設計
- PoC 設計を依頼された場合は、`<codex_relative_path>/skills/poc-design/SKILL.md` を使う。
- PoC は `USERS_GUIDE.md` に記載した利用者操作と期待体験を、最小構成で実際に触って検証するための文書として扱う。
- PoC の結果は `REQUIREMENTS.md` の具体化と優先順位付けに必ず反映する。

### 実装
- 機能追加、バグ修正、実装作業を行う場合は、`<codex_relative_path>/skills/implementation/SKILL.md` を使う。

### アーキテクチャレビュー
- ADR レビューを依頼された場合は、
`<codex_relative_path>/skills/adr-review/SKILL.md` を基準本体としつつ、独立 reviewer として `<codex_relative_path>/agents/adr-reviewer.md` の方針で実施する。
- `<codex_relative_path>/docs/POC.md` のレビューを依頼された場合は、
`<codex_relative_path>/skills/poc-design/SKILL.md` を基準本体としつつ、独立 reviewer として `<codex_relative_path>/agents/poc-design-reviewer.md` の方針で実施する。
- 詳細アーキテクチャレビューを依頼された場合は、
`<codex_relative_path>/skills/architecture-review/SKILL.md` を基準本体としつつ、独立 reviewer として `<codex_relative_path>/agents/architecture-reviewer.md` の方針で実施する。
