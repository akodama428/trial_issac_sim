# .codex の使い方

## Codex CLI 向け

`codex/AGENTS.md` は、リポジトリ直下の `AGENTS.md` にコピーして使用してください。

リポジトリ直下に `AGENTS.md` を配置すると、Codex のエージェントがデフォルトで参照するためです。

あわせて、`AGENTS.md` の冒頭には、リポジトリ直下から `codex` フォルダまでの相対パスを記載してください。

## Claude Code 向け

Claude Code 用の設定は `.claude/` フォルダに配置されています。

| .codex の場所 | Claude Code の場所 | 用途 |
|---|---|---|
| `AGENTS.md` | `CLAUDE.md` | メイン設定・オーケストレーション方針 |
| `.codex/skills/*/SKILL.md` | `.claude/commands/*.md` | スラッシュコマンド（`/adr-review` など） |
| `.codex/agents/*.md` | `.claude/agents/*.md` | サブエージェント |

### 利用可能なスラッシュコマンド
- `/adr-design` — ADR 設計
- `/adr-review` — ADR レビュー
- `/architecture-design` — 詳細アーキテクチャ設計
- `/architecture-review` — 詳細アーキテクチャレビュー
- `/explain-code` — 既存コード解説
- `/implementation` — 実装作業
- `/poc-design` — PoC 設計
- `/requirements-review` — 要件定義レビュー
- `/requirements-supporter` — 要件定義の軽微修正

### 利用可能なサブエージェント
- `adr-reviewer` — 独立 ADR レビュアー
- `architecture-reviewer` — 独立アーキテクチャレビュアー
- `poc-design-reviewer` — 独立 PoC レビュアー
- `requirements-reviewer` — 独立要件定義レビュアー
- `web-researcher` — Web 調査エージェント
