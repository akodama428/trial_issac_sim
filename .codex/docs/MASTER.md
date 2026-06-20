---
title: MASTER.md
version: 0.1.0
status: draft
owner: atsushi
created: 2026-06-17
updated: 2026-06-18
---

# 目的
このリポジトリの文書状態と進行順を管理する。

## 文書索引
| Document | Status | Notes |
| --- | --- | --- |
| `RESEARCH.md` | `draft` | Isaac Sim Docker + ROS 2 Jazzy + Franka Panda + eye-to-hand tomato harvest 構成を調査中 |
| `USERS_GUIDE.md` | `draft` | Isaac Sim 初心者向けに、起動から収穫、結果確認、リセットまでの使い方を定義 |
| `POC.md` | `draft` | 5 スプリントで起動、可視化、収穫動作、detach、reset を積み上げる PoC 計画 |
| `REQUIREMENTS.md` | `not_created` | 未作成 |
| `TESTING.md` | `not_created` | 未作成 |
| `ADR.md` | `draft` | 1 コンテナ内レイヤ分離型を本番アーキテクチャの推奨案として整理 |
| `ARCHITECTURE.md` | `not_created` | 未作成 |
| `PATTERNS.md` | `not_created` | 未作成 |

## 進行順
`RESEARCH.md` → `USERS_GUIDE.md` → `POC.md` → `REQUIREMENTS.md` → `TESTING.md` → `ADR.md` → `ARCHITECTURE.md` → `PATTERNS.md`

## 現在の焦点
- トマトをロボットハンドで収穫するシミュレータの初期構成を調査する。
- PoC 結果を踏まえ、本番用のレイヤ分離アーキテクチャを `ADR.md` に整理する。
