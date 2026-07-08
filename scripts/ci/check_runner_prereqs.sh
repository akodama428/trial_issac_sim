#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ARTIFACT_ROOT="${CI_ARTIFACT_ROOT:-${REPO_ROOT}/.artifacts/ci}"
SUMMARY_FILE="${ARTIFACT_ROOT}/runner_prereqs.txt"

mkdir -p "${ARTIFACT_ROOT}"

require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "Missing required command: ${command_name}" >&2
    exit 1
  fi
}

require_command docker
require_command nvidia-smi

docker info >/dev/null

{
  echo "date=$(date -Iseconds)"
  echo "runner=$(hostname)"
  echo "docker=$(docker --version)"
  echo "gpu=$(nvidia-smi --query-gpu=name --format=csv,noheader | paste -sd ',' -)"
} > "${SUMMARY_FILE}"

cat "${SUMMARY_FILE}"
