#!/usr/bin/env bash
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RESULT_ROOT="${CI_ARTIFACT_ROOT:-${REPO_ROOT}/.artifacts/initial-pose-e2e}"
CASES="default,elbow_left,elbow_right,shoulder_high,shoulder_low,wrist_left,wrist_right,folded_near,extended_far,near_singularity_extended"
mkdir -p "${RESULT_ROOT}"

IFS=',' read -ra CASE_IDS <<< "${INITIAL_POSE_CASE_IDS:-${CASES}}"
for case_id in "${CASE_IDS[@]}"; do
  started="$(date +%s)"
  echo "Running initial pose case: ${case_id}"
  # 外乱注入は無効のまま、Issue #28 改善3のlocal planner補正を通常運転として有効化する。
  CI_ARTIFACT_ROOT="${RESULT_ROOT}/${case_id}" \
  TOMATO_HARVEST_INITIAL_POSE_ID="${case_id}" \
  TOMATO_HARVEST_DEBUG_PHYSICS_GRASP="${INITIAL_POSE_DEBUG_PHYSICS-1}" \
    bash "${REPO_ROOT}/scripts/ci/run_e2e.sh"
  status=$?
  ended="$(date +%s)"
  echo "E2E_CASE_DURATION_SEC=$((ended - started))" >> "${RESULT_ROOT}/${case_id}/e2e/docker-e2e-console.log"
  echo "Case ${case_id} exit status: ${status}"
done

python3 "${REPO_ROOT}/scripts/ci/summarize_initial_pose_e2e.py" \
  --root "${RESULT_ROOT}" \
  --cases "$(IFS=,; echo "${CASE_IDS[*]}")" \
  --sha "${GITHUB_SHA:-local}" \
  --threshold "${INITIAL_POSE_SUCCESS_THRESHOLD:-0.70}"
