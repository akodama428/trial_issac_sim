#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ARTIFACT_ROOT="${CI_ARTIFACT_ROOT:-${REPO_ROOT}/.artifacts/ci}"
ARTIFACT_DIR="${ARTIFACT_ROOT}/e2e"
IMAGE_REPOSITORY="${CI_IMAGE_REPOSITORY:-tomato-harvest-sim-ci-base}"
IMAGE_TAG="${CI_IMAGE_TAG:-cached}"
IMAGE_NAME="${IMAGE_REPOSITORY}:${IMAGE_TAG}"
CACHE_ROOT="${CI_CACHE_ROOT:-/tmp/tomato-harvest-sim-cache-github-actions}"

mkdir -p \
  "${ARTIFACT_DIR}" \
  "${CACHE_ROOT}/franka-ws" \
  "${CACHE_ROOT}/ov-cache" \
  "${CACHE_ROOT}/ov-data" \
  "${CACHE_ROOT}/ov-logs" \
  "${CACHE_ROOT}/kit-cache"

docker run --rm \
  --gpus all \
  --network host \
  --shm-size=1g \
  -e ACCEPT_EULA="${ACCEPT_EULA:-Y}" \
  -e PRIVACY_CONSENT="${PRIVACY_CONSENT:-Y}" \
  -e CI_ARTIFACT_DIR=/tmp/tomato-harvest-ci-artifacts \
  -e FRANKA_ROS2_WS=/tmp/tomato-harvest-ci-franka-ws \
  -e CI_HEADLESS_STEPS="${CI_HEADLESS_STEPS:-900}" \
  -e CI_E2E_TIMEOUT_SEC="${CI_E2E_TIMEOUT_SEC:-2400}" \
  -v "${ARTIFACT_DIR}:/tmp/tomato-harvest-ci-artifacts" \
  -v "${CACHE_ROOT}/franka-ws:/tmp/tomato-harvest-ci-franka-ws" \
  -v "${CACHE_ROOT}/ov-cache:/root/.cache/ov" \
  -v "${CACHE_ROOT}/ov-data:/root/.local/share/ov/data" \
  -v "${CACHE_ROOT}/ov-logs:/root/.nvidia-omniverse/logs" \
  -v "${CACHE_ROOT}/kit-cache:/isaac-sim/kit/cache" \
  -v "${REPO_ROOT}:/workspace/tomato-harvest" \
  -w /workspace/tomato-harvest \
  "${IMAGE_NAME}" \
  bash ./scripts/ci/in_container_e2e.sh \
  2>&1 | tee "${ARTIFACT_DIR}/docker-e2e-console.log"
