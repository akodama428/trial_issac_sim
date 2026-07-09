#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ARTIFACT_ROOT="${CI_ARTIFACT_ROOT:-${REPO_ROOT}/.artifacts/ci}"
ARTIFACT_DIR="${ARTIFACT_ROOT}/unit"
IMAGE_REPOSITORY="${CI_IMAGE_REPOSITORY:-tomato-harvest-sim-ci-base}"
IMAGE_TAG="${CI_IMAGE_TAG:-cached}"
IMAGE_NAME="${IMAGE_REPOSITORY}:${IMAGE_TAG}"
CACHE_ROOT="${CI_CACHE_ROOT:-/tmp/tomato-harvest-sim-cache-github-actions}"

mkdir -p "${ARTIFACT_DIR}" "${CACHE_ROOT}/unit-colcon"

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -e CI_ARTIFACT_DIR=/tmp/tomato-harvest-ci-artifacts \
  -e CI_COLCON_ROOT=/tmp/tomato-harvest-ci-colcon \
  -e PYTHONDONTWRITEBYTECODE=1 \
  -e HOME=/tmp/tomato-harvest-ci-home \
  -v "${ARTIFACT_DIR}:/tmp/tomato-harvest-ci-artifacts" \
  -v "${CACHE_ROOT}/unit-colcon:/tmp/tomato-harvest-ci-colcon" \
  -v "${REPO_ROOT}:/workspace/tomato-harvest:ro" \
  -w /workspace/tomato-harvest \
  "${IMAGE_NAME}" \
  bash ./scripts/ci/in_container_unit_tests.sh \
  2>&1 | tee "${ARTIFACT_DIR}/docker-unit-console.log"
