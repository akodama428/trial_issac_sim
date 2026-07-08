#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ARTIFACT_ROOT="${CI_ARTIFACT_ROOT:-${REPO_ROOT}/.artifacts/ci}"
ARTIFACT_DIR="${ARTIFACT_ROOT}/unit"
IMAGE_REPOSITORY="${CI_IMAGE_REPOSITORY:-tomato-harvest-sim-ci}"
IMAGE_TAG="${CI_IMAGE_TAG:-local}"
IMAGE_NAME="${IMAGE_REPOSITORY}:${IMAGE_TAG}"

mkdir -p "${ARTIFACT_DIR}"

docker run --rm \
  -e CI_ARTIFACT_DIR=/tmp/tomato-harvest-ci-artifacts \
  -v "${ARTIFACT_DIR}:/tmp/tomato-harvest-ci-artifacts" \
  "${IMAGE_NAME}" \
  bash ./scripts/ci/in_container_unit_tests.sh \
  2>&1 | tee "${ARTIFACT_DIR}/docker-unit-console.log"
