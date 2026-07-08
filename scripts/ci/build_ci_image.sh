#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ARTIFACT_ROOT="${CI_ARTIFACT_ROOT:-${REPO_ROOT}/.artifacts/ci}"
IMAGE_REPOSITORY="${CI_IMAGE_REPOSITORY:-tomato-harvest-sim-ci}"
IMAGE_TAG="${CI_IMAGE_TAG:-local}"
IMAGE_NAME="${IMAGE_REPOSITORY}:${IMAGE_TAG}"

mkdir -p "${ARTIFACT_ROOT}"

(
  cd "${REPO_ROOT}"
  IMAGE_NAME="${IMAGE_NAME}" bash ./build.sh
) 2>&1 | tee "${ARTIFACT_ROOT}/docker-build.log"

printf '%s\n' "${IMAGE_NAME}" > "${ARTIFACT_ROOT}/image_ref.txt"
