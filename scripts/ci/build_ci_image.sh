#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ARTIFACT_ROOT="${CI_ARTIFACT_ROOT:-${REPO_ROOT}/.artifacts/ci}"
IMAGE_REPOSITORY="${CI_IMAGE_REPOSITORY:-tomato-harvest-sim-ci-base}"
IMAGE_TAG="${CI_IMAGE_TAG:-cached}"
IMAGE_NAME="${IMAGE_REPOSITORY}:${IMAGE_TAG}"
DOCKERFILE_PATH="${DOCKERFILE_PATH:-docker/Dockerfile}"
ISAAC_SIM_IMAGE="${ISAAC_SIM_IMAGE:-nvcr.io/nvidia/isaac-sim:6.0.0}"
FINGERPRINT="$(
  {
    printf '%s\n' "${ISAAC_SIM_IMAGE}"
    sha256sum \
      "${REPO_ROOT}/${DOCKERFILE_PATH}" \
      "${REPO_ROOT}/docker/entrypoint.sh"
  } | sha256sum | cut -d' ' -f1
)"
CURRENT_FINGERPRINT="$(
  docker image inspect "${IMAGE_NAME}" \
    --format '{{ index .Config.Labels "com.tomato_harvest.ci_base_fingerprint" }}' \
    2>/dev/null || true
)"

mkdir -p "${ARTIFACT_ROOT}"

if [[ -n "${CURRENT_FINGERPRINT}" && "${CURRENT_FINGERPRINT}" == "${FINGERPRINT}" ]]; then
  {
    echo "Reusing cached CI base image: ${IMAGE_NAME}"
    echo "fingerprint=${FINGERPRINT}"
  } | tee "${ARTIFACT_ROOT}/docker-build.log"
else
  (
    cd "${REPO_ROOT}"
    docker build \
      --build-arg "ISAAC_SIM_IMAGE=${ISAAC_SIM_IMAGE}" \
      --label "com.tomato_harvest.ci_base_fingerprint=${FINGERPRINT}" \
      --target ci-base \
      -t "${IMAGE_NAME}" \
      -f "${DOCKERFILE_PATH}" \
      .
  ) 2>&1 | tee "${ARTIFACT_ROOT}/docker-build.log"
fi

printf '%s\n' "${IMAGE_NAME}" > "${ARTIFACT_ROOT}/image_ref.txt"
