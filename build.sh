#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

IMAGE_NAME="${IMAGE_NAME:-tomato-harvest-sim}"
DOCKERFILE_PATH="${DOCKERFILE_PATH:-docker/Dockerfile}"
ISAAC_SIM_IMAGE="${ISAAC_SIM_IMAGE:-nvcr.io/nvidia/isaac-sim:6.0.0}"

echo "Building image: ${IMAGE_NAME}"
echo "Dockerfile: ${DOCKERFILE_PATH}"
echo "Base Isaac Sim image: ${ISAAC_SIM_IMAGE}"

exec docker build \
  --build-arg "ISAAC_SIM_IMAGE=${ISAAC_SIM_IMAGE}" \
  -t "${IMAGE_NAME}" \
  -f "${DOCKERFILE_PATH}" \
  .
