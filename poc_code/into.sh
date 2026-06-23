#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

IMAGE_NAME="${IMAGE_NAME:-tomato-harvest-sim}"
CONTAINER_NAME="${CONTAINER_NAME:-tomato-harvest-sim-debug}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
POC_HOST="${POC_HOST:-0.0.0.0}"
POC_PORT="${POC_PORT:-8080}"
ACCEPT_EULA="${ACCEPT_EULA:-Y}"
PRIVACY_CONSENT="${PRIVACY_CONSENT:-Y}"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/isaac-sim}"
DISPLAY_VALUE="${DISPLAY:-}"
XSOCK="/tmp/.X11-unix"
TMP_MOUNT_DIR="${TMP_MOUNT_DIR:-${SCRIPT_DIR}/.container_tmp}"

mkdir -p "${TMP_MOUNT_DIR}"

if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "Entering running container: ${CONTAINER_NAME}"
  exec docker exec -it "${CONTAINER_NAME}" bash
fi

if docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "Starting existing stopped container: ${CONTAINER_NAME}"
  docker start "${CONTAINER_NAME}" >/dev/null
  exec docker exec -it "${CONTAINER_NAME}" bash
fi

echo "Creating debug container: ${CONTAINER_NAME}"
DOCKER_ARGS=(
  -d
  --name "${CONTAINER_NAME}"
  --gpus all
  --network=host
  -e "ACCEPT_EULA=${ACCEPT_EULA}"
  -e "PRIVACY_CONSENT=${PRIVACY_CONSENT}"
  -e "ROS_DISTRO=${ROS_DISTRO}"
  -e "POC_HOST=${POC_HOST}"
  -e "POC_PORT=${POC_PORT}"
  -e "ISAAC_SIM_ROOT=${ISAAC_SIM_ROOT}"
  -e "TMPDIR=/tmp"
  -v "${TMP_MOUNT_DIR}:/tmp"
  -v "${SCRIPT_DIR}:/workspace/tomato-harvest"
  -w /workspace/tomato-harvest
  --entrypoint bash
)

if [[ -n "${DISPLAY_VALUE}" && -d "${XSOCK}" ]]; then
  DOCKER_ARGS+=(
    -e "DISPLAY=${DISPLAY_VALUE}"
    -e "QT_X11_NO_MITSHM=1"
    -v "${XSOCK}:${XSOCK}:rw"
  )
fi

docker run "${DOCKER_ARGS[@]}" \
  "${IMAGE_NAME}" \
  -lc 'if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then source "/opt/ros/${ROS_DISTRO}/setup.bash"; fi; trap : TERM INT; sleep infinity & wait' \
  >/dev/null

echo "Entering new container: ${CONTAINER_NAME}"
exec docker exec -it "${CONTAINER_NAME}" bash
