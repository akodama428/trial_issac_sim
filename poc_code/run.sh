#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

IMAGE_NAME="${IMAGE_NAME:-tomato-harvest-sim}"
CONTAINER_NAME="${CONTAINER_NAME:-tomato-harvest-sim}"
POC_RUNTIME="${POC_RUNTIME:-isaac}"
POC_SCENARIO="${POC_SCENARIO:-success}"
POC_HOST="${POC_HOST:-0.0.0.0}"
POC_PORT="${POC_PORT:-8080}"
POC_HEADLESS="${POC_HEADLESS:-0}"
POC_TEST_MODE="${POC_TEST_MODE:-0}"
POC_CAMERA_VIEW="${POC_CAMERA_VIEW:-fixed}"
ACCEPT_EULA="${ACCEPT_EULA:-Y}"
PRIVACY_CONSENT="${PRIVACY_CONSENT:-Y}"
TMP_MOUNT_DIR="${TMP_MOUNT_DIR:-${SCRIPT_DIR}/.container_tmp}"
DISPLAY_VALUE="${DISPLAY:-}"
XSOCK="/tmp/.X11-unix"

mkdir -p "${TMP_MOUNT_DIR}"

echo "Running image: ${IMAGE_NAME}"
echo "Runtime mode: ${POC_RUNTIME}"
echo "Scenario: ${POC_SCENARIO}"
echo "Headless: ${POC_HEADLESS}"
echo "Initial camera view: ${POC_CAMERA_VIEW}"
echo "Container /tmp mount: ${TMP_MOUNT_DIR}"

DOCKER_ARGS=(
  --rm
  --name "${CONTAINER_NAME}"
  --gpus all
  --network=host
  -e "ACCEPT_EULA=${ACCEPT_EULA}"
  -e "PRIVACY_CONSENT=${PRIVACY_CONSENT}"
  -e "POC_RUNTIME=${POC_RUNTIME}"
  -e "POC_SCENARIO=${POC_SCENARIO}"
  -e "POC_HOST=${POC_HOST}"
  -e "POC_PORT=${POC_PORT}"
  -e "POC_HEADLESS=${POC_HEADLESS}"
  -e "POC_TEST_MODE=${POC_TEST_MODE}"
  -e "POC_CAMERA_VIEW=${POC_CAMERA_VIEW}"
  -e "TMPDIR=/tmp"
  -v "${TMP_MOUNT_DIR}:/tmp"
)

if [[ "${POC_RUNTIME}" == "isaac" && "${POC_HEADLESS}" != "1" ]]; then
  if [[ -z "${DISPLAY_VALUE}" || ! -d "${XSOCK}" ]]; then
    echo "DISPLAY is not set. Use an X11/VNC session, set POC_HEADLESS=1 for headless validation, or debug via ./into.sh." >&2
    exit 2
  fi
  echo "3DView mode: native Isaac Sim window"
  DOCKER_ARGS+=(
    -e "DISPLAY=${DISPLAY_VALUE}"
    -e "QT_X11_NO_MITSHM=1"
    -v "${XSOCK}:${XSOCK}:rw"
  )
fi

exec docker run "${DOCKER_ARGS[@]}" "${IMAGE_NAME}"
