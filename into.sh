#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

IMAGE_NAME="${IMAGE_NAME:-tomato-harvest-sim}"
CONTAINER_NAME="${CONTAINER_NAME:-tomato-harvest-sim-debug}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ACCEPT_EULA="${ACCEPT_EULA:-Y}"
PRIVACY_CONSENT="${PRIVACY_CONSENT:-Y}"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/isaac-sim}"
DISPLAY_VALUE="${DISPLAY:-}"
XSOCK="/tmp/.X11-unix"
CACHE_ROOT="${CACHE_ROOT:-/tmp/tomato-harvest-sim-cache}"

ensure_cache_dirs() {
  mkdir -p \
    "${CACHE_ROOT}/ov-cache" \
    "${CACHE_ROOT}/ov-data" \
    "${CACHE_ROOT}/ov-logs" \
    "${CACHE_ROOT}/kit-cache"
}

has_legacy_tmp_bind() {
  docker inspect --format '{{range .Mounts}}{{println .Destination}}{{end}}' "$1" 2>/dev/null | grep -Fxq "/tmp"
}

has_legacy_hub_detect_only() {
  docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' "$1" 2>/dev/null | grep -Fxq "HUB__ARGS__DETECT_ONLY=true"
}

has_missing_gui_cache_binds() {
  local destinations
  destinations="$(docker inspect --format '{{range .Mounts}}{{println .Destination}}{{end}}' "$1" 2>/dev/null || true)"
  [[ "${destinations}" == *"/root/.cache/ov"* ]] || return 0
  [[ "${destinations}" == *"/root/.local/share/ov/data"* ]] || return 0
  [[ "${destinations}" == *"/root/.nvidia-omniverse/logs"* ]] || return 0
  [[ "${destinations}" == *"/isaac-sim/kit/cache"* ]] || return 0
  return 1
}

should_recreate_container() {
  has_legacy_tmp_bind "$1" || has_legacy_hub_detect_only "$1" || has_missing_gui_cache_binds "$1"
}

if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  if should_recreate_container "${CONTAINER_NAME}"; then
    echo "Recreating ${CONTAINER_NAME} because it uses stale Isaac Sim container settings."
    docker rm -f "${CONTAINER_NAME}" >/dev/null
  else
  echo "Entering running container: ${CONTAINER_NAME}"
  exec docker exec -it "${CONTAINER_NAME}" bash
  fi
fi

if docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  if should_recreate_container "${CONTAINER_NAME}"; then
    echo "Removing stopped container ${CONTAINER_NAME} because it uses stale Isaac Sim container settings."
    docker rm -f "${CONTAINER_NAME}" >/dev/null
  else
  echo "Starting existing stopped container: ${CONTAINER_NAME}"
  docker start "${CONTAINER_NAME}" >/dev/null
  exec docker exec -it "${CONTAINER_NAME}" bash
  fi
fi

echo "Creating debug container: ${CONTAINER_NAME}"
ensure_cache_dirs
DOCKER_ARGS=(
  -d
  --name "${CONTAINER_NAME}"
  --gpus all
  --network=host
  -e "ACCEPT_EULA=${ACCEPT_EULA}"
  -e "PRIVACY_CONSENT=${PRIVACY_CONSENT}"
  -e "ROS_DISTRO=${ROS_DISTRO}"
  -e "ISAAC_SIM_ROOT=${ISAAC_SIM_ROOT}"
  -e "HUB__ARGS__DETECT_ONLY=false"
  -e "PYTHONPATH=/workspace/tomato-harvest/src"
  -v "${CACHE_ROOT}/ov-cache:/root/.cache/ov"
  -v "${CACHE_ROOT}/ov-data:/root/.local/share/ov/data"
  -v "${CACHE_ROOT}/ov-logs:/root/.nvidia-omniverse/logs"
  -v "${CACHE_ROOT}/kit-cache:/isaac-sim/kit/cache"
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
  -lc 'set +u; if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then source "/opt/ros/${ROS_DISTRO}/setup.bash"; fi; set -u; trap : TERM INT; sleep infinity & wait' \
  >/dev/null

echo "Entering new container: ${CONTAINER_NAME}"
exec docker exec -it "${CONTAINER_NAME}" bash
