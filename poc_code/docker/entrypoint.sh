#!/usr/bin/env bash
set -euo pipefail

ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/isaac-sim}"
ISAAC_PYTHON="${ISAAC_SIM_ROOT}/python.sh"

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

cd /workspace/tomato-harvest

runtime_args=(--mode "${POC_RUNTIME:-isaac}")

if [[ "${POC_HEADLESS:-0}" == "1" ]]; then
  runtime_args+=(--headless)
fi

if [[ "${POC_TEST_MODE:-0}" == "1" ]]; then
  runtime_args+=(--test)
fi

if [[ -n "${POC_CAMERA_VIEW:-}" ]]; then
  runtime_args+=(--camera-view "${POC_CAMERA_VIEW}")
fi

if [[ -x "${ISAAC_PYTHON}" ]]; then
  exec "${ISAAC_PYTHON}" scripts/run_poc.py "${runtime_args[@]}"
fi

runtime_args+=(--host "${POC_HOST:-0.0.0.0}" --port "${POC_PORT:-8080}")
exec python3 scripts/run_poc.py "${runtime_args[@]}"
