#!/usr/bin/env bash
set -euo pipefail

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

# Source the colcon install directory built inside the project root.
if [[ -f "/workspace/tomato-harvest/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "/workspace/tomato-harvest/install/setup.bash"
  set -u
fi

# Base Isaac Sim image exposes HUB__ARGS__DETECT_ONLY=true, which prevents
# OmniHub from being launched and makes GUI startup appear hung.
unset HUB__ARGS__DETECT_ONLY || true

cd /workspace/tomato-harvest

if [[ "$#" -eq 0 ]]; then
  exec bash
fi

exec "$@"
