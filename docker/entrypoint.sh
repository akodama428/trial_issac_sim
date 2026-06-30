#!/usr/bin/env bash
set -euo pipefail

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

# Source the franka_ros2_control colcon workspace if built.
if [[ -f "/workspace/ros2_ws/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "/workspace/ros2_ws/install/setup.bash"
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
