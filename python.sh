#!/usr/bin/env bash
set -euo pipefail

if [ -f /opt/ros/jazzy/setup.bash ]; then
  # MoveIt2 / rclpy を有効化したうえで Isaac Sim Python を起動する。
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi

exec /isaac-sim/python.sh "$@"
