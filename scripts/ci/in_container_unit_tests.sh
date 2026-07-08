#!/usr/bin/env bash
set -euo pipefail

ARTIFACT_DIR="${CI_ARTIFACT_DIR:-/tmp/tomato-harvest-ci-artifacts}"

mkdir -p "${ARTIFACT_DIR}"

pytest_status=0
colcon_status=0

set +e
python3 -m pytest \
  tests \
  src/tomato_harvest_sim/robot \
  src/tomato_harvest_sim/simulator \
  --junitxml "${ARTIFACT_DIR}/pytest-results.xml" \
  2>&1 | tee "${ARTIFACT_DIR}/pytest.log"
pytest_status=${PIPESTATUS[0]}

colcon test \
  --packages-select franka_ros2_control \
  --event-handlers console_direct+ \
  --return-code-on-test-failure \
  2>&1 | tee "${ARTIFACT_DIR}/colcon-test.log"
colcon_status=${PIPESTATUS[0]}

colcon test-result --verbose \
  2>&1 | tee "${ARTIFACT_DIR}/colcon-test-result.log"
set -e

if [[ "${pytest_status}" -ne 0 ]]; then
  exit "${pytest_status}"
fi

if [[ "${colcon_status}" -ne 0 ]]; then
  exit "${colcon_status}"
fi
