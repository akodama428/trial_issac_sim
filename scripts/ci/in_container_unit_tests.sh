#!/usr/bin/env bash
set -euo pipefail

ARTIFACT_DIR="${CI_ARTIFACT_DIR:-/tmp/tomato-harvest-ci-artifacts}"
COLCON_ROOT="${CI_COLCON_ROOT:-/tmp/tomato-harvest-ci-colcon}"
BUILD_BASE="${COLCON_ROOT}/build"
INSTALL_BASE="${COLCON_ROOT}/install"
LOG_BASE="${COLCON_ROOT}/log"

mkdir -p "${ARTIFACT_DIR}"
mkdir -p "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"

pytest_status=0
build_status=0
colcon_status=0

set +e
colcon build \
  --packages-up-to franka_ros2_control \
  --symlink-install \
  --event-handlers console_direct+ \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --log-base "${LOG_BASE}" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  2>&1 | tee "${ARTIFACT_DIR}/colcon-build.log"
build_status=${PIPESTATUS[0]}

if [[ -f "${INSTALL_BASE}/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${INSTALL_BASE}/setup.bash"
  set -u
fi

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
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --log-base "${LOG_BASE}" \
  --return-code-on-test-failure \
  2>&1 | tee "${ARTIFACT_DIR}/colcon-test.log"
colcon_status=${PIPESTATUS[0]}

colcon test-result \
  --test-result-base "${BUILD_BASE}" \
  --verbose \
  2>&1 | tee "${ARTIFACT_DIR}/colcon-test-result.log"
set -e

if [[ "${build_status}" -ne 0 ]]; then
  exit "${build_status}"
fi

if [[ "${pytest_status}" -ne 0 ]]; then
  exit "${pytest_status}"
fi

if [[ "${colcon_status}" -ne 0 ]]; then
  exit "${colcon_status}"
fi
