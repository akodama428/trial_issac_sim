#!/usr/bin/env bash
set -euo pipefail

ARTIFACT_DIR="${CI_ARTIFACT_DIR:-/tmp/tomato-harvest-ci-artifacts}"
HEADLESS_STEPS="${CI_HEADLESS_STEPS:-900}"
E2E_TIMEOUT_SEC="${CI_E2E_TIMEOUT_SEC:-2400}"
CONTROLLER_LOG="${ARTIFACT_DIR}/franka_controller.log"
ROBOT_LOG="${ARTIFACT_DIR}/robot_node.log"
STACK_LOG="${ARTIFACT_DIR}/run_ros2_components.log"

mkdir -p "${ARTIFACT_DIR}"
# self-hosted runnerのartifact volumeはrun間で再利用される。grep判定が過去runの
# metricを拾わないよう、今回の判定対象ログを必ず空にしてから開始する。
truncate -s 0 "${CONTROLLER_LOG}" "${ROBOT_LOG}" "${STACK_LOG}"

set +e
timeout --signal=INT "${E2E_TIMEOUT_SEC}" \
  ./scripts/run_ros2_components.sh \
  --isaac \
  --moveit \
  --headless \
  --headless-steps "${HEADLESS_STEPS}" \
  --auto-start \
  --controller-log "${CONTROLLER_LOG}" \
  --robot-log "${ROBOT_LOG}" \
  2>&1 | tee "${STACK_LOG}"
status=${PIPESTATUS[0]}
set -e

if [[ "${status}" -eq 124 ]]; then
  echo "E2E timed out after ${E2E_TIMEOUT_SEC} seconds." >&2
  exit 1
fi

if [[ "${status}" -ne 0 ]]; then
  echo "run_ros2_components.sh exited with status ${status}." >&2
  exit "${status}"
fi

if ! grep -q "Headless simulator node setup completed." "${STACK_LOG}"; then
  echo "Headless Isaac Sim completion marker was not found." >&2
  exit 1
fi

if ! grep -Eq 'Phase: returning_home .* complete' "${ROBOT_LOG}"; then
  echo "Harvest cycle completion marker was not found in robot log." >&2
  exit 1
fi

if grep -Eq 'Phase: .* failed' "${ROBOT_LOG}"; then
  echo "Failure phase transition was detected in robot log." >&2
  exit 1
fi

# JTC feedback由来の実行中tracking errorがServo adapterから周期配信されることを固定する。
if ! grep -Eq 'execution_status \{"status":"running","tracking_error_rad":[0-9]' "${CONTROLLER_LOG}"; then
  echo "Periodic live tracking-error status was not observed." >&2
  exit 1
fi
