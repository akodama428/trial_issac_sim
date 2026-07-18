#!/usr/bin/env bash
set -euo pipefail

ARTIFACT_DIR="${CI_ARTIFACT_DIR:-/tmp/tomato-harvest-ci-artifacts}"
HEADLESS_STEPS="${CI_HEADLESS_STEPS:-900}"
E2E_TIMEOUT_SEC="${CI_E2E_TIMEOUT_SEC:-2400}"
CONTROLLER_LOG="${ARTIFACT_DIR}/franka_controller.log"
ROBOT_LOG="${ARTIFACT_DIR}/robot_node.log"
STACK_LOG="${ARTIFACT_DIR}/run_ros2_components.log"
BAG_DIR="${ARTIFACT_DIR}/home_divergence_bag"
BAG_LOG="${ARTIFACT_DIR}/bag_record.log"
BAG_PID=""

mkdir -p "${ARTIFACT_DIR}"
# self-hosted runnerのartifact volumeはrun間で再利用される。grep判定が過去runの
# metricを拾わないよう、今回の判定対象ログを必ず空にしてから開始する。
truncate -s 0 "${CONTROLLER_LOG}" "${ROBOT_LOG}" "${STACK_LOG}"

stop_rosbag() {
  if [[ -n "${BAG_PID}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    # 非同期jobはbashからSIGINTをignoreした状態で起動されるためSIGTERMでflushさせる。
    kill -TERM "${BAG_PID}"
    wait "${BAG_PID}" || true
  fi
}
trap stop_rosbag EXIT

if [[ "${CI_RECORD_HOME_DIVERGENCE_BAG:-}" == "1" ]]; then
  if [[ -e "${BAG_DIR}" ]]; then
    echo "Rosbag output already exists: ${BAG_DIR}" >&2
    exit 1
  fi
  ros2 bag record \
    --disable-keyboard-controls \
    --include-unpublished-topics \
    --output "${BAG_DIR}" \
    --storage mcap \
    --topics \
    /tomato_harvest/phase \
    /joint_trajectory_controller/joint_trajectory \
    /joint_trajectory_controller/controller_state \
    >"${BAG_LOG}" 2>&1 &
  BAG_PID=$!
fi

set +e
timeout --signal=INT "${E2E_TIMEOUT_SEC}" \
  ./scripts/run_ros2_components.sh \
  --isaac \
  --moveit \
  --rebuild \
  --headless \
  --headless-steps "${HEADLESS_STEPS}" \
  --grasp-mode "${CI_GRASP_MODE:-physics}" \
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
