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

# Issue #38: injection専用値ではなく、JTC feedback由来の実行中tracking errorが
# executorから周期配信されていることを通常CIでも固定する。
if ! grep -Eq 'execution_status \{"status":"running","tracking_error_rad":[0-9]' "${CONTROLLER_LOG}"; then
  echo "Periodic live tracking-error status was not observed." >&2
  exit 1
fi

if [[ -n "${TOMATO_HARVEST_INJECT_LOCAL_PLAN_PHASES:-}" ]]; then
  IFS=',' read -ra LOCAL_PLAN_PHASES <<< "${TOMATO_HARVEST_INJECT_LOCAL_PLAN_PHASES}"
  for phase in "${LOCAL_PLAN_PHASES[@]}"; do
    phase="$(echo "${phase}" | tr -d '[:space:]')"
    [[ -z "${phase}" ]] && continue
    if grep -Eq "\"event\": \"local_plan_published\".*\"phase\": \"${phase}\"" "${ROBOT_LOG}"; then
      if ! grep -Eq "\"event\": \"plan_adopted\".*\"planned_from_phase\": \"${phase}\".*\"producer_kind\": \"local_planner\"" "${ROBOT_LOG}"; then
        echo "Local correction plan was not adopted through arbitration for phase ${phase}." >&2
        exit 1
      fi
      continue
    fi
    if grep -Eq "\"event\": \"local_plan_skipped\".*\"phase\": \"${phase}\".*\"reason\": \"unsafe_or_unavailable_candidate\"" "${ROBOT_LOG}"; then
      echo "Local correction was safety-rejected in phase ${phase}; accepting safe fallback."
      continue
    fi
    echo "Local correction was neither published nor safety-rejected in phase ${phase}." >&2
    exit 1
  done
fi

if [[ -n "${TOMATO_HARVEST_INJECT_SUFFIX_REPLAN_PHASES:-}" ]]; then
  IFS=',' read -ra INJECTION_PHASES <<< "${TOMATO_HARVEST_INJECT_SUFFIX_REPLAN_PHASES}"
  for phase in "${INJECTION_PHASES[@]}"; do
    phase="$(echo "${phase}" | tr -d '[:space:]')"
    [[ -z "${phase}" ]] && continue
    if ! grep -Eq "\"event\": \"suffix_e2e_disturbance_injected\".*\"phase\": \"${phase}\"" "${ROBOT_LOG}"; then
      echo "Suffix E2E disturbance was not injected in phase ${phase}." >&2
      exit 1
    fi
    if ! grep -Eq "\"event\": \"hybrid_event_routed\".*\"phase\": \"${phase}\".*\"route\": \"local\".*\"trigger\": \"tracking_error\"" "${ROBOT_LOG}"; then
      echo "Tracking-error event was not routed exclusively to the local planner in phase ${phase}." >&2
      exit 1
    fi
    if grep -Eq "\"event\": \"suffix_replan_completed\".*\"phase\": \"${phase}\".*\"trigger\": \"tracking_error\"" "${ROBOT_LOG}"; then
      echo "Tracking error incorrectly started the global suffix planner in phase ${phase}." >&2
      exit 1
    fi
  done
fi
