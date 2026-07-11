#!/usr/bin/env bash
set -euo pipefail

ARTIFACT_DIR="${CI_ARTIFACT_DIR:-/tmp/tomato-harvest-ci-artifacts}"
HEADLESS_STEPS="${CI_HEADLESS_STEPS:-900}"
E2E_TIMEOUT_SEC="${CI_E2E_TIMEOUT_SEC:-2400}"
CONTROLLER_LOG="${ARTIFACT_DIR}/franka_controller.log"
ROBOT_LOG="${ARTIFACT_DIR}/robot_node.log"
STACK_LOG="${ARTIFACT_DIR}/run_ros2_components.log"

mkdir -p "${ARTIFACT_DIR}"

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

if [[ -n "${TOMATO_HARVEST_INJECT_LOCAL_PLAN_PHASES:-}" ]]; then
  IFS=',' read -ra LOCAL_PLAN_PHASES <<< "${TOMATO_HARVEST_INJECT_LOCAL_PLAN_PHASES}"
  for phase in "${LOCAL_PLAN_PHASES[@]}"; do
    phase="$(echo "${phase}" | tr -d '[:space:]')"
    [[ -z "${phase}" ]] && continue
    if ! grep -Eq "\"event\": \"local_plan_published\".*\"phase\": \"${phase}\"" "${ROBOT_LOG}"; then
      echo "Local correction plan was not published in phase ${phase}." >&2
      exit 1
    fi
    if ! grep -Eq "\"event\": \"plan_adopted\".*\"planned_from_phase\": \"${phase}\".*\"producer_kind\": \"local_planner\"" "${ROBOT_LOG}"; then
      echo "Local correction plan was not adopted through arbitration for phase ${phase}." >&2
      exit 1
    fi
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
    if ! grep -Eq "\"event\": \"suffix_replan_completed\".*\"phase\": \"${phase}\".*\"success\": true" "${ROBOT_LOG}"; then
      echo "Successful real MoveIt suffix replan metric was not found for phase ${phase}." >&2
      exit 1
    fi
  done
fi
