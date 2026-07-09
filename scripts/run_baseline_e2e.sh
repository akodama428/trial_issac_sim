#!/usr/bin/env bash
# ベースライン E2E を指定回数連続実行する（Step 0 検証用）。
# コンテナ内での実行を想定。各 run のログを /tmp/sim_runN.log / /tmp/robot_runN.log に分離する。
#
# 使用例（コンテナ内）:
#   TOMATO_HARVEST_DEBUG_PHYSICS_GRASP=1 ./scripts/run_baseline_e2e.sh 2 5
set -euo pipefail

START_RUN="${1:?start run index}"
END_RUN="${2:?end run index}"
HEADLESS_STEPS="${HEADLESS_STEPS:-12000}"

cd "$(dirname "${BASH_SOURCE[0]}")/.."

for run in $(seq "${START_RUN}" "${END_RUN}"); do
  echo "=== baseline run ${run} start: $(date -u +%H:%M:%S) ==="
  ./scripts/run_ros2_components.sh \
    --isaac --moveit --headless --headless-steps "${HEADLESS_STEPS}" --auto-start \
    --robot-log "/tmp/robot_run${run}.log" \
    > "/tmp/sim_run${run}.log" 2>&1 || echo "run ${run} exited non-zero"
  phases=$(grep -c 'Phase:' "/tmp/robot_run${run}.log" 2>/dev/null || echo 0)
  complete=$(grep -c 'Phase: .* complete' "/tmp/robot_run${run}.log" 2>/dev/null || echo 0)
  echo "=== baseline run ${run} done: phases=${phases} complete=${complete} ==="
  sleep 5
done
