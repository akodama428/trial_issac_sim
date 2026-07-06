#!/usr/bin/env bash
# run_ros2_components.sh
#
# 新 ROS2 コンポーネントアーキテクチャを 1 コマンドで起動する。
#
# 起動順:
#   1. C++ パッケージをビルド（未ビルドまたは --rebuild 指定時）
#   2. franka_ros2_control (controller_manager + JTC + JSB) を background 起動
#   3. controller_manager の起動待機とコントローラーのスポウン
#   4. MoveIt2 move_group を background 起動（--moveit 指定時のみ）
#   5. tomato_harvest_robot_node を background 起動
#   6. tomato_harvest_simulator_node（または Isaac Sim）を foreground 起動
#   7. 終了時に全 background プロセスをクリーンアップ
#
# 使い方:
#   ./scripts/run_ros2_components.sh [オプション]
#
# オプション:
#   --isaac                    Isaac Sim 統合モードで起動（デフォルト: toy physics）
#   --headless                 Isaac Sim をヘッドレスモードで起動（--isaac 時のみ有効）
#   --headless-steps N         ヘッドレス実行ステップ数（デフォルト: 64）
#   --auto-start               起動後に自動で Start コマンドを送信
#   --rebuild                  C++ パッケージを強制再ビルドする
#   --moveit                   MoveIt2 move_group を起動する（GetMotionPlan サービス提供）
#   --ros-distro <distro>      ROS2 ディストリビューション（デフォルト: 自動検出）
#   --ws-dir <path>            colcon ワークスペースのパス（デフォルト: /tmp/franka_ros2_ws）
#   --controller-log <path>    C++ コントローラのログ出力先（デフォルト: /tmp/franka_controller.log）
#   --robot-log <path>         robot_node のログ出力先（デフォルト: /tmp/robot_node.log）

set -euo pipefail

# ---------------------------------------------------------------------------- #
# デフォルト値
# ---------------------------------------------------------------------------- #
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PKG_SRC="${REPO_ROOT}/src/franka_ros2_control"

# ROS_DISTRO: 環境変数優先、なければインストール済みディストリを自動検出
if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    ROS_DISTRO=jazzy
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    ROS_DISTRO=humble
  else
    ROS_DISTRO=jazzy  # フォールバック（require_ros でエラー表示）
  fi
fi

WS_DIR="${FRANKA_ROS2_WS:-/tmp/franka_ros2_ws}"
CONTROLLER_LOG="${FRANKA_CONTROLLER_LOG:-/tmp/franka_controller.log}"
ROBOT_LOG="/tmp/robot_node.log"
REBUILD=false
USE_ISAAC=false
AUTO_START=false
USE_MOVEIT=false
HEADLESS_ARGS=()

# ---------------------------------------------------------------------------- #
# 引数解析
# ---------------------------------------------------------------------------- #
while [[ $# -gt 0 ]]; do
  case "$1" in
    --isaac)
      USE_ISAAC=true
      shift
      ;;
    --auto-start)
      AUTO_START=true
      shift
      ;;
    --rebuild)
      REBUILD=true
      shift
      ;;
    --ros-distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --ws-dir)
      WS_DIR="$2"
      shift 2
      ;;
    --controller-log)
      CONTROLLER_LOG="$2"
      shift 2
      ;;
    --robot-log)
      ROBOT_LOG="$2"
      shift 2
      ;;
    --headless)
      HEADLESS_ARGS+=(--headless)
      shift
      ;;
    --headless-steps)
      HEADLESS_ARGS+=(--headless-steps "$2")
      shift 2
      ;;
    --moveit)
      USE_MOVEIT=true
      shift
      ;;
    *)
      echo "[ERROR] 不明なオプション: $1" >&2
      exit 1
      ;;
  esac
done

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WS_SETUP="${WS_DIR}/install/setup.bash"

# ---------------------------------------------------------------------------- #
# ヘルパー
# ---------------------------------------------------------------------------- #
log() { echo "[run_ros2_components] $*" >&2; }

require_ros() {
  if [[ ! -f "${ROS_SETUP}" ]]; then
    log "ERROR: ROS2 setup が見つかりません: ${ROS_SETUP}"
    log "       --ros-distro で正しいディストリビューションを指定してください。"
    exit 1
  fi
  # shellcheck disable=SC1090
  set +u; source "${ROS_SETUP}"; set -u
}

source_ws() {
  if [[ -f "${WS_SETUP}" ]]; then
    # shellcheck disable=SC1090
    set +u; source "${WS_SETUP}"; set -u
  fi
}

# ---------------------------------------------------------------------------- #
# 1. C++ パッケージのビルド
# ---------------------------------------------------------------------------- #
ensure_build_tools() {
  if ! command -v g++ &>/dev/null || ! command -v cmake &>/dev/null; then
    log "C/C++ コンパイラが未インストールです。apt でインストールします..."
    apt-get update -q && apt-get install -y --no-install-recommends build-essential cmake
  fi
}

build_cpp_package() {
  log "--- C++ パッケージのビルド ---"
  ensure_build_tools
  require_ros
  mkdir -p "${WS_DIR}/src"

  local link_target="${WS_DIR}/src/franka_ros2_control"
  if [[ ! -L "${link_target}" ]]; then
    ln -s "${PKG_SRC}" "${link_target}"
    log "  symlink 作成: ${link_target} -> ${PKG_SRC}"
  fi

  cd "${WS_DIR}"
  colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-select franka_ros2_control \
    2>&1 | tee "${CONTROLLER_LOG}.build"
  cd "${REPO_ROOT}"
  log "  ビルド完了"
}

needs_build() {
  [[ "${REBUILD}" == "true" ]] \
    || [[ ! -f "${WS_SETUP}" ]] \
    || [[ "${PKG_SRC}/src/isaac_sim_hardware_interface.cpp" \
          -nt "${WS_DIR}/build/franka_ros2_control/libfranka_ros2_control.so" ]] 2>/dev/null
}

if needs_build; then
  build_cpp_package
else
  log "C++ パッケージは最新です（スキップ）。再ビルドするには --rebuild を指定してください。"
fi

# ---------------------------------------------------------------------------- #
# 2. background プロセスの管理
# ---------------------------------------------------------------------------- #
require_ros
source_ws

BG_PIDS=()

start_bg() {
  local label="$1"
  shift
  log "  ${label} 起動中..."
  "$@" &
  local pid=$!
  BG_PIDS+=("${pid}")
  log "  ${label} PID=${pid}"
}

cleanup() {
  log "--- クリーンアップ ---"
  for pid in "${BG_PIDS[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      log "  PID ${pid} を終了"
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  # ros2_control / spawner の残留プロセスも念のため終了
  pkill -f "ros2_control_node" 2>/dev/null || true
  pkill -f "spawner joint_" 2>/dev/null || true
  log "クリーンアップ完了"
}
trap cleanup EXIT INT TERM

log "--- 既存プロセスのクリーンアップ ---"
pkill -f "ros2_control_node" 2>/dev/null || true
pkill -f "tomato_harvest_sim\." 2>/dev/null || true  # 全 tomato_harvest_sim ノード
pkill -f "run_harvest_viewer" 2>/dev/null || true
pkill -f "moveit_ros_move_group" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
sleep 3

# ---------------------------------------------------------------------------- #
# 3. franka_ros2_control 起動（background）
# ---------------------------------------------------------------------------- #
log "--- franka_ros2_control 起動 ---"
log "  ログ: ${CONTROLLER_LOG}"

URDF_PATH="${PKG_SRC}/config/franka_ros2_control.urdf"
CONTROLLERS_YAML="${PKG_SRC}/config/franka_controllers.yaml"

if [[ ! -f "${URDF_PATH}" ]]; then
  log "ERROR: URDF が見つかりません: ${URDF_PATH}"
  exit 1
fi

ROBOT_DESCRIPTION="$(cat "${URDF_PATH}")"

start_bg "robot_state_publisher" \
  ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p "robot_description:=${ROBOT_DESCRIPTION}" \
  >> "${CONTROLLER_LOG}" 2>&1
sleep 1

start_bg "ros2_control_node" \
  ros2 run controller_manager ros2_control_node \
  --ros-args --params-file "${CONTROLLERS_YAML}" \
  >> "${CONTROLLER_LOG}" 2>&1

# controller_manager の起動待機
log "  controller_manager 起動待機中..."
for i in $(seq 1 30); do
  if ros2 service list 2>/dev/null | grep -q "/controller_manager/list_controllers"; then
    log "  controller_manager 起動確認"
    break
  fi
  sleep 0.5
  if [[ "${i}" -eq 30 ]]; then
    log "ERROR: controller_manager 起動タイムアウト（15秒）。ログ: ${CONTROLLER_LOG}"
    exit 1
  fi
done

# コントローラースポウン
ros2 run controller_manager spawner joint_state_broadcaster \
  --controller-manager /controller_manager \
  >> "${CONTROLLER_LOG}" 2>&1 &

ros2 run controller_manager spawner joint_trajectory_controller \
  --controller-manager /controller_manager \
  >> "${CONTROLLER_LOG}" 2>&1 &

log "  コントローラースポウン完了待機中..."
for i in $(seq 1 40); do
  ACTIVE_COUNT=$(ros2 control list_controllers 2>/dev/null | grep -cE '\bactive\b' || true)
  if [[ "${ACTIVE_COUNT}" -ge 2 ]]; then
    log "  コントローラー active 確認（count: ${ACTIVE_COUNT}）"
    break
  fi
  sleep 0.5
  if [[ "${i}" -eq 40 ]]; then
    log "WARN: コントローラー active タイムアウト。起動を続行します。"
    ros2 control list_controllers 2>/dev/null || true
  fi
done

# ---------------------------------------------------------------------------- #
# 4. MoveIt2 move_group 起動（optional, background）
# ---------------------------------------------------------------------------- #
if [[ "${USE_MOVEIT}" == "true" ]]; then
  log "--- MoveIt2 move_group 起動 ---"
  log "  ログ: ${CONTROLLER_LOG}.move_group"
  start_bg "move_group" \
    ros2 launch franka_ros2_control move_group.launch.py \
    >> "${CONTROLLER_LOG}.move_group" 2>&1

  log "  move_group 起動待機中（/plan_kinematic_path サービス）..."
  for i in $(seq 1 60); do
    if ros2 service list 2>/dev/null | grep -q "/plan_kinematic_path"; then
      log "  move_group 起動確認"
      break
    fi
    sleep 0.5
    if [[ "${i}" -eq 60 ]]; then
      log "WARN: move_group 起動タイムアウト（30秒）。起動を続行します。"
      log "      ログ: ${CONTROLLER_LOG}.move_group"
    fi
  done
fi

# ---------------------------------------------------------------------------- #
# 5. tomato_harvest ロボットノード群 起動（background）
# ---------------------------------------------------------------------------- #
log "--- ロボットノード群 起動 ---"
log "  ログ: ${ROBOT_LOG}"

# 各ノードのログを同じファイルへ集約（識別のためノード名を先頭に付加）
PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
start_bg "tomato_detector_node" \
  python3 -m tomato_harvest_sim.robot.perception \
  >> "${ROBOT_LOG}" 2>&1

PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
start_bg "behavior_planner_node" \
  python3 -m tomato_harvest_sim.robot.behavior_planner \
  >> "${ROBOT_LOG}" 2>&1

PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
start_bg "trajectory_planner_node" \
  python3 -m tomato_harvest_sim.robot.motion_planner \
  >> "${ROBOT_LOG}" 2>&1

PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
start_bg "trajectory_monitor_node" \
  python3 -m tomato_harvest_sim.robot.trajectory_monitor_node \
  >> "${ROBOT_LOG}" 2>&1

PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
start_bg "motion_command_node" \
  python3 -m tomato_harvest_sim.robot.motion_command_node \
  >> "${ROBOT_LOG}" 2>&1

PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
start_bg "motion_command_executor_node" \
  python3 -m tomato_harvest_sim.robot.motion_command_executor_node \
  >> "${ROBOT_LOG}" 2>&1

# ---------------------------------------------------------------------------- #
# 6. auto-start コマンド送信タスク（background）
# ---------------------------------------------------------------------------- #
if [[ "${AUTO_START}" == "true" ]]; then
  # --moveit 使用時は move_group 起動待機（最大60秒）があるため、
  # simulator_node が起動するまで最大 120 秒待機する。
  _AUTO_START_TIMEOUT=120
  (
    log "  auto-start: scene_snapshot 受信待機中（最大 ${_AUTO_START_TIMEOUT} 秒）..."
    for i in $(seq 1 "${_AUTO_START_TIMEOUT}"); do
      if ros2 topic list 2>/dev/null | grep -q "scene_snapshot"; then
        sleep 2  # シミュレータ安定待ち
        log "  auto-start: 'start' コマンド送信"
        ros2 topic pub --once /tomato_harvest/control std_msgs/msg/String "data: 'start'" >/dev/null 2>&1
        break
      fi
      sleep 1
      if [[ "${i}" -eq "${_AUTO_START_TIMEOUT}" ]]; then
        log "  auto-start: タイムアウト（simulator が起動しませんでした）"
      fi
    done
  ) &
  BG_PIDS+=($!)
fi

# ---------------------------------------------------------------------------- #
# 7. simulator_node 起動（foreground）
# ---------------------------------------------------------------------------- #
if [[ "${USE_ISAAC}" == "true" ]]; then
  log "--- Isaac Sim 起動 (SimulatorNode 統合モード) ---"
  log "  オプション: ${HEADLESS_ARGS[*]:-（なし）}"
  PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
  "${REPO_ROOT}/python.sh" \
    "${REPO_ROOT}/scripts/run_harvest_viewer.py" \
    ${HEADLESS_ARGS[@]+"${HEADLESS_ARGS[@]}"}
else
  log "--- simulator_node 起動（toy physics モード）---"
  log "  Isaac Sim を使う場合は --isaac オプションを指定してください。"

  PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
  python3 -m tomato_harvest_sim.simulator.simulator_node
fi

log "simulator 終了"
