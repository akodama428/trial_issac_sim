#!/usr/bin/env bash
# run_with_ros2_control.sh
#
# 新アーキテクチャ（C++ ros2_control + Isaac Sim）を 1 コマンドで起動する。
#
# 起動順:
#   1. C++ パッケージをビルド（未ビルドまたは --rebuild 指定時）
#   2. franka_ros2_control ノード（JointTrajectoryController）を background 起動
#   3. Isaac Sim を ros2_control バックエンドで起動（foreground）
#   4. Isaac Sim 終了時に background プロセスを自動クリーンアップ
#
# 使い方:
#   ./scripts/run_with_ros2_control.sh [オプション]
#
# オプション:
#   --headless                 Isaac Sim をヘッドレスモードで起動
#   --headless-steps N         ヘッドレス実行ステップ数（デフォルト: 64）
#   --auto-start               起動後に自動で Start を押す
#   --grasp-mode <success|failure>  グラスプモード（デフォルト: success）
#   --transport <in_memory|ros2|auto>  ブリッジ種別（デフォルト: ros2）
#   --rebuild                  C++ パッケージを強制再ビルドする
#   --ros-distro <distro>      ROS2 ディストリビューション（デフォルト: jazzy）
#   --ws-dir <path>            colcon ワークスペースのパス（デフォルト: /tmp/franka_ros2_ws）
#   --controller-log <path>    C++ コントローラのログ出力先（デフォルト: /tmp/franka_controller.log）

set -euo pipefail

# ---------------------------------------------------------------------------- #
# デフォルト値
# ---------------------------------------------------------------------------- #
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PKG_SRC="${REPO_ROOT}/packages/franka_ros2_control"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS_DIR="${FRANKA_ROS2_WS:-/tmp/franka_ros2_ws}"
CONTROLLER_LOG="${FRANKA_CONTROLLER_LOG:-/tmp/franka_controller.log}"
REBUILD=false
ISAAC_ARGS=()

# ---------------------------------------------------------------------------- #
# 引数解析（Isaac Sim 用フラグはそのまま転送する）
# ---------------------------------------------------------------------------- #
while [[ $# -gt 0 ]]; do
  case "$1" in
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
    *)
      ISAAC_ARGS+=("$1")
      shift
      ;;
  esac
done

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WS_SETUP="${WS_DIR}/install/setup.bash"

# ---------------------------------------------------------------------------- #
# ヘルパー
# ---------------------------------------------------------------------------- #
log() { echo "[run_with_ros2_control] $*" >&2; }

require_ros() {
  if [[ ! -f "${ROS_SETUP}" ]]; then
    log "ERROR: ROS2 setup not found: ${ROS_SETUP}"
    log "       --ros-distro で正しいディストリビューションを指定してください。"
    exit 1
  fi
  # shellcheck disable=SC1090
  set +u; source "${ROS_SETUP}"; set -u
}

# ---------------------------------------------------------------------------- #
# 1. C++ パッケージのビルド
# ---------------------------------------------------------------------------- #
ensure_build_tools() {
  if ! command -v g++ &>/dev/null || ! command -v cmake &>/dev/null; then
    log "C/C++ コンパイラが未インストールです。apt でインストールします..."
    apt-get update -q && apt-get install -y --no-install-recommends build-essential cmake
    log "  build-essential / cmake インストール完了"
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

  log "  ビルド完了: ${WS_DIR}/install"
  cd "${REPO_ROOT}"
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
# 2. C++ ros2_control ノードを background 起動
# ---------------------------------------------------------------------------- #
require_ros
# shellcheck disable=SC1090
set +u; source "${WS_SETUP}"; set -u

log "--- 既存の ros2_control プロセスをクリーンアップ ---"
pkill -f "ros2_control_node" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
sleep 1

log "--- C++ ros2_control ノード起動 ---"
log "  ログ: ${CONTROLLER_LOG}"

URDF_PATH="${PKG_SRC}/config/franka_ros2_control.urdf"
CONTROLLERS_YAML="${PKG_SRC}/config/franka_controllers.yaml"

if [[ ! -f "${URDF_PATH}" ]]; then
  log "ERROR: URDF が見つかりません: ${URDF_PATH}"
  exit 1
fi

ROBOT_DESCRIPTION="$(cat "${URDF_PATH}")"

# background プロセス PID 一覧（クリーンアップ用）
rsp_pid=""
ros2_control_pid=""
cleanup() {
  log "クリーンアップ中..."
  for pid_var in ros2_control_pid rsp_pid; do
    local pid="${!pid_var}"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      log "  PID ${pid} を終了します"
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  log "完了"
}
trap cleanup EXIT INT TERM

# ros2_control 4.x 以降は robot_description をトピック経由で受け取る。
# robot_state_publisher を使って /robot_description を latched トピックとして発行する。
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p "robot_description:=${ROBOT_DESCRIPTION}" \
  >> "${CONTROLLER_LOG}" 2>&1 &
rsp_pid=$!
log "  robot_state_publisher PID=${rsp_pid}"
sleep 1  # /robot_description トピックが利用可能になるまで待機

ros2 run controller_manager ros2_control_node \
  --ros-args \
  --params-file "${CONTROLLERS_YAML}" \
  >> "${CONTROLLER_LOG}" 2>&1 &
ros2_control_pid=$!
log "  ros2_control_node PID=${ros2_control_pid}"

# コントローラースポウン（controller_manager が起動するまで待機）
log "  controller_manager の起動待機中..."
for i in $(seq 1 30); do
  if ros2 service list 2>/dev/null | grep -q "/controller_manager/list_controllers"; then
    break
  fi
  sleep 0.5
  if ! kill -0 "${ros2_control_pid}" 2>/dev/null; then
    log "ERROR: ros2_control_node が異常終了しました。ログ: ${CONTROLLER_LOG}"
    exit 1
  fi
  if [[ "${i}" -eq 30 ]]; then
    log "ERROR: controller_manager の起動タイムアウト（15秒）"
    exit 1
  fi
done
log "  controller_manager 起動確認"

# joint_state_broadcaster を起動
ros2 run controller_manager spawner joint_state_broadcaster \
  --controller-manager /controller_manager \
  >> "${CONTROLLER_LOG}" 2>&1 &
log "  joint_state_broadcaster スポウン"

# joint_trajectory_controller を起動
ros2 run controller_manager spawner joint_trajectory_controller \
  --controller-manager /controller_manager \
  >> "${CONTROLLER_LOG}" 2>&1 &
log "  joint_trajectory_controller スポウン"

# 両コントローラーが active になるまで待機（最大20秒）
log "  コントローラー active 待機中..."
for i in $(seq 1 40); do
  ACTIVE_COUNT=$(ros2 control list_controllers 2>/dev/null | grep -cE '\bactive\b' || true)
  if [[ "${ACTIVE_COUNT}" -ge 2 ]]; then
    log "  コントローラー起動確認（active: ${ACTIVE_COUNT}）"
    break
  fi
  sleep 0.5
  if [[ "${i}" -eq 40 ]]; then
    log "WARN: コントローラーの active 確認タイムアウト（20秒）。起動を続行します。"
    ros2 control list_controllers 2>/dev/null >> "${CONTROLLER_LOG}" || true
  fi
done

# ---------------------------------------------------------------------------- #
# 3. Isaac Sim を ros2_control バックエンドで起動（foreground）
# ---------------------------------------------------------------------------- #
log "--- Isaac Sim 起動 (backend=ros2_control) ---"
log "  オプション: ${ISAAC_ARGS[*]:-（なし）}"

PYTHONPATH="${REPO_ROOT}/src" \
TOMATO_HARVEST_TRAJECTORY_BACKEND=ros2_control \
"${REPO_ROOT}/python.sh" \
  "${REPO_ROOT}/scripts/run_harvest_viewer.py" \
  --backend ros2_control \
  --transport ros2 \
  "${ISAAC_ARGS[@]:-}" 2>&1

log "Isaac Sim 終了"
