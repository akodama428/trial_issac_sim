#!/bin/bash
# 起動中のすべての ROS 2 ノードとデーモンを停止する

source /opt/ros/jazzy/setup.bash 2>/dev/null || true

echo "=== 起動中のノード ==="
nodes=$(ros2 node list 2>/dev/null)
if [ -z "$nodes" ]; then
    echo "(ノードなし)"
else
    echo "$nodes"
fi

echo ""
echo "=== 停止中 (SIGINT) ==="

TARGETS=(
    "ros2 run"
    "ros2 launch"
    "move_group"
    "plotjuggler"
    "foxglove_bridge"
    "rviz2"
)

for pattern in "${TARGETS[@]}"; do
    if pgrep -f "$pattern" > /dev/null 2>&1; then
        echo "  kill: $pattern"
        pkill -SIGINT -f "$pattern" 2>/dev/null || true
    fi
done

sleep 2

echo "=== 残存プロセスを強制終了 (SIGKILL) ==="

for pattern in "${TARGETS[@]}"; do
    if pgrep -f "$pattern" > /dev/null 2>&1; then
        echo "  force kill: $pattern"
        pkill -SIGKILL -f "$pattern" 2>/dev/null || true
    fi
done

echo ""
echo "=== ROS 2 デーモン停止 ==="
ros2 daemon stop 2>/dev/null && echo "  daemon stopped" || echo "  daemon already stopped"

echo ""
echo "完了"
