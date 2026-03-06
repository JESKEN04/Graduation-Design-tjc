#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

ROS_WS=${ROS_WS:-$HOME/ros2_ws}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
WORLD_FILE=${WORLD_FILE:-$REPO_ROOT/uav_inspection_system/worlds/inspection_environment.sdf}
AGENT_PORT=${AGENT_PORT:-8888}
UAV_COUNT=${UAV_COUNT:-3}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[ERR] 缺少命令: $1"
    exit 1
  fi
}

require_cmd gz
require_cmd ros2
require_cmd MicroXRCEAgent

if [[ ! -d "$PX4_DIR" ]]; then
  echo "[ERR] PX4目录不存在: $PX4_DIR"
  echo "      请设置 PX4_DIR 环境变量，例如: export PX4_DIR=~/PX4-Autopilot"
  exit 1
fi

if [[ ! -f "$WORLD_FILE" ]]; then
  echo "[ERR] 场景文件不存在: $WORLD_FILE"
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[ERR] 未找到 /opt/ros/humble/setup.bash，请先安装 ROS2 Humble"
  exit 1
fi

source /opt/ros/humble/setup.bash
if [[ -f "$ROS_WS/install/setup.bash" ]]; then
  source "$ROS_WS/install/setup.bash"
else
  echo "[WARN] 未找到 $ROS_WS/install/setup.bash，后续 ros2 run 可能失败"
fi

export GZ_SIM_RESOURCE_PATH="$(dirname "$WORLD_FILE"):$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

cleanup() {
  echo "[INFO] 正在清理后台进程..."
  jobs -pr | xargs -r kill
}
trap cleanup EXIT INT TERM

echo "[1/4] 启动 Gazebo..."
(gz sim -v 4 "$WORLD_FILE" &) 
sleep 4

echo "[2/4] 启动 MicroXRCEAgent (udp4:${AGENT_PORT})..."
(MicroXRCEAgent udp4 -p "$AGENT_PORT" &) 
sleep 2

echo "[3/4] 启动 PX4 多机 SITL (${UAV_COUNT} 架)..."
if [[ -x "$PX4_DIR/Tools/simulation/gz/sitl_multiple_run.sh" ]]; then
  (
    cd "$PX4_DIR"
    ./Tools/simulation/gz/sitl_multiple_run.sh -m iris -n "$UAV_COUNT" -w "$WORLD_FILE"
  ) &
else
  echo "[WARN] 未找到 sitl_multiple_run.sh，使用兼容模式逐个启动"
  for i in $(seq 0 $((UAV_COUNT - 1))); do
    (
      cd "$PX4_DIR"
      PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_iris PX4_GZ_WORLD="$(basename "$WORLD_FILE")" \
      PX4_GZ_MODEL_POSE="$((i*3)),$((i*2)),0,0,0,0" ./build/px4_sitl_default/bin/px4 -i "$i"
    ) &
    sleep 2
  done
fi

sleep 4

echo "[4/4] 启动 ROS2 算法节点..."
ros2 run uav_inspection_system rbf_pid_autotune_node.py &
ros2 run uav_inspection_system formation_consensus_node.py &

echo "[INFO] 全链路已启动。可打开 QGC 解锁与起飞。"
wait
