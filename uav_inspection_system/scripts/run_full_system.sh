#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

ROS_WS=${ROS_WS:-$HOME/ros2_ws}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
WORLD_FILE=${WORLD_FILE:-$REPO_ROOT/uav_inspection_system/worlds/inspection_environment.sdf}
AGENT_PORT=${AGENT_PORT:-8888}
UAV_COUNT=${UAV_COUNT:-3}
LOG_DIR=${LOG_DIR:-$REPO_ROOT/.run_logs}

mkdir -p "$LOG_DIR"

log() {
  printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*"
}

die() {
  log "[ERR] $*"
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "缺少命令: $1"
}

start_bg() {
  local name="$1"
  local logfile="$2"
  shift 2

  log "启动: $name"
  "$@" >"$logfile" 2>&1 &
  local pid=$!
  sleep 2

  if ! kill -0 "$pid" 2>/dev/null; then
    log "[ERR] $name 启动失败，日志: $logfile"
    tail -n 80 "$logfile" || true
    return 1
  fi

  log "$name 已启动 (PID=$pid, log=$logfile)"
  PIDS+=("$pid")
}

show_logs_hint() {
  cat <<EOM

====================
实时日志查看（新开终端）:
  tail -f "$LOG_DIR/gz.log"
  tail -f "$LOG_DIR/agent.log"
  tail -f "$LOG_DIR/px4_multi.log"
  tail -f "$LOG_DIR/rbf.log"
  tail -f "$LOG_DIR/formation.log"
====================
EOM
}

PIDS=()
cleanup() {
  log "正在清理后台进程..."
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

require_cmd gz
require_cmd ros2
require_cmd MicroXRCEAgent

[[ -d "$PX4_DIR" ]] || die "PX4目录不存在: $PX4_DIR（请先 export PX4_DIR=~/PX4-Autopilot）"
[[ -f "$WORLD_FILE" ]] || die "场景文件不存在: $WORLD_FILE"
[[ -f /opt/ros/humble/setup.bash ]] || die "未找到 /opt/ros/humble/setup.bash，请先安装 ROS2 Humble"

source /opt/ros/humble/setup.bash
if [[ -f "$ROS_WS/install/setup.bash" ]]; then
  source "$ROS_WS/install/setup.bash"
else
  log "[WARN] 未找到 $ROS_WS/install/setup.bash，后续 ros2 run 可能失败"
fi

export GZ_SIM_RESOURCE_PATH="$(dirname "$WORLD_FILE"):$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

log "配置: PX4_DIR=$PX4_DIR"
log "配置: WORLD_FILE=$WORLD_FILE"
log "配置: UAV_COUNT=$UAV_COUNT"
log "配置: LOG_DIR=$LOG_DIR"

start_bg "Gazebo" "$LOG_DIR/gz.log" gz sim -v 4 "$WORLD_FILE"
start_bg "MicroXRCEAgent" "$LOG_DIR/agent.log" MicroXRCEAgent udp4 -p "$AGENT_PORT"

if [[ -x "$PX4_DIR/Tools/simulation/gz/sitl_multiple_run.sh" ]]; then
  start_bg "PX4 多机脚本" "$LOG_DIR/px4_multi.log" bash -lc "cd '$PX4_DIR' && ./Tools/simulation/gz/sitl_multiple_run.sh -m iris -n '$UAV_COUNT' -w '$WORLD_FILE'"
else
  log "[WARN] 未找到 sitl_multiple_run.sh，使用兼容模式逐个启动"
  for i in $(seq 0 $((UAV_COUNT - 1))); do
    start_bg "PX4-$i" "$LOG_DIR/px4_$i.log" bash -lc "cd '$PX4_DIR' && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_iris PX4_GZ_WORLD='$(basename "$WORLD_FILE")' PX4_GZ_MODEL_POSE='$((i*3)),$((i*2)),0,0,0,0' ./build/px4_sitl_default/bin/px4 -i '$i'"
  done
fi

start_bg "RBF自整定节点" "$LOG_DIR/rbf.log" ros2 run uav_inspection_system rbf_pid_autotune_node.py
start_bg "一致性编队节点" "$LOG_DIR/formation.log" ros2 run uav_inspection_system formation_consensus_node.py

log "全链路已启动。可打开 QGC 解锁与起飞。"
show_logs_hint

wait
