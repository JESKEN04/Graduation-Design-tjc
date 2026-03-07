#!/usr/bin/env bash
set -euo pipefail

# 用法: ./run_full_stack.sh <PX4_ROOT>
PX4_ROOT="${1:-$HOME/PX4-Autopilot}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORLD_FILE="$REPO_ROOT/uav_inspection_system/worlds/inspection_environment.sdf"

if [[ ! -d "$PX4_ROOT" ]]; then
  echo "[ERROR] PX4 root not found: $PX4_ROOT"
  exit 1
fi

source /opt/ros/humble/setup.bash
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/uav_inspection_system/worlds:${GZ_SIM_RESOURCE_PATH:-}"

cleanup() {
  pkill -f "MicroXRCEAgent udp4 -p 8888" || true
  pkill -f "px4" || true
  pkill -f "gz sim" || true
}
trap cleanup EXIT

echo "[1/5] Start MicroXRCEAgent..."
MicroXRCEAgent udp4 -p 8888 > /tmp/microxrce_agent.log 2>&1 &
sleep 2

echo "[2/5] Start Gazebo Garden world..."
gz sim -r "$WORLD_FILE" > /tmp/gz_inspection_world.log 2>&1 &
sleep 5

echo "[3/5] Start 3 PX4 SITL instances (iris) ..."
cd "$PX4_ROOT"
for i in 0 1 2; do
  PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=inspection_environment PX4_GZ_MODEL_POSE="$((i * 3)),$((i * 2)),0,0,0,0" \
    ./build/px4_sitl_default/bin/px4 -i "$i" > "/tmp/px4_${i}.log" 2>&1 &
  sleep 1
done

echo "[4/5] Start RBF PID auto tuner + formation consensus nodes..."
python3 "$REPO_ROOT/uav_inspection_system/scripts/rbf_pid_auto_tuner.py" > /tmp/rbf_tuner.log 2>&1 &
python3 "$REPO_ROOT/uav_inspection_system/scripts/formation_consensus_node.py" > /tmp/formation_consensus.log 2>&1 &

echo "[5/5] Ready. Open QGroundControl and connect UDP ports from PX4 instances."
echo "    Logs: /tmp/microxrce_agent.log /tmp/gz_inspection_world.log /tmp/px4_*.log /tmp/rbf_tuner.log /tmp/formation_consensus.log"
wait
