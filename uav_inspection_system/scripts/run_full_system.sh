#!/usr/bin/env bash
set -euo pipefail

ROS_WS=${ROS_WS:-$HOME/ros2_ws}
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
WORLD_FILE=${WORLD_FILE:-$(pwd)/uav_inspection_system/worlds/inspection_environment.sdf}

if [[ ! -d "$PX4_DIR" ]]; then
  echo "[ERR] PX4目录不存在: $PX4_DIR"
  exit 1
fi

source /opt/ros/humble/setup.bash

if [[ -f "$ROS_WS/install/setup.bash" ]]; then
  source "$ROS_WS/install/setup.bash"
fi

export PX4_GZ_WORLD="$(basename "$WORLD_FILE")"
export PX4_GZ_MODEL=iris
export GZ_SIM_RESOURCE_PATH="$(dirname "$WORLD_FILE"):$PX4_DIR/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"

# 1) 启动Gazebo
(gz sim -v 4 "$WORLD_FILE" &) 
sleep 4

# 2) 启动MicroXRCE-DDS Agent
(MicroXRCEAgent udp4 -p 8888 &) 
sleep 2

# 3) 启动三架PX4 SITL (iris)
for i in 0 1 2; do
  (
    cd "$PX4_DIR"
    PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_iris PX4_GZ_MODEL_POSE="$((i*3)),$((i*2)),0,0,0,0" ./build/px4_sitl_default/bin/px4 -i $i
  ) &
  sleep 2
done

# 4) 启动ROS2算法节点
ros2 run uav_inspection_system rbf_pid_autotune_node.py &
ros2 run uav_inspection_system formation_consensus_node.py &

wait
