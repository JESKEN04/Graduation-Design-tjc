# Graduation-Design-tjc

面向“复杂输电线路巡检”的多无人机编队毕业设计工程（ROS 2 Humble + Gazebo Sim 8/Garden + PX4）。

## 1. 项目内容

本仓库提供：

- 复杂巡检场景 `inspection_environment.sdf`：山体、两座输电塔（山上/平地）、弧垂导线、树木障碍、3 架 Iris。  
- 在线自整定节点 `rbf_pid_autotune_node.py`：订阅飞行误差并输出 PID 建议值。  
- 编队一致性节点 `formation_consensus_node.py`：基于图论拉普拉斯生成三机目标位姿。  
- 一键脚本 `run_full_system.sh`：拉起 Gazebo + MicroXRCEAgent + PX4 多机 + ROS2 节点。  

---

## 2. 推荐版本

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Sim 8.x（Garden 系列）
- PX4 v1.16 / v1.17（已编译 `px4_sitl_default`）
- MicroXRCEAgent v2.4.2

---

## 3. 从零开始：在哪个目录下载 + 如何放到 ROS2 工作空间

> 建议目录结构（最稳妥）：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <你的GitHub仓库地址> Graduation-Design-tjc
```

然后把 ROS 包放进工作空间（本仓库里真正要编译的是 `uav_inspection_system`）：

```bash
cd ~/ros2_ws/src
ln -snf ~/ros2_ws/src/Graduation-Design-tjc/uav_inspection_system ./uav_inspection_system
```

> 如果你已经直接把仓库放在 `~/ros2_ws/src/Graduation-Design-tjc`，上面的软链接命令也可以直接执行。

---

## 4. 编译

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_inspection_system
source install/setup.bash
```

---

## 5. 一键启动（推荐）

```bash
cd ~/ros2_ws/src/Graduation-Design-tjc
chmod +x uav_inspection_system/scripts/run_full_system.sh
./uav_inspection_system/scripts/run_full_system.sh
```

可选环境变量：

```bash
export PX4_DIR=~/PX4-Autopilot
export ROS_WS=~/ros2_ws
export UAV_COUNT=3
export AGENT_PORT=8888
```

脚本会自动：

1. 启动 Gazebo 并加载 `uav_inspection_system/worlds/inspection_environment.sdf`；
2. 启动 `MicroXRCEAgent udp4 -p 8888`；
3. 优先调用 PX4 官方 `Tools/simulation/gz/sitl_multiple_run.sh` 启动 3 架 Iris；
4. 启动两个 ROS2 算法节点。

---

## 6. 用 ROS2 launch 启动（可选）

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_inspection_system full_system.launch.py px4_dir:=~/PX4-Autopilot uav_count:=3
```

---

## 7. QGroundControl 使用

1. 打开 QGC；
2. 等待 3 架机连接；
3. 分别 Arm / Takeoff；
4. 按需切换 Offboard 或 Mission。

ROS2 关键话题：

- `/uavX/telemetry` (`std_msgs/Float32MultiArray`)：`[ex, ey, ez, evx, evy, evz]`
- `/uavX/pid_gains` (`std_msgs/Float32MultiArray`)：`[kp, ki, kd]`
- `/uavX/target_pose` (`geometry_msgs/PoseStamped`)

---

## 8. 常见启动失败与排查（这次重点补齐）

### 8.1 `ros2 run ... not found`
- 先确认已 `colcon build` 且 `source ~/ros2_ws/install/setup.bash`。
- 检查：
  ```bash
  ros2 pkg executables uav_inspection_system
  ```

### 8.2 Gazebo 里看不到 iris
- 检查 `PX4_DIR` 是否正确。
- 检查模型路径：
  ```bash
  echo $GZ_SIM_RESOURCE_PATH
  ```

### 8.3 脚本报 `sitl_multiple_run.sh` 不存在
- 说明你的 PX4 版本目录结构不同。
- 脚本已内置回退模式，会逐个启动 PX4 实例；若仍失败，请先在 `~/PX4-Autopilot` 完成：
  ```bash
  make px4_sitl_default
  ```

### 8.4 MicroXRCEAgent 命令找不到
- 检查安装是否成功：
  ```bash
  which MicroXRCEAgent
  ```

### 8.5 端口冲突（QGC 看不到三机）
- 关闭旧进程后重启：
  ```bash
  pkill -f MicroXRCEAgent
  pkill -f px4
  pkill -f "gz sim"
  ```

---

## 9. 关键文件说明

```text
uav_inspection_system/
├── worlds/inspection_environment.sdf      # 复杂巡检场景
├── scripts/run_full_system.sh             # 一键启动脚本
├── scripts/rbf_pid_autotune_node.py       # RBF+RLS PID 自整定
├── scripts/formation_consensus_node.py    # 一致性编队目标生成
├── launch/full_system.launch.py           # launch 启动入口
├── CMakeLists.txt / package.xml           # ROS2 包构建与依赖
```
