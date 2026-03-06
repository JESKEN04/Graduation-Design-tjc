# Graduation-Design-tjc

面向“复杂输电线路巡检”的多无人机编队毕业设计工程（ROS2 Humble + Gazebo Garden/Sim8 + PX4）。

## 1. 你现在能直接得到什么

本仓库已补齐一个可落地运行的“完整方案骨架”：

- **Gazebo 环境模型**（三维栅格建模思路）：
  - 两个输电塔（一个在山上，一个在平地）。
  - 两塔之间有弧垂电线。
  - 山体+林木障碍，环境复杂度足够体现“单机巡检困难，编队巡检必要”。
  - 场景中预置 **3 架 Iris** 等待 QGC 指令。
- **RBF 神经网络 PID 自整定节点**：
  - 在线订阅飞行误差数据，使用 **RBF + 递推最小二乘（RLS）** 更新参数。
  - 发布每架无人机 PID 增益建议值。
- **编队控制节点**：
  - 采用图论拉普拉斯矩阵 + 一致性更新，输出三机目标位姿。
- **一键运行脚本**：启动 Gazebo、MicroXRCEAgent、3 架 PX4 SITL、算法节点。

---

## 2. 环境版本（与你提供的一致）

建议版本：

- ROS2: humble
- Gazebo Sim: 8.10.x（Garden 兼容）
- PX4: v1.17.0-alpha1 附近版本
- MicroXRCEAgent: v2.4.2

---

## 3. 仓库结构与每个关键文件作用

```text
uav_inspection_system/
├── worlds/
│   └── inspection_environment.sdf      # 复杂巡检环境（山体/双塔/弧垂电线/树林/3架Iris）
├── launch/
│   ├── full_system.launch.py           # ROS2 launch入口（Gazebo + Agent +算法节点）
│   └── ...
├── scripts/
│   ├── run_full_system.sh              # 一键拉起完整链路（推荐）
│   ├── rbf_pid_autotune_node.py        # RBF+RLS在线PID自整定节点
│   └── formation_consensus_node.py     # 图论一致性编队目标生成节点
├── config/
│   └── *.yaml                          # 参数文件（可继续扩展）
├── src/
│   └── ...                             # C++模块骨架（后续可逐步替换成工程级实现）
└── CMakeLists.txt / package.xml        # 构建与安装配置
```

---

## 4. 快速开始（最推荐：一键脚本）

> 假设你的 PX4 在 `~/PX4-Autopilot`，ROS 工作空间为 `~/ros2_ws`。

### 4.1 编译 ROS2 包

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -s /workspace/Graduation-Design-tjc/uav_inspection_system .

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_inspection_system
source install/setup.bash
```

### 4.2 运行完整系统

```bash
cd /workspace/Graduation-Design-tjc
./uav_inspection_system/scripts/run_full_system.sh
```

脚本会自动执行：
1. 启动 Gazebo 并加载 `inspection_environment.sdf`。
2. 启动 `MicroXRCEAgent udp4 -p 8888`。
3. 启动 3 个 PX4 SITL 实例（iris 模型）。
4. 启动 RBF 自整定节点与编队一致性节点。

---

## 5. QGC 使用说明（你关心的解锁/起飞）

1. 启动 QGroundControl。
2. 保证与本机 UDP/MAVLink 端口互通。
3. 每架无人机可在 QGC 分别执行：
   - 解锁（Arm）
   - 起飞（Takeoff）
   - 切换 Offboard / Mission 模式（按你的任务流）
4. RBF 节点会实时根据误差输入，发布 `/uavX/pid_gains` 建议值，用于外部参数写入策略。

> 说明：PX4 参数写入可通过 MAVLink/MAVSDK 或 px4_ros_com 扩展实现，这部分在本仓库中以“接口就绪”为主，便于你毕业设计答辩时分阶段展示。

---

## 6. 关键话题设计（ROS2）

- 输入：`/uavX/telemetry` (`std_msgs/Float32MultiArray`)
  - 约定数据：`[ex, ey, ez, evx, evy, evz]`
- 输出：`/uavX/pid_gains` (`std_msgs/Float32MultiArray`)
  - 数据：`[kp, ki, kd]`
- 编队目标：`/uavX/target_pose` (`geometry_msgs/PoseStamped`)

你可以让 px4_bridge 节点将 PX4 uORB/RTPS 数据转换成上述话题。

---

## 7. 论文撰写建议（可直接引用）

- **环境复杂性论证**：山体遮挡 + 林木障碍 + 输电走廊跨度大 + 电线弧垂几何复杂，单机路径需频繁绕障且视角受限。
- **编队必要性**：三机可实现“主巡检 + 侧向避障感知 + 后方补盲”，并提高故障定位效率。
- **控制创新点**：在传统 PID 上叠加 RBF-RLS 自适应参数调节，提升扰动工况下稳定性。
- **理论基础**：图论邻接关系建模 + 拉普拉斯一致性协议，保持相对队形并跟踪巡检走廊。

---

## 8. 常见问题

1. **Gazebo 找不到 iris 模型**
   - 检查 `GZ_SIM_RESOURCE_PATH` 是否包含 `PX4-Autopilot/Tools/simulation/gz/models`。
2. **MicroXRCEAgent 编译问题**
   - 如你所说，可将 FastDDS 版本字符串修正为 `v2.12.1` 后重新 `make`。
3. **QGC 无法同时看到三机**
   - 检查 PX4 实例 ID 和 MAVLink UDP 端口分配是否冲突。

---

## 9. 后续可选增强

- 将 RBF 输出直接通过 MAVSDK 写入 PX4 参数，实现全自动闭环调参。
- 在编队控制层增加障碍势场项，实现“编队一致性 + 避障”统一优化。
- 用真实 DEM 或点云替换当前几何山体模型，提高论文工程可信度。
