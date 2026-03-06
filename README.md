# 毕业设计一键复现实验：Gazebo Garden + PX4 + ROS 2 多无人机输电巡检

本仓库给出一个可直接落地的仿真方案，目标对应你的需求：

- Gazebo Garden 8.10.0 中构建 **复杂巡检场景**（山体、两座高差输电塔、弯垂电线、树林障碍）。
- 场景中放置 **3 架 Iris 无人机** 等待 QGroundControl（QGC）下发解锁/起飞。
- 使用 ROS 2 节点实现：
  - **编队一致性控制（图论 + 共识）**；
  - **RBF 神经网络在线训练（最小二乘）**，实时发布 PID 增益补偿建议。
- 使用 MicroXRCEAgent 作为 PX4 <-> ROS 2 通信代理。

---

## 1. 目录说明

- `uav_inspection_system/worlds/inspection_environment.sdf`  
  巡检环境模型（山地 + 输电塔 + 电线 + 树林 + 3 架 Iris）。
- `uav_inspection_system/scripts/run_full_stack.sh`  
  一键启动脚本：Agent、Gazebo、3 实例 PX4、RBF 自动调参、编队共识控制。
- `uav_inspection_system/scripts/rbf_pid_auto_tuner.py`  
  RBF 在线训练器（从 PX4 ROS2 话题读取状态，最小二乘拟合并发布 PID 补偿）。
- `uav_inspection_system/scripts/formation_consensus_node.py`  
  三机编队控制节点（基于一致性协议保持相对构型）。

---

## 2. 环境要求（与你版本对齐）

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Sim 8.10.x（Garden 兼容）
- PX4-Autopilot `v1.17.0-alpha1`（你的当前版本）
- MicroXRCEAgent `v2.4.2`

> 说明：你当前系统中 `gz sim --version` 已是 `8.10.0`，可直接用本方案。

---

## 3. 快速启动（推荐）

### 3.1 给脚本执行权限

```bash
cd /workspace/Graduation-Design-tjc
chmod +x uav_inspection_system/scripts/run_full_stack.sh
chmod +x uav_inspection_system/scripts/rbf_pid_auto_tuner.py
chmod +x uav_inspection_system/scripts/formation_consensus_node.py
```

### 3.2 启动完整系统

```bash
cd /workspace/Graduation-Design-tjc
./uav_inspection_system/scripts/run_full_stack.sh ~/PX4-Autopilot
```

脚本将按顺序自动启动：
1. `MicroXRCEAgent udp4 -p 8888`
2. `gz sim` + `inspection_environment.sdf`
3. 3 个 PX4 SITL Iris 实例
4. RBF 在线调参与编队共识节点

日志在：

- `/tmp/microxrce_agent.log`
- `/tmp/gz_inspection_world.log`
- `/tmp/px4_0.log`, `/tmp/px4_1.log`, `/tmp/px4_2.log`
- `/tmp/rbf_tuner.log`
- `/tmp/formation_consensus.log`

---

## 4. QGroundControl 使用方式

1. 启动 QGC。  
2. 连接 PX4 SITL（默认 UDP）。  
3. 对三架机分别执行：
   - Arm（解锁）
   - Takeoff（起飞）
4. 起飞后，编队控制节点会根据一致性模型不断修正目标点。
5. RBF 节点持续学习飞行状态并发布 PID 补偿建议话题。

---

## 5. RBF 自动训练逻辑（你论文可直接写）

`rbf_pid_auto_tuner.py` 实现思路：

1. 订阅每架无人机的 PX4 ROS2 话题：
   - `/uavX/fmu/out/vehicle_local_position`
   - `/uavX/fmu/out/vehicle_attitude`
2. 提取速度/加速度状态并构建 RBF 特征向量。
3. 在滑动窗口内积累样本，使用正则化最小二乘：

\[
W = (\Phi^T\Phi + \lambda I)^{-1}\Phi^T Y
\]

4. 输出当前时刻预测补偿量到：
   - `/uavX/rbf_pid_correction`（`Float32MultiArray`）

这些补偿量可作为 PX4 外部自适应调参输入（你可后续在 offboard 控制器中叠加到 PID 参数或控制量）。

---

## 6. 编队控制逻辑（蜂群/图论/一致性）

`formation_consensus_node.py` 采用：

- 图结构：3 机全连接邻接；
- 一致性项：\(u_i = \sum_{j \neq i}(x_j - x_i - d_{ij})\)；
- 群体中心收敛项：引导编队整体稳定巡检。

输出话题：
- `/uavX/formation_setpoint`（`geometry_msgs/PoseStamped`）

可作为后续 offboard setpoint 的输入。

---

## 7. 论文中的“复杂场景必要性”体现点

本场景满足无人机编队巡检必要性的关键论据：

- 山地起伏导致视线遮挡、飞行高度变化大；
- 两座高差输电塔 + 弯垂电线提升轨迹规划难度；
- 树林障碍增加避障与协同覆盖需求；
- 多机协同可提高覆盖率、鲁棒性和任务效率。

---

## 8. 常见问题排查

1. `MicroXRCEAgent` 启动失败：
   - 执行 `MicroXRCEAgent -h` 检查安装。
2. `model://iris` 未找到：
   - 需确认 PX4 Gazebo 模型路径或 Gazebo Fuel 能访问。
3. ROS2 话题没有数据：
   - 检查 PX4 实例是否成功连接 Agent。
4. 需要停止系统：
   - `Ctrl + C`，脚本内置清理逻辑会自动 `pkill`。

---

## 9. 建议答辩展示流程

1. 先打开 Gazebo 展示场景复杂性（山体 + 输电线路 + 树林）。
2. 打开 QGC，三机解锁起飞。
3. 展示 ROS2 话题数据（位置、编队 setpoint、RBF 补偿输出）。
4. 对比开/关 RBF 补偿时编队稳定性差异（抖动、超调、收敛时间）。

祝你毕业设计顺利通过！
