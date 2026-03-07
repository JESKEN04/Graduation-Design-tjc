cmake_minimum_required(VERSION 3.8)
project(uav_inspection_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 包含目录
include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# 编译标志
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============ 库编译 ============

# 1. 环境建模库
add_library(grid_map_generator
  src/environment_modeling/grid_map_generator.cpp
)
ament_target_dependencies(grid_map_generator rclcpp std_msgs sensor_msgs)

# 2. PID控制器库
add_library(pid_controller
  src/single_uav_control/pid_controller.cpp
)
ament_target_dependencies(pid_controller rclcpp)

# 3. RBF神经网络库
add_library(rbf_neural_network
  src/single_uav_control/rbf_neural_network.cpp
)
ament_target_dependencies(rbf_neural_network rclcpp)

# 4. 自适应控制器库
add_library(adaptive_controller
  src/single_uav_control/adaptive_controller.cpp
)
target_link_libraries(adaptive_controller pid_controller rbf_neural_network)
ament_target_dependencies(adaptive_controller rclcpp)

# 5. 人工蜂群算法库
add_library(artificial_bee_colony
  src/path_planning/artificial_bee_colony.cpp
)
ament_target_dependencies(artificial_bee_colony rclcpp)

# 6. 路径优化库
add_library(path_optimizer
  src/path_planning/path_optimizer.cpp
)
target_link_libraries(path_optimizer artificial_bee_colony grid_map_generator)
ament_target_dependencies(path_optimizer rclcpp)

# 7. 图论库
add_library(graph_theory
  src/formation_control/graph_theory.cpp
)
ament_target_dependencies(graph_theory rclcpp)

# 8. 共识算法库
add_library(consensus_algorithm
  src/formation_control/consensus_algorithm.cpp
)
target_link_libraries(consensus_algorithm graph_theory)
ament_target_dependencies(consensus_algorithm rclcpp)

# 9. 编队保持库
add_library(formation_keeper
  src/formation_control/formation_keeper.cpp
)
target_link_libraries(formation_keeper consensus_algorithm adaptive_controller)
ament_target_dependencies(formation_keeper rclcpp geometry_msgs nav_msgs)

# ============ 可执行文件编译 ============

# Gazebo桥接节点
add_executable(gazebo_bridge_node
  src/ros2_nodes/gazebo_bridge_node.cpp
)
target_link_libraries(gazebo_bridge_node grid_map_generator)
ament_target_dependencies(gazebo_bridge_node rclcpp sensor_msgs)

# PX4桥接节点
add_executable(px4_bridge_node
  src/ros2_nodes/px4_bridge_node.cpp
)
target_link_libraries(px4_bridge_node adaptive_controller)
ament_target_dependencies(px4_bridge_node rclcpp std_msgs)

# 任务协调节点
add_executable(mission_coordinator_node
  src/ros2_nodes/mission_coordinator_node.cpp
)
target_link_libraries(mission_coordinator_node path_optimizer formation_keeper)
ament_target_dependencies(mission_coordinator_node rclcpp std_msgs geometry_msgs)

# 状态监测节点
add_executable(state_monitor_node
  src/ros2_nodes/state_monitor_node.cpp
)
ament_target_dependencies(state_monitor_node rclcpp std_msgs geometry_msgs nav_msgs)

# ============ 安装 ============

# 安装可执行文件
install(TARGETS
  gazebo_bridge_node
  px4_bridge_node
  mission_coordinator_node
  state_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装库
install(TARGETS
  grid_map_generator
  pid_controller
  rbf_neural_network
  adaptive_controller
  artificial_bee_colony
  path_optimizer
  graph_theory
  consensus_algorithm
  formation_keeper
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装launch、worlds、config等
install(DIRECTORY
  launch
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装可执行脚本
install(PROGRAMS
  scripts/run_full_system.sh
  scripts/rbf_pid_autotune_node.py
  scripts/formation_consensus_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装可执行脚本
install(PROGRAMS
  scripts/run_full_system.sh
  scripts/rbf_pid_autotune_node.py
  scripts/formation_consensus_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
