#!/usr/bin/env python3
"""
多无人机 + Gazebo 8.10.0 仿真启动脚本
使用gzsim命令启动Gazebo
"""

import os
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 获取包共享目录
    pkg_dir = get_package_share_directory('uav_inspection_system')
    world_file = os.path.join(pkg_dir, 'worlds', 'inspection_environment.sdf')
    
    # 确认文件存在
    if not os.path.exists(world_file):
        print(f"[WARNING] World file not found: {world_file}")
        world_file = ""
    
    ld = LaunchDescription()
    
    # ============ 启动 Gazebo 8.10.0 ============
    # 使用 gzsim 而不是 gazebo 命令
    gazebo_process = ExecuteProcess(
        cmd=['gzsim', '-v', '4', world_file if world_file else ''],
        output='screen',
        shell=False,
        on_exit=None
    )
    ld.add_action(gazebo_process)
    
    # ============ 启动三个无人机 ============
    # 注：实际启动PX4需要额外配置，这里仅启动桥接节点
    
    for i in range(1, 4):
        namespace = f'uav{i}'
        
        # PX4桥接节点
        px4_bridge = Node(
            package='uav_inspection_system',
            executable='px4_bridge_node',
            namespace=namespace,
            output='screen',
            parameters=[
                {'uav_id': i},
                {'instance': i-1}
            ]
        )
        ld.add_action(px4_bridge)
    
    # ============ 启动Gazebo桥接节点 ============
    gazebo_bridge = Node(
        package='uav_inspection_system',
        executable='gazebo_bridge_node',
        output='screen'
    )
    ld.add_action(gazebo_bridge)
    
    # ============ 启动任务协调节点 ============
    mission_coordinator = Node(
        package='uav_inspection_system',
        executable='mission_coordinator_node',
        output='screen'
    )
    ld.add_action(mission_coordinator)
    
    # ============ 启动状态监测节点 ============
    state_monitor = Node(
        package='uav_inspection_system',
        executable='state_monitor_node',
        output='screen'
    )
    ld.add_action(state_monitor)
    
    return ld
