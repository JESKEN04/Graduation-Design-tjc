#!/usr/bin/env python3
"""
仅启动Gazebo 8.10.0环境（不启动无人机和ROS节点）
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('uav_inspection_system')
    world_file = os.path.join(pkg_dir, 'worlds', 'inspection_environment.sdf')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gzsim', '-v', '4', world_file],
            output='screen'
        )
    ])
