#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('uav_inspection_system')
    world_file = os.path.join(pkg_dir, 'worlds', 'inspection_environment.sdf')
    
    if not os.path.exists(world_file):
        world_file = os.path.expanduser("~/ros2_ws/src/uav_inspection_system/worlds/inspection_environment.sdf")
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', world_file],
            output='screen'
        )
    ])
