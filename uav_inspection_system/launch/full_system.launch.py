#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('uav_inspection_system')
    world_file = os.path.join(pkg_dir, 'worlds', 'inspection_environment.sdf')

    return LaunchDescription([
        ExecuteProcess(cmd=['gz', 'sim', '-v', '4', world_file], output='screen'),
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='screen'),
        ExecuteProcess(cmd=['ros2', 'run', 'uav_inspection_system', 'rbf_pid_autotune_node.py'], output='screen'),
        ExecuteProcess(cmd=['ros2', 'run', 'uav_inspection_system', 'formation_consensus_node.py'], output='screen'),
    ])
