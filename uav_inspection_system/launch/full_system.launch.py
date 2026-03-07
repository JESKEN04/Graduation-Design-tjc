#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('uav_inspection_system')
    default_world = os.path.join(pkg_dir, 'worlds', 'inspection_environment.sdf')

    px4_dir_arg = DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot'))
    world_arg = DeclareLaunchArgument('world', default_value=default_world)
    uav_count_arg = DeclareLaunchArgument('uav_count', default_value='3')

    px4_dir = LaunchConfiguration('px4_dir')
    world = LaunchConfiguration('world')
    uav_count = LaunchConfiguration('uav_count')

    return LaunchDescription([
        px4_dir_arg,
        world_arg,
        uav_count_arg,
        ExecuteProcess(cmd=['gz', 'sim', '-v', '4', world], output='screen'),
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='screen'),
        ExecuteProcess(
            cmd=['bash', '-lc', 'cd "$0" && ./Tools/simulation/gz/sitl_multiple_run.sh -m iris -n "$1" -w "$2"', px4_dir, uav_count, world],
            output='screen'
        ),
        ExecuteProcess(cmd=['ros2', 'run', 'uav_inspection_system', 'rbf_pid_autotune_node.py'], output='screen'),
        ExecuteProcess(cmd=['ros2', 'run', 'uav_inspection_system', 'formation_consensus_node.py'], output='screen'),
    ])
