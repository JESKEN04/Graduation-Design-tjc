#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _px4_command(context):
    px4_dir = LaunchConfiguration('px4_dir').perform(context)
    world = LaunchConfiguration('world').perform(context)
    uav_count = LaunchConfiguration('uav_count').perform(context)
    cmd = f"cd '{px4_dir}' && ./Tools/simulation/gz/sitl_multiple_run.sh -m iris -n '{uav_count}' -w '{world}'"
    return [ExecuteProcess(cmd=['bash', '-lc', cmd], output='screen')]


def generate_launch_description():
    pkg_dir = get_package_share_directory('uav_inspection_system')
    default_world = os.path.join(pkg_dir, 'worlds', 'inspection_environment.sdf')

    return LaunchDescription([
        DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot')),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('uav_count', default_value='3'),
        ExecuteProcess(cmd=['gz', 'sim', '-v', '4', LaunchConfiguration('world')], output='screen'),
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='screen'),
        OpaqueFunction(function=_px4_command),
        ExecuteProcess(cmd=['ros2', 'run', 'uav_inspection_system', 'rbf_pid_autotune_node.py'], output='screen'),
        ExecuteProcess(cmd=['ros2', 'run', 'uav_inspection_system', 'formation_consensus_node.py'], output='screen'),
    ])
