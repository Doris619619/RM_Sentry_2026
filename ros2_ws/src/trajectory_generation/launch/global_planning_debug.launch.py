"""ROS2 equivalent of ROS1 global_searcher_debug.launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('trajectory_generation')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('planner_prefix', default_value='gdb -ex run --args'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'global_planning.launch.py')),
            launch_arguments={
                'use_rviz': LaunchConfiguration('use_rviz'),
                'planner_prefix': LaunchConfiguration('planner_prefix'),
            }.items()),
    ])
