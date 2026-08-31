"""Non-actuating diagnostics: global-localization service and planning graph."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    hgl = get_package_share_directory('hdl_global_localization')
    planning = get_package_share_directory('trajectory_generation')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(hgl, 'launch', 'global_localization.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(planning, 'launch', 'global_planning.launch.py')),
            launch_arguments={'use_rviz': 'false'}.items()),
    ])
