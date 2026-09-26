"""Independent stage-two entry; never starts tracking, localization hardware or MCU."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Forward the planner toggle for map-only QA; normal start enables planning."""
    launch=Path(get_package_share_directory('sentry_gazebo'))/'launch/planning.launch.py'
    return LaunchDescription([DeclareLaunchArgument('enable_planner',default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)),
            launch_arguments={'enable_planner':LaunchConfiguration('enable_planner')}.items())])
