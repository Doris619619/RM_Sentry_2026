"""Deterministic software fixture entrypoint: no physical serial device."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    decision = get_package_share_directory('decision_node')
    sim = get_package_share_directory('sentry_sim')
    return LaunchDescription([
        DeclareLaunchArgument('baudrate', default_value='921600'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(decision, 'launch', 'sentry_system_mock.launch.py')),
            launch_arguments={'serial_port': '/dev/ttyUSB_MOCK',
                              'baudrate': LaunchConfiguration('baudrate')}.items()),
        Node(package='sentry_sim', executable='fixture_inputs', output='screen'),
    ])
