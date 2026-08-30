from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('decision_node')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=share + '/config/mcu.yaml'),
        Node(package='decision_node', executable='mcu_communicator', name='mcu_communicator',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
