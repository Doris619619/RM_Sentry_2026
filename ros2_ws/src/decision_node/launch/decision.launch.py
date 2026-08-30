from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('decision_node')
    default = share + '/config/decision.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        Node(package='decision_node', executable='strategy_node', name='strategy_node',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
