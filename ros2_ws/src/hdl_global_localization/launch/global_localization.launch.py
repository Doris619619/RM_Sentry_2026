from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hdl_global_localization')
    default_params = share + '/config/global_localization.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='ROS2 global-localization parameters; standard default is BBS.'),
        Node(
            package='hdl_global_localization',
            executable='hdl_global_localization_node',
            name='hdl_global_localization',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
