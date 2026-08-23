# 此文件用于同时启动外部 PCD 地图服务器与 HDL 定位节点，形成 map 到 odom 到 base_link 定位链。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# 此函数用于组装 HDL 定位启动描述；输入为 launch 参数，输出为地图服务器和定位节点，副作用是启动 ROS2 节点。
def generate_launch_description():
    package_share = get_package_share_directory('hdl_localization')
    default_params = os.path.join(package_share, 'config', 'sentry_left_mid360.yaml')
    params_file = LaunchConfiguration('params_file')
    globalmap_pcd = LaunchConfiguration('globalmap_pcd')

    return LaunchDescription([
        DeclareLaunchArgument(
            'globalmap_pcd',
            description='必填：外部全局地图 PCD 的绝对路径；不会随仓库提交。'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='HDL 定位参数 YAML 路径。'),
        Node(
            package='hdl_localization', executable='hdl_localization_map_server',
            name='globalmap_server', output='screen',
            parameters=[params_file, {'globalmap_pcd': globalmap_pcd}]),
        Node(
            package='hdl_localization', executable='hdl_localization_node',
            name='hdl_localization', output='screen',
            parameters=[params_file]),
    ])
