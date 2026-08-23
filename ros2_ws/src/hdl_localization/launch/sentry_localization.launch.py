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
        DeclareLaunchArgument(
            'points_topic', default_value='/filted_topic_3d',
            description='输入的过滤点云话题。'),
        DeclareLaunchArgument(
            'globalmap_topic', default_value='/globalmap',
            description='地图服务器发布的全局点云话题。'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/point_lio/odometry',
            description='Point-LIO 发布的运动预测里程计话题。'),
        DeclareLaunchArgument(
            'odom_frame', default_value='odom',
            description='Point-LIO 里程计父坐标系。'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='Point-LIO 里程计子坐标系。'),
        DeclareLaunchArgument(
            'cloud_qos_depth', default_value='5',
            description='HDL 输入点云 QoS 深度。'),
        DeclareLaunchArgument(
            'odom_qos_depth', default_value='20',
            description='HDL 运动预测里程计 QoS 深度。'),
        DeclareLaunchArgument(
            'tf_lookup_timeout_seconds', default_value='0.1',
            description='查询 odom 到 base_link 的最大等待秒数。'),
        DeclareLaunchArgument(
            'max_prediction_age_seconds', default_value='0.2',
            description='运动预测允许的最大时效秒数。'),
        DeclareLaunchArgument(
            'ndt_resolution', default_value='1.0',
            description='CPU NDT_OMP 的分辨率。'),
        DeclareLaunchArgument(
            'downsample_resolution', default_value='0.1',
            description='输入地图与点云的下采样分辨率。'),
        Node(
            package='hdl_localization', executable='hdl_localization_map_server',
            name='globalmap_server', output='screen',
            parameters=[params_file, {
                'globalmap_pcd': globalmap_pcd,
                'globalmap_topic': LaunchConfiguration('globalmap_topic'),
                'downsample_resolution': LaunchConfiguration('downsample_resolution'),
            }]),
        Node(
            package='hdl_localization', executable='hdl_localization_node',
            name='hdl_localization', output='screen',
            parameters=[params_file, {
                'points_topic': LaunchConfiguration('points_topic'),
                'globalmap_topic': LaunchConfiguration('globalmap_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'robot_odom_frame_id': LaunchConfiguration('odom_frame'),
                'odom_child_frame_id': LaunchConfiguration('base_frame'),
                'cloud_qos_depth': LaunchConfiguration('cloud_qos_depth'),
                'odom_qos_depth': LaunchConfiguration('odom_qos_depth'),
                'tf_lookup_timeout_seconds': LaunchConfiguration('tf_lookup_timeout_seconds'),
                'max_prediction_age_seconds': LaunchConfiguration('max_prediction_age_seconds'),
                'ndt_resolution': LaunchConfiguration('ndt_resolution'),
                'downsample_resolution': LaunchConfiguration('downsample_resolution'),
                'use_imu': False,
            }]),
    ])
