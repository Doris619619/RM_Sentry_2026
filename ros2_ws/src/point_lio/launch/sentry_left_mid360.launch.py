# 此文件用于启动只消费左 MID360 CustomMsg 与左侧 IMU 的 Point-LIO 节点。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 此函数用于创建左 MID360 Point-LIO 启动描述；输入为可覆盖的话题和参数路径，输出为 LaunchDescription。
def generate_launch_description():
    package_share = get_package_share_directory('point_lio')
    default_params = os.path.join(package_share, 'config', 'sentry_left_mid360.yaml')

    params_argument = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='左 MID360 Point-LIO 参数文件路径。')
    lidar_topic_argument = DeclareLaunchArgument(
        'lidar_topic', default_value='/livox/lidar_192_168_1_3',
        description='左 MID360 Livox CustomMsg 话题。')
    imu_topic_argument = DeclareLaunchArgument(
        'imu_topic', default_value='/livox/imu_192_168_1_3',
        description='左 MID360 IMU 话题。')
    odom_frame_argument = DeclareLaunchArgument(
        'odom_frame', default_value='odom', description='Point-LIO 里程计父坐标系。')
    base_frame_argument = DeclareLaunchArgument(
        'base_frame', default_value='base_link', description='左 MID360 IMU 对应的机体坐标系。')
    lidar_qos_depth_argument = DeclareLaunchArgument(
        'lidar_qos_depth', default_value='5', description='Livox CustomMsg 订阅 QoS 队列深度。')
    imu_qos_depth_argument = DeclareLaunchArgument(
        'imu_qos_depth', default_value='0', description='IMU QoS 队列深度；0 时按缓存秒数和频率计算。')
    imu_buffer_seconds_argument = DeclareLaunchArgument(
        'imu_buffer_seconds', default_value='3.0', description='IMU 缓存时长，单位秒。')
    imu_frequency_argument = DeclareLaunchArgument(
        'imu_expected_frequency_hz', default_value='200.0', description='用于计算 IMU 缓存容量的预期频率。')

    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'common.lid_topic': LaunchConfiguration('lidar_topic'),
                'common.imu_topic': LaunchConfiguration('imu_topic'),
                'odom_header_frame_id': LaunchConfiguration('odom_frame'),
                'odom_child_frame_id': LaunchConfiguration('base_frame'),
                'communication.lidar_qos_depth': LaunchConfiguration('lidar_qos_depth'),
                'communication.imu_qos_depth': LaunchConfiguration('imu_qos_depth'),
                'communication.imu_buffer_seconds': LaunchConfiguration('imu_buffer_seconds'),
                'communication.imu_expected_frequency_hz': LaunchConfiguration('imu_expected_frequency_hz'),
            },
        ],
    )

    return LaunchDescription([
        params_argument, lidar_topic_argument, imu_topic_argument,
        odom_frame_argument, base_frame_argument,
        lidar_qos_depth_argument, imu_qos_depth_argument,
        imu_buffer_seconds_argument, imu_frequency_argument,
        point_lio_node,
    ])
