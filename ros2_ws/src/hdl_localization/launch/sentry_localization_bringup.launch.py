# 此文件用于统一启动 Livox、左 MID360 Point-LIO、点云处理和 HDL，形成 map 到 odom 到 base_link 定位链。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# 此函数用于组合传感器、里程计、点云处理和 HDL 定位的启动描述；输入为 launch 参数，输出为完整定位链，副作用是启动多个 ROS2 节点。
def generate_launch_description():
    cloud_launch = os.path.join(
        get_package_share_directory('livox_cloudpoint_processor'),
        'launch', 'dual_mid360_cloud.launch.py')
    point_lio_launch = os.path.join(
        get_package_share_directory('point_lio'),
        'launch', 'sentry_left_mid360.launch.py')
    hdl_launch = os.path.join(
        get_package_share_directory('hdl_localization'),
        'launch', 'sentry_localization.launch.py')
    point_lio_params = os.path.join(
        get_package_share_directory('point_lio'), 'config', 'sentry_left_mid360.yaml')
    hdl_params = os.path.join(
        get_package_share_directory('hdl_localization'), 'config', 'sentry_left_mid360.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'globalmap_pcd',
            description='必填：外部全局地图 PCD 的绝对路径；不会随仓库提交。'),
        DeclareLaunchArgument('left_lidar_topic', default_value='/livox/lidar_192_168_1_3'),
        DeclareLaunchArgument('left_imu_topic', default_value='/livox/imu_192_168_1_3'),
        DeclareLaunchArgument('right_lidar_topic', default_value='/livox/lidar_192_168_1_105'),
        DeclareLaunchArgument('enable_dual_lidar_fusion', default_value='false'),
        DeclareLaunchArgument('point_lio_params_file', default_value=point_lio_params),
        DeclareLaunchArgument('hdl_params_file', default_value=hdl_params),
        DeclareLaunchArgument('driver_config_path', default_value=os.path.join(
            get_package_share_directory('livox_cloudpoint_processor'),
            'config', 'dual_mid360_driver.json')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cloud_launch),
            launch_arguments={
                'driver_config_path': LaunchConfiguration('driver_config_path'),
                'left_topic': LaunchConfiguration('left_lidar_topic'),
                'right_topic': LaunchConfiguration('right_lidar_topic'),
                'enable_dual_lidar_fusion': LaunchConfiguration('enable_dual_lidar_fusion'),
                'frame_id': 'base_link',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(point_lio_launch),
            launch_arguments={
                'params_file': LaunchConfiguration('point_lio_params_file'),
                'lidar_topic': LaunchConfiguration('left_lidar_topic'),
                'imu_topic': LaunchConfiguration('left_imu_topic'),
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hdl_launch),
            launch_arguments={
                'params_file': LaunchConfiguration('hdl_params_file'),
                'globalmap_pcd': LaunchConfiguration('globalmap_pcd'),
            }.items()),
    ])
