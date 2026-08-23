# 此文件用于统一启动 Livox、左 MID360 Point-LIO、点云处理和 HDL，形成 map 到 odom 到 base_link 定位链。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# 此函数用于组合传感器、里程计、点云处理和 HDL 定位的启动描述；输入为 launch 参数，输出为完整定位链，副作用是启动多个 ROS2 节点。
def generate_launch_description():
    cloud_share = get_package_share_directory('livox_cloudpoint_processor')
    point_lio_share = get_package_share_directory('point_lio')
    hdl_share = get_package_share_directory('hdl_localization')
    cloud_launch = os.path.join(cloud_share, 'launch', 'dual_mid360_cloud.launch.py')
    point_lio_launch = os.path.join(point_lio_share, 'launch', 'sentry_left_mid360.launch.py')
    hdl_launch = os.path.join(hdl_share, 'launch', 'sentry_localization.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('globalmap_pcd', description='必填：外部全局地图 PCD 的绝对路径；不会随仓库提交。'),
        DeclareLaunchArgument('driver_config_path', default_value=os.path.join(cloud_share, 'config', 'dual_mid360_driver.json'), description='Livox 驱动 JSON 配置。'),
        DeclareLaunchArgument('cloud_params_file', default_value=os.path.join(cloud_share, 'config', 'dual_mid360_cloud.yaml'), description='点云处理参数 YAML。'),
        DeclareLaunchArgument('point_lio_params_file', default_value=os.path.join(point_lio_share, 'config', 'sentry_left_mid360.yaml'), description='Point-LIO 参数 YAML。'),
        DeclareLaunchArgument('hdl_params_file', default_value=os.path.join(hdl_share, 'config', 'sentry_left_mid360.yaml'), description='HDL 参数 YAML。'),
        DeclareLaunchArgument('left_lidar_topic', default_value='/livox/lidar_192_168_1_3', description='左 MID360 CustomMsg 话题。'),
        DeclareLaunchArgument('left_imu_topic', default_value='/livox/imu_192_168_1_3', description='左 MID360 IMU 话题。'),
        DeclareLaunchArgument('right_lidar_topic', default_value='/livox/lidar_192_168_1_105', description='右 MID360 CustomMsg 话题。'),
        DeclareLaunchArgument('enable_dual_lidar_fusion', default_value='false', description='是否启用需实测外参验证的双 MID360 融合。'),
        DeclareLaunchArgument('fused_custom_topic', default_value='/livox/fused_custom', description='双融合重建后的 Livox CustomMsg 话题。'),
        DeclareLaunchArgument('dual_sync_tolerance_ms', default_value='10.0', description='双雷达允许 timebase 差，单位毫秒。'),
        DeclareLaunchArgument('dual_cache_seconds', default_value='0.2', description='双雷达配对缓存时长，单位秒。'),
        DeclareLaunchArgument('dual_drop_policy', default_value='drop_older', description='不同步帧策略：drop_older 或 drop_pair。'),
        DeclareLaunchArgument('right_extrinsic_translation', default_value='[0.0, 0.0, 0.0]', description='右雷达到左雷达的平移外参。'),
        DeclareLaunchArgument('right_extrinsic_rotation', default_value='[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]', description='右雷达到左雷达的行主序旋转矩阵。'),
        DeclareLaunchArgument('cloud_input_qos_depth', default_value='1', description='点云处理 Livox 输入 QoS 深度。'),
        DeclareLaunchArgument('raw_topic', default_value='/lidar_3d', description='原始点云输出话题。'),
        DeclareLaunchArgument('filtered_topic', default_value='/filted_topic_3d', description='供 HDL 使用的过滤点云话题。'),
        DeclareLaunchArgument('globalmap_topic', default_value='/globalmap', description='地图服务器发布给 HDL 的全局点云话题。'),
        DeclareLaunchArgument('grid_topic', default_value='/grid', description='占用栅格输出话题。'),
        DeclareLaunchArgument('odom_frame', default_value='odom', description='Point-LIO 里程计父坐标系。'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='左 MID360 IMU 对应机体坐标系。'),
        DeclareLaunchArgument('lidar_qos_depth', default_value='5', description='Point-LIO LiDAR QoS 深度。'),
        DeclareLaunchArgument('imu_qos_depth', default_value='0', description='Point-LIO IMU QoS 深度，0 自动计算。'),
        DeclareLaunchArgument('imu_buffer_seconds', default_value='3.0', description='Point-LIO IMU 缓存秒数。'),
        DeclareLaunchArgument('imu_expected_frequency_hz', default_value='200.0', description='Point-LIO IMU 预期频率。'),
        DeclareLaunchArgument('point_lio_odom_topic', default_value='/point_lio/odometry', description='Point-LIO 供 HDL 运动预测使用的里程计话题。'),
        DeclareLaunchArgument('hdl_cloud_qos_depth', default_value='5', description='HDL 点云 QoS 深度。'),
        DeclareLaunchArgument('hdl_odom_qos_depth', default_value='20', description='HDL 里程计 QoS 深度。'),
        DeclareLaunchArgument('tf_lookup_timeout_seconds', default_value='0.1', description='HDL 查询 odom 到 base_link 的最大等待秒数。'),
        DeclareLaunchArgument('max_prediction_age_seconds', default_value='0.2', description='HDL 使用 Point-LIO 预测的最大时效秒数。'),
        DeclareLaunchArgument('ndt_resolution', default_value='1.0', description='CPU NDT_OMP 分辨率。'),
        DeclareLaunchArgument('downsample_resolution', default_value='0.1', description='HDL 与地图服务器下采样分辨率。'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cloud_launch),
            launch_arguments={
                'driver_config_path': LaunchConfiguration('driver_config_path'),
                'processor_params_path': LaunchConfiguration('cloud_params_file'),
                'left_topic': LaunchConfiguration('left_lidar_topic'),
                'right_topic': LaunchConfiguration('right_lidar_topic'),
                'enable_dual_lidar_fusion': LaunchConfiguration('enable_dual_lidar_fusion'),
                'fused_custom_topic': LaunchConfiguration('fused_custom_topic'),
                'dual_sync_tolerance_ms': LaunchConfiguration('dual_sync_tolerance_ms'),
                'dual_cache_seconds': LaunchConfiguration('dual_cache_seconds'),
                'dual_drop_policy': LaunchConfiguration('dual_drop_policy'),
                'right_extrinsic_translation': LaunchConfiguration('right_extrinsic_translation'),
                'right_extrinsic_rotation': LaunchConfiguration('right_extrinsic_rotation'),
                'input_qos_depth': LaunchConfiguration('cloud_input_qos_depth'),
                'raw_topic': LaunchConfiguration('raw_topic'),
                'filtered_topic': LaunchConfiguration('filtered_topic'),
                'grid_topic': LaunchConfiguration('grid_topic'),
                'frame_id': LaunchConfiguration('base_frame'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(point_lio_launch),
            launch_arguments={
                'params_file': LaunchConfiguration('point_lio_params_file'),
                'lidar_topic': LaunchConfiguration('left_lidar_topic'),
                'imu_topic': LaunchConfiguration('left_imu_topic'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'lidar_qos_depth': LaunchConfiguration('lidar_qos_depth'),
                'imu_qos_depth': LaunchConfiguration('imu_qos_depth'),
                'imu_buffer_seconds': LaunchConfiguration('imu_buffer_seconds'),
                'imu_expected_frequency_hz': LaunchConfiguration('imu_expected_frequency_hz'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hdl_launch),
            launch_arguments={
                'params_file': LaunchConfiguration('hdl_params_file'),
                'globalmap_pcd': LaunchConfiguration('globalmap_pcd'),
                'points_topic': LaunchConfiguration('filtered_topic'),
                'globalmap_topic': LaunchConfiguration('globalmap_topic'),
                'odom_topic': LaunchConfiguration('point_lio_odom_topic'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'cloud_qos_depth': LaunchConfiguration('hdl_cloud_qos_depth'),
                'odom_qos_depth': LaunchConfiguration('hdl_odom_qos_depth'),
                'tf_lookup_timeout_seconds': LaunchConfiguration('tf_lookup_timeout_seconds'),
                'max_prediction_age_seconds': LaunchConfiguration('max_prediction_age_seconds'),
                'ndt_resolution': LaunchConfiguration('ndt_resolution'),
                'downsample_resolution': LaunchConfiguration('downsample_resolution'),
            }.items()),
    ])
