# 此文件用于同时启动 ROS2 Livox 双 MID360 驱动与兼容旧话题的点云处理节点。
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 此函数用于创建双 MID360 驱动和点云处理节点的启动描述；无输入，输出为 LaunchDescription。
def generate_launch_description():
    processor_share = get_package_share_directory('livox_cloudpoint_processor')
    default_params = os.path.join(processor_share, 'config', 'dual_mid360_cloud.yaml')
    default_driver_config = os.path.join(processor_share, 'config', 'dual_mid360_driver.json')

    driver_config_argument = DeclareLaunchArgument('driver_config_path', default_value=default_driver_config, description='双 MID360 的 Livox SDK JSON 配置路径。')
    params_argument = DeclareLaunchArgument('processor_params_path', default_value=default_params, description='点云处理节点参数 YAML 路径。')
    left_topic_argument = DeclareLaunchArgument('left_topic', default_value='/livox/lidar_192_168_1_3', description='左侧 MID360 CustomMsg 话题。')
    right_topic_argument = DeclareLaunchArgument('right_topic', default_value='/livox/lidar_192_168_1_105', description='右侧 MID360 CustomMsg 话题。')
    dual_lidar_fusion_argument = DeclareLaunchArgument("enable_dual_lidar_fusion", default_value="false", description="是否启用需要实测右雷达外参的双雷达融合。")
    fused_custom_topic_argument = DeclareLaunchArgument('fused_custom_topic', default_value='/livox/fused_custom', description='启用双融合后发布的统一 Livox CustomMsg 话题。')
    dual_sync_tolerance_argument = DeclareLaunchArgument('dual_sync_tolerance_ms', default_value='10.0', description='双 MID360 允许的最大 timebase 时间差，单位毫秒。')
    dual_cache_seconds_argument = DeclareLaunchArgument('dual_cache_seconds', default_value='0.2', description='双 MID360 等待配对帧的缓存时长，单位秒。')
    dual_drop_policy_argument = DeclareLaunchArgument('dual_drop_policy', default_value='drop_older', description='时间不匹配时的丢帧策略：drop_older 或 drop_pair。')
    right_translation_argument = DeclareLaunchArgument('right_extrinsic_translation', default_value='[0.0, 0.0, 0.0]', description='右 MID360 到左 MID360 的平移外参 [m]。')
    right_rotation_argument = DeclareLaunchArgument('right_extrinsic_rotation', default_value='[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]', description='右 MID360 到左 MID360 的行主序旋转矩阵。')
    input_qos_depth_argument = DeclareLaunchArgument('input_qos_depth', default_value='1', description='Livox CustomMsg 订阅和融合输出的 QoS 队列深度。')
    raw_topic_argument = DeclareLaunchArgument('raw_topic', default_value='/lidar_3d', description='原始合并点云话题。')
    filtered_topic_argument = DeclareLaunchArgument('filtered_topic', default_value='/filted_topic_3d', description='过滤后三维点云话题。')
    grid_topic_argument = DeclareLaunchArgument('grid_topic', default_value='/grid', description='输出占用栅格话题。')
    frame_id_argument = DeclareLaunchArgument('frame_id', default_value='base_link', description='处理后点云和栅格的坐标系。')

    driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 1,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': LaunchConfiguration('frame_id'),
            'user_config_path': LaunchConfiguration('driver_config_path'),
        }],
    )
    processor = Node(
        package='livox_cloudpoint_processor',
        executable='livox_cloudpoint_processor_node',
        name='threeD_lidar_filter_pointcloud',
        output='screen',
        parameters=[
            LaunchConfiguration('processor_params_path'),
            {
                'left_topic': LaunchConfiguration('left_topic'),
                'right_topic': LaunchConfiguration('right_topic'),
                "enable_dual_lidar_fusion": LaunchConfiguration("enable_dual_lidar_fusion"),
                'fused_custom_topic': LaunchConfiguration('fused_custom_topic'),
                'dual_sync_tolerance_ms': LaunchConfiguration('dual_sync_tolerance_ms'),
                'dual_cache_seconds': LaunchConfiguration('dual_cache_seconds'),
                'dual_drop_policy': LaunchConfiguration('dual_drop_policy'),
                'right_extrinsic_translation': LaunchConfiguration('right_extrinsic_translation'),
                'right_extrinsic_rotation': LaunchConfiguration('right_extrinsic_rotation'),
                'input_qos_depth': LaunchConfiguration('input_qos_depth'),
                'raw_topic': LaunchConfiguration('raw_topic'),
                'filtered_topic': LaunchConfiguration('filtered_topic'),
                'grid_topic': LaunchConfiguration('grid_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
            },
        ],
    )
    return LaunchDescription([
        driver_config_argument,
        params_argument,
        left_topic_argument,
        right_topic_argument,
        dual_lidar_fusion_argument,
        fused_custom_topic_argument,
        dual_sync_tolerance_argument,
        dual_cache_seconds_argument,
        dual_drop_policy_argument,
        right_translation_argument,
        right_rotation_argument,
        input_qos_depth_argument,
        raw_topic_argument,
        filtered_topic_argument,
        grid_topic_argument,
        frame_id_argument,
        driver,
        processor,
    ])
