"""Production topology with explicit hardware and motion opt-in.

Localization/control: Livox+IMU -> cloud processing -> Point-LIO -> HDL ->
planning -> tracking -> /cmd_vel -> MCU/chassis.
Decision loop: MCU/referee/radar -> Decision -> /clicked_point -> planning ->
tracking -> /tracking/arrived,/dstar_status -> Decision.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _launch(package, filename):
    return os.path.join(get_package_share_directory(package), 'launch', filename)


def generate_launch_description():
    globalmap_pcd = LaunchConfiguration('globalmap_pcd')
    enable_mcu = LaunchConfiguration('enable_mcu')
    enable_sensor_pipeline = LaunchConfiguration('enable_sensor_pipeline')
    allow_motion_output = LaunchConfiguration('allow_motion_output')
    safe_mcu_condition = IfCondition(PythonExpression([
        "'", enable_mcu, "' == 'true' and '", allow_motion_output, "' == 'true'"
    ]))
    decision_share = get_package_share_directory('decision_node')
    return LaunchDescription([
        DeclareLaunchArgument('globalmap_pcd', description='Required absolute PCD map path.'),
        DeclareLaunchArgument('enable_sensor_pipeline', default_value='false',
                              description='Start physical Livox, cloud processing, and Point-LIO only when explicitly enabled.'),
        DeclareLaunchArgument('enable_mcu', default_value='false',
                              description='Start the physical MCU bridge only with allow_motion_output=true.'),
        DeclareLaunchArgument('allow_motion_output', default_value='false',
                              description='Explicitly permit physical chassis command output.'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='921600'),
        DeclareLaunchArgument('enable_global_localization', default_value='true'),
        DeclareLaunchArgument('enable_dual_lidar_fusion', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch('livox_cloudpoint_processor', 'dual_mid360_cloud.launch.py')),
            condition=IfCondition(enable_sensor_pipeline),
            launch_arguments={
                'enable_dual_lidar_fusion': LaunchConfiguration('enable_dual_lidar_fusion'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch('point_lio', 'sentry_left_mid360.launch.py')),
            condition=IfCondition(enable_sensor_pipeline)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch('hdl_localization', 'sentry_localization_bringup.launch.py')),
            launch_arguments={
                'globalmap_pcd': globalmap_pcd,
                'enable_dual_lidar_fusion': LaunchConfiguration('enable_dual_lidar_fusion'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch('hdl_global_localization', 'global_localization.launch.py')),
            condition=IfCondition(LaunchConfiguration('enable_global_localization'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(_launch('trajectory_generation', 'global_planning.launch.py')),
                                 launch_arguments={'use_rviz': 'false'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(_launch('trajectory_tracking', 'tracking.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(_launch('decision_node', 'decision.launch.py'))),
        Node(package='decision_node', executable='mcu_communicator', name='mcu_communicator',
             output='screen', condition=safe_mcu_condition,
             parameters=[os.path.join(decision_share, 'config', 'mcu.yaml'), {
                 'serial_port': LaunchConfiguration('serial_port'),
                 'baudrate': LaunchConfiguration('baudrate'),
             }]),
    ])
