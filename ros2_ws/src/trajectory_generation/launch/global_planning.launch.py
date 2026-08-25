from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    planner_share = get_package_share_directory('trajectory_generation')
    waypoint_share = get_package_share_directory('waypoint_generator')
    planner_params = LaunchConfiguration('planner_params')
    waypoint_params = LaunchConfiguration('waypoint_params')
    use_rviz = LaunchConfiguration('use_rviz')
    planner_prefix = LaunchConfiguration('planner_prefix')
    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_params',
            default_value=os.path.join(planner_share, 'config', 'global_planning.yaml')),
        DeclareLaunchArgument(
            'waypoint_params',
            default_value=os.path.join(waypoint_share, 'config', 'waypoint_generator.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('planner_prefix', default_value=''),
        Node(package='waypoint_generator', executable='waypoint_generator_node',
             name='waypoint_generator', output='screen', parameters=[waypoint_params]),
        Node(package='trajectory_generation', executable='trajectory_generator_node',
             name='trajectory_generation', output='screen', parameters=[planner_params], prefix=planner_prefix),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(planner_share, 'rviz', 'global_planning.rviz')],
             condition=IfCondition(use_rviz)),
    ])
