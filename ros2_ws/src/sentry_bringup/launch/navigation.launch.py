"""Start only planning, MPC and a velocity bridge against caller-supplied localization and maps."""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(get_package_share_directory('trajectory_generation')) / 'python'))
from sentry_map_profile import load_profile


# Validate one profile before starting any process, then remap both algorithms at the same seam.
def start_navigation(context):
    def value(name):
        """Resolve a declared launch argument in the caller's launch context."""
        return LaunchConfiguration(name).perform(context)
    planning = Path(get_package_share_directory('trajectory_generation'))
    tracking = Path(get_package_share_directory('trajectory_tracking'))
    profile = load_profile(value('map_profile'))
    sim_time = value('use_sim_time') == 'true'
    planner = Node(package='trajectory_generation', executable='trajectory_generator_node',
        name='trajectory_generation', output='screen', parameters=[
            str(planning / 'config/global_planning.yaml'), profile.parameters('trajectory_generation'),
            {'use_sim_time': sim_time, 'topics.odom': value('odom_topic'),
             'topics.point_cloud': value('cloud_topic'), 'topics.goal': value('goal_topic'),
             'topics.global_trajectory': value('trajectory_topic')}])
    tracker = Node(package='trajectory_tracking', executable='trajectory_tracking_node',
        name='trajectory_tracking', output='screen', parameters=[
            str(tracking / 'config/tracking.yaml'), profile.parameters('trajectory_tracking'),
            {'use_sim_time': sim_time}], remappings=[
                ('/localization/odometry', value('odom_topic')),
                ('/aligned_points', value('cloud_topic')),
                ('/global_trajectory', value('trajectory_topic'))])
    bridge = Node(package='trajectory_tracking', executable='hit_bridge', output='screen',
        parameters=[{'use_sim_time': sim_time, 'cmd_vel_topic': value('cmd_vel_topic')}])
    processes = [planner, tracker, bridge]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Navigation component exited'))])) for node in processes]
    return handlers + processes


# The default output is an unconnected navigation command, not a physical actuator topic.
def generate_launch_description():
    defaults = {
        'map_profile': get_package_share_directory('trajectory_generation'),
        'odom_topic': '/localization/odometry', 'cloud_topic': '/aligned_points',
        'goal_topic': '/goal', 'trajectory_topic': '/global_trajectory',
        'cmd_vel_topic': '/navigation/cmd_vel', 'use_sim_time': 'false'}
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in defaults.items()] + [OpaqueFunction(function=start_navigation)])
