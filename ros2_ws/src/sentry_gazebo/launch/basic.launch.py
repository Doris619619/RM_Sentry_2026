from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    share = Path(get_package_share_directory('sentry_gazebo'))
    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-v', '3',
                                 str(share / 'worlds/basic.sdf')], output='screen')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  name='sentry_gazebo_bridge', parameters=[
                      {'config_file': str(share / 'config/bridge.yaml'), 'use_sim_time': True}],
                  output='screen')
    adapter = Node(package='sentry_gazebo', executable='sim_adapter.py',
                   parameters=[{'use_sim_time': True}], output='screen')
    actions = [SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', str(Path(get_package_prefix('sentry_gazebo')) / 'lib')),
               SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', str(share / 'models')),
               SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')]
    for process in (gazebo, bridge, adapter):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Simulation component exited'))])))
    return LaunchDescription(actions + [gazebo, bridge, adapter])
