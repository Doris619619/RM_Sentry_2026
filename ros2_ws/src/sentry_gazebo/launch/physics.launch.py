"""Run a generic force-driven chassis against real collision geometry, gravity and a five-degree ramp."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess,SetEnvironmentVariable,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    """Keep physics verification separate from the ideal-navigation baseline and production hardware nodes."""
    share=Path(get_package_share_directory('sentry_gazebo'));prefix=Path(get_package_prefix('sentry_gazebo'))
    gazebo=ExecuteProcess(cmd=['ign','gazebo','-r','-v','3',str(share/'worlds/physics.sdf'),
        '--gui-config',str(share/'config/physics-gui.config')],output='screen')
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',name='sentry_gazebo_bridge',
        parameters=[{'config_file':str(share/'config/physics_bridge.yaml'),'use_sim_time':True}],output='screen')
    adapter=Node(package='sentry_gazebo',executable='sim_adapter.py',parameters=[{'use_sim_time':True}],output='screen')
    cloud=Node(package='sentry_gazebo',executable='cloud_adapter.py',parameters=[{'use_sim_time':True}],output='screen')
    rviz=Node(package='sentry_gazebo',executable='rviz_safe',arguments=['-d',str(share/'config/physics.rviz')],
        parameters=[{'use_sim_time':True}],output='screen')
    processes=[gazebo,bridge,adapter,cloud,rviz]
    actions=[SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',str(prefix/'lib')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',str(share/'models')),SetEnvironmentVariable('QT_QPA_PLATFORM','xcb')]
    for process in processes:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason='Physics simulation component exited'))])))
    layout=ExecuteProcess(cmd=['python3',str(prefix/'lib/sentry_gazebo/gui_layout.py')],output='screen')
    return LaunchDescription(actions+processes+[layout])
