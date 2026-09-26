"""Launch low-speed autonomous simulation with real MPC and exclusive protected control."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess,SetEnvironmentVariable,RegisterEventHandler,EmitEvent,DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    """Keep production defaults isolated; use simulation clock and deterministic QA seed."""
    share=Path(get_package_share_directory('sentry_gazebo'))
    planner_share=Path(get_package_share_directory('trajectory_generation'))
    tracking_share=Path(get_package_share_directory('trajectory_tracking'))
    gazebo=ExecuteProcess(cmd=['ign','gazebo','-r','-v','3',str(share/'worlds/planning.sdf'),'--gui-config',str(share/'config/autonomy-gui.config')],output='screen')
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',name='sentry_gazebo_bridge',
        parameters=[{'config_file':str(share/'config/lidar_bridge.yaml'),'use_sim_time':True}],output='screen')
    adapter=Node(package='sentry_gazebo',executable='sim_adapter.py',parameters=[{'use_sim_time':True}],output='screen')
    cloud=Node(package='sentry_gazebo',executable='cloud_adapter.py',parameters=[{'use_sim_time':True}],output='screen')
    display=Node(package='sentry_gazebo',executable='map_display.py',parameters=[{'use_sim_time':True}],output='screen')
    planner=Node(package='trajectory_generation',executable='trajectory_generator_node',name='trajectory_generation',
        parameters=[str(planner_share/'config/global_planning.yaml'),str(planner_share/'config/map_metadata.yaml'),
                    str(share/'config/autonomy.yaml'),{'use_sim_time':True,'planner.test_random_seed':7}],output='screen',
        condition=IfCondition(LaunchConfiguration('enable_planner')))
    rviz=Node(package='sentry_gazebo',executable='rviz_safe',arguments=['-d',str(share/'config/autonomy.rviz')],parameters=[{'use_sim_time':True}],output='screen')
    tracking=Node(package='trajectory_tracking',executable='trajectory_tracking_node',name='trajectory_tracking',
        parameters=[str(tracking_share/'config/tracking.yaml'),str(planner_share/'config/map_metadata.yaml'),
                    str(share/'config/autonomy.yaml'),{'use_sim_time':True}],output='screen')
    hit=Node(package='trajectory_tracking',executable='hit_bridge',parameters=[
        {'use_sim_time':True,'cmd_vel_topic':'/sim/auto_cmd_vel','tracking_arrived_topic':'/sim/arrived'}],output='screen')
    guard=Node(package='sentry_gazebo',executable='autonomy_guard.py',parameters=[{'use_sim_time':True}],output='screen')
    processes=[gazebo,bridge,adapter,cloud,display,planner,rviz,tracking,hit,guard]
    actions=[DeclareLaunchArgument('enable_planner',default_value='true'),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',str(Path(get_package_prefix('sentry_gazebo'))/'lib')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',str(share/'models')),SetEnvironmentVariable('QT_QPA_PLATFORM','xcb')]
    for process in processes:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[EmitEvent(event=Shutdown(reason='Planning simulation component exited'))])))
    layout=ExecuteProcess(cmd=['python3',str(Path(get_package_prefix('sentry_gazebo'))/'lib/sentry_gazebo/gui_layout.py')],output='screen')
    return LaunchDescription(actions+processes+[layout])
