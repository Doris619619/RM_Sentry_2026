"""Launch staged navigation or the complete simulated serial/decision/physical navigation chain."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess,SetEnvironmentVariable,RegisterEventHandler,EmitEvent,DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

# Choose a resource without evaluating launch substitutions before launch context exists.
def resource(flag,yes,no):
    return PythonExpression(["'",str(yes),"' if '",LaunchConfiguration(flag),"' == 'true' else '",str(no),"'"])

# The full-system label must always include the actual estimator and force/contact model.
def validate(context):
    if LaunchConfiguration('full_system').perform(context)=='true':
        if LaunchConfiguration('localization').perform(context)!='ndt' or LaunchConfiguration('physics').perform(context)!='true':
            raise RuntimeError('full_system requires localization:=ndt and physics:=true')
    return []

def generate_launch_description():
    """Preserve independent truth, NDT, and full-system modes with one exclusive actuator output."""
    share=Path(get_package_share_directory('sentry_gazebo'))
    planner_share=Path(get_package_share_directory('trajectory_generation'))
    tracking_share=Path(get_package_share_directory('trajectory_tracking'))
    decision_share=Path(get_package_share_directory('decision_node'))
    ndt_condition=IfCondition(PythonExpression(["'",LaunchConfiguration('localization'),"' == 'ndt'"]))
    full_condition=IfCondition(LaunchConfiguration('full_system'))
    truth_enabled=ParameterValue(PythonExpression(["'",LaunchConfiguration('localization'),"' == 'truth'"]),value_type=bool)
    world=resource('physics',share/'worlds/planning_physics.sdf',share/'worlds/planning.sdf')
    gazebo=ExecuteProcess(cmd=['ign','gazebo','-r','-v','3',world,'--gui-config',str(share/'config/autonomy-gui.config')],output='screen')
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',name='sentry_gazebo_bridge',
        parameters=[{'config_file':resource('full_system',share/'config/full_bridge.yaml',share/'config/lidar_bridge.yaml'),'use_sim_time':True}],output='screen')
    adapter=Node(package='sentry_gazebo',executable='sim_adapter.py',parameters=[{'use_sim_time':True,'publish_truth':truth_enabled}],output='screen')
    cloud=Node(package='sentry_gazebo',executable='cloud_adapter.py',parameters=[{'use_sim_time':True}],output='screen')
    display=Node(package='sentry_gazebo',executable='map_display.py',parameters=[{'use_sim_time':True}],output='screen')
    planner=Node(package='trajectory_generation',executable='trajectory_generator_node',name='trajectory_generation',
        parameters=[str(planner_share/'config/global_planning.yaml'),str(planner_share/'config/map_metadata.yaml'),
                    str(share/'config/autonomy.yaml'),{'use_sim_time':True,'planner.test_random_seed':7,
                    'planner.reference_desire_speed':ParameterValue(PythonExpression(["0.25 if '",LaunchConfiguration('full_system'),"' == 'true' else 0.3"]),value_type=float),
                    'planner.reference_desire_speed_xtl':ParameterValue(PythonExpression(["0.25 if '",LaunchConfiguration('full_system'),"' == 'true' else 0.3"]),value_type=float),
                    'planner.reference_v_max':ParameterValue(PythonExpression(["0.35 if '",LaunchConfiguration('full_system'),"' == 'true' else 0.45"]),value_type=float)}],output='screen',
        condition=IfCondition(LaunchConfiguration('enable_planner')))
    rviz=Node(package='sentry_gazebo',executable='rviz_safe',arguments=['-d',str(share/'config/autonomy.rviz')],parameters=[{'use_sim_time':True}],output='screen')
    tracking=Node(package='trajectory_tracking',executable='trajectory_tracking_node',name='trajectory_tracking',
        parameters=[str(tracking_share/'config/tracking.yaml'),str(planner_share/'config/map_metadata.yaml'),
                    str(share/'config/autonomy.yaml'),{'use_sim_time':True,
                    'output.max_speed':ParameterValue(PythonExpression(["0.35 if '",LaunchConfiguration('full_system'),"' == 'true' else 0.45"]),value_type=float),
                    'arrival.distance':ParameterValue(PythonExpression(["0.06 if '",LaunchConfiguration('localization'),"' == 'ndt' else 0.12"]),value_type=float)}],output='screen')
    hit=Node(package='trajectory_tracking',executable='hit_bridge',parameters=[
        {'use_sim_time':True,'cmd_vel_topic':'/sim/auto_cmd_vel','tracking_arrived_topic':'/sim/arrived'}],output='screen')
    guard=Node(package='sentry_gazebo',executable='autonomy_guard.py',parameters=[{'use_sim_time':True,
        'require_referee':ParameterValue(LaunchConfiguration('full_system'),value_type=bool),
        'arrival_distance':ParameterValue(PythonExpression(["0.08 if '",LaunchConfiguration('localization'),"' == 'ndt' else 0.15"]),value_type=float)}],output='screen')
    estimated=Node(package='sentry_gazebo',executable='estimated_odometry.py',parameters=[{'use_sim_time':True}],condition=ndt_condition,output='screen')
    ndt=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/ndt_probe.launch.py')),condition=ndt_condition,
        launch_arguments={'ndt_search':resource('full_system','KDTREE','DIRECT7')}.items())
    serial=Node(package='sentry_gazebo',executable='pty_mcu_sim.py',parameters=[{'use_sim_time':True}],condition=full_condition,output='screen')
    decision=Node(package='decision_node',executable='strategy_node',name='strategy_node',
        parameters=[str(decision_share/'config/decision.yaml'),str(share/'config/full_decision.yaml'),{'use_sim_time':True}],
        remappings=[('/clicked_point','/sim/decision_goal')],condition=full_condition,output='screen')
    router=Node(package='sentry_gazebo',executable='decision_goal_router.py',parameters=[{'use_sim_time':True}],condition=full_condition,output='screen')
    processes=[estimated,gazebo,bridge,adapter,cloud,display,planner,rviz,tracking,hit,guard,serial,decision,router]
    actions=[DeclareLaunchArgument('localization',default_value='truth',choices=['truth','ndt']),
        DeclareLaunchArgument('physics',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('full_system',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('enable_planner',default_value='true'),OpaqueFunction(function=validate),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',str(Path(get_package_prefix('sentry_gazebo'))/'lib')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',str(share/'models')),SetEnvironmentVariable('QT_QPA_PLATFORM','xcb')]
    for process in processes:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[EmitEvent(event=Shutdown(reason='Simulation component exited'))])))
    layout=ExecuteProcess(cmd=['python3',str(Path(get_package_prefix('sentry_gazebo'))/'lib/sentry_gazebo/gui_layout.py')],output='screen')
    return LaunchDescription(actions+processes+[ndt,layout])
