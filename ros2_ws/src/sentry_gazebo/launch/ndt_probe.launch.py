"""Probe the existing CPU HDL/NDT with real lidar and a known prior, without consuming ground-truth odometry."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler,EmitEvent,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    """Keep all estimator outputs private for first-stage independent accuracy measurements."""
    share=Path(get_package_share_directory('sentry_gazebo'))
    common={'use_sim_time':True}
    nodes=[
        Node(package='hdl_localization',executable='hdl_localization_map_server',name='sim_ndt_map',
            parameters=[common,{'globalmap_pcd':str(share/'maps/ndt_reference.pcd'),'globalmap_topic':'/sim/ndt/map',
            'downsample_resolution':.05,'convert_utm_to_local':False}],output='screen'),
        Node(package='hdl_localization',executable='hdl_localization_node',name='sim_ndt',
            additional_env={'OMP_NUM_THREADS':'2'},
            parameters=[common,{'points_topic':'/filted_topic_3d','globalmap_topic':'/sim/ndt/map',
                'odom_topic':'/sim/ndt/unused_prediction','imu_topic':'/sim/ndt/unused_imu',
                'send_tf_transforms':False,'enable_robot_odometry_prediction':False,'use_imu':False,
                'use_global_localization':False,'reg_method':'NDT_OMP','ndt_resolution':.3,'ndt_neighbor_search_method':LaunchConfiguration('ndt_search'),
                'downsample_resolution':.05,'specify_init_pose':True,
                'init_pos_x':-.769,'init_pos_y':-4.354,'init_pos_z':0.,
                'init_ori_w':1.,'init_ori_x':0.,'init_ori_y':0.,'init_ori_z':0.}],
            remappings=[('/localization/odometry','/sim/ndt/odometry'),('/aligned_points','/sim/ndt/aligned'),
                        ('/status','/sim/ndt/status')],output='screen')]
    handlers=[RegisterEventHandler(OnProcessExit(target_action=n,on_exit=[EmitEvent(event=Shutdown(reason='NDT component exited'))])) for n in nodes]
    return LaunchDescription([DeclareLaunchArgument('ndt_search',default_value='DIRECT7',choices=['DIRECT7','KDTREE'])]+handlers+nodes)
