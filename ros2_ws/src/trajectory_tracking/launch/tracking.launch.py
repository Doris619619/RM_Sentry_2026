import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tracking_share = get_package_share_directory("trajectory_tracking")
    planning_share = get_package_share_directory("trajectory_generation")
    tracking_params = os.path.join(tracking_share, "config", "tracking.yaml")
    map_metadata = os.path.join(planning_share, "config", "map_metadata.yaml")
    return LaunchDescription([
        Node(package="trajectory_tracking", executable="trajectory_tracking_node",
             name="trajectory_tracking", output="screen", parameters=[tracking_params, map_metadata]),
        Node(package="trajectory_tracking", executable="hit_bridge",
             name="hit_bridge", output="screen"),
    ])
