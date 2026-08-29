from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="trajectory_tracking", executable="trajectory_tracking_node",
             name="trajectory_tracking", output="screen"),
        Node(package="trajectory_tracking", executable="hit_bridge",
             name="hit_bridge", output="screen"),
    ])
