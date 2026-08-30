"""Software integration bringup; pair serial_port with a PTY emulator, not hardware."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    decision = get_package_share_directory("decision_node")
    planning = get_package_share_directory("trajectory_generation")
    tracking = get_package_share_directory("trajectory_tracking")
    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB_MOCK"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("decision_params", default_value=decision + "/config/decision.yaml"),
        Node(package="trajectory_generation", executable="trajectory_generator_node", parameters=[
            planning + "/config/global_planning.yaml", planning + "/config/map_metadata.yaml"]),
        Node(package="trajectory_tracking", executable="trajectory_tracking_node", parameters=[
            tracking + "/config/tracking.yaml", planning + "/config/map_metadata.yaml"]),
        Node(package="trajectory_tracking", executable="hit_bridge"),
        Node(package="decision_node", executable="strategy_node",
             parameters=[LaunchConfiguration("decision_params")]),
        Node(package="decision_node", executable="mcu_communicator", parameters=[{
            "serial_port": LaunchConfiguration("serial_port"),
            "baudrate": LaunchConfiguration("baudrate"),
        }]),
    ])
