"""Fixture A: odom/cloud + manual goal must traverse planner and tracker."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_testing
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import PointCloud2
from sentry_msgs.msg import SlaverSpeed
from std_msgs.msg import Bool
from trajectory_generation.msg import TrajectoryPoly


def generate_test_description():
    planning = get_package_share_directory('trajectory_generation')
    tracking = get_package_share_directory('trajectory_tracking')
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='sentry_sim', executable='fixture_inputs', output='screen'),
        launch_ros.actions.Node(package='trajectory_generation', executable='trajectory_generator_node',
                                parameters=[os.path.join(planning, 'config', 'global_planning.yaml'),
                                            os.path.join(planning, 'config', 'map_metadata.yaml')]),
        launch_ros.actions.Node(package='trajectory_tracking', executable='trajectory_tracking_node',
                                parameters=[os.path.join(tracking, 'config', 'tracking.yaml'),
                                            os.path.join(planning, 'config', 'map_metadata.yaml')]),
        launch_ros.actions.Node(package='trajectory_tracking', executable='hit_bridge'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestPlanningControlFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('sentry_planning_control_fixture_client')
        cls.trajectories = []
        cls.clouds = []
        cls.solver_status = []
        cls.speeds = []
        cls.cmd_vel = []
        cls.node.create_subscription(TrajectoryPoly, '/global_trajectory', cls.trajectories.append, 10)
        cls.node.create_subscription(PointCloud2, '/filted_topic_3d', cls.clouds.append, 10)
        cls.node.create_subscription(Bool, '/solver_status', cls.solver_status.append, 10)
        cls.node.create_subscription(SlaverSpeed, '/sentry_des_speed', cls.speeds.append, 10)
        cls.node.create_subscription(Twist, '/cmd_vel', cls.cmd_vel.append, 10)
        cls.goal = cls.node.create_publisher(PoseStamped, '/goal', 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_fixture_goal_planning_tracking(self):
        deadline = time.monotonic() + 30.0
        goal_sent = False
        ready_after = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if not goal_sent and now >= ready_after:
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.pose.position.x = -1.35
                goal.pose.position.y = -4.20
                goal.pose.orientation.w = 1.0
                self.goal.publish(goal)
                goal_sent = True
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (self.clouds and self.trajectories and self.speeds and self.cmd_vel
                    and any(status.data for status in self.solver_status)
                    and any(abs(cmd.linear.x) + abs(cmd.linear.y) > 1e-6 for cmd in self.cmd_vel)):
                break
        self.assertTrue(self.clouds, 'fixture did not publish filtered cloud interface')
        self.assertTrue(self.trajectories, 'manual /goal did not reach planner')
        self.assertTrue(any(msg.duration for msg in self.trajectories),
                        'planner produced malformed trajectory')
        self.assertTrue(any(status.data for status in self.solver_status),
                        'tracking never reported a successful SQP solve')
        self.assertTrue(self.speeds, 'tracking never published /sentry_des_speed')
        self.assertTrue(self.cmd_vel, 'hit_bridge never converted tracking output to /cmd_vel')
        self.assertTrue(any(abs(cmd.linear.x) + abs(cmd.linear.y) > 1e-6 for cmd in self.cmd_vel),
                        'tracking/hit_bridge only produced zero velocity for an unarrived fixture goal')
