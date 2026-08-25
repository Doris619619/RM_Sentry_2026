#!/usr/bin/env python3
"""Software-only acceptance client for the ROS1-equivalent waypoint modes."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class WaypointMock(Node):
    def __init__(self):
        super().__init__('waypoint_generator_mock')
        self.paths = []
        self.visualisations = []
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)
        self.trigger_pub = self.create_publisher(PoseStamped, '/traj_start_trigger', 10)
        self.odom_pub = self.create_publisher(Odometry, '/localization/odometry', 10)
        self.create_subscription(Path, '/waypoint_generator/waypoints', self.paths.append, 10)
        self.create_subscription(PoseArray, '/waypoint_generator/waypoints_vis', self.visualisations.append, 10)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_subscribers(self):
        deadline = time.monotonic() + 5.0
        publishers = (self.goal_pub, self.trigger_pub, self.odom_pub)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if all(publisher.get_subscription_count() > 0 for publisher in publishers):
                return
        raise RuntimeError('waypoint-generator subscriptions were not discovered')

    def wait_for_paths(self, count, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if len(self.paths) >= count:
                return
        raise RuntimeError(f'timed out waiting for path {count}; got {len(self.paths)}')

    def odom(self, yaw=0.0):
        message = Odometry()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = 3.0
        message.pose.pose.position.y = -2.0
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.odom_pub.publish(message)

    def goal(self, x, y, z):
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)

    def trigger(self):
        self.trigger_pub.publish(PoseStamped())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', required=True, choices=('manual', 'noyaw', 'point', 'free', 'circle', 'eight', 'series'))
    arguments = parser.parse_args()
    expected = {'point': 8, 'free': 8, 'circle': 12, 'eight': 16}
    rclpy.init()
    node = WaypointMock()
    try:
        node.wait_for_subscribers()
        node.odom(yaw=0.7)
        node.spin_for(0.2)
        if arguments.mode in expected:
            node.trigger()
            node.wait_for_paths(1)
            path = node.paths[0]
            assert len(path.poses) == expected[arguments.mode]
            if arguments.mode in ('point', 'free'):
                assert abs(path.poses[0].pose.position.x - 14.0) < 1e-6
        elif arguments.mode == 'series':
            node.goal(0.0, 0.0, 0.0)
            node.spin_for(0.1)
            node.odom()
            node.wait_for_paths(1)
            node.spin_for(0.25)
            node.odom()
            node.wait_for_paths(2)
            assert [len(path.poses) for path in node.paths[:2]] == [1, 1]
        else:
            node.goal(1.0, 2.0, 1.0)
            node.spin_for(0.2)
            assert not node.paths
            node.goal(0.0, 0.0, -2.0)
            node.wait_for_paths(1)
            path = node.paths[0]
            assert len(path.poses) == 1
            assert abs(path.poses[0].pose.position.x - 1.0) < 1e-6
            if arguments.mode == 'noyaw':
                yaw = math.atan2(2.0 * path.poses[0].pose.orientation.z * path.poses[0].pose.orientation.w,
                                 1.0 - 2.0 * path.poses[0].pose.orientation.z ** 2)
                assert abs(yaw - 0.7) < 1e-5
            assert node.visualisations and len(node.visualisations[-1].poses) == 2
        print(f'PASS mode={arguments.mode}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(f'FAIL: {error}', file=sys.stderr)
        sys.exit(1)
