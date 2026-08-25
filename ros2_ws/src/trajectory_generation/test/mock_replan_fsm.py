#!/usr/bin/env python3
"""Software-only ReplanFSM acceptance client.

Start trajectory_generator_node first, then run this program in the same ROS2
domain.  It uses the deterministic free-space pair selected from occtopo.png.
"""

import argparse
import json
import math
import struct
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from trajectory_generation.msg import TrajectoryPoly


class ReplanFsmMock(Node):
    def __init__(self):
        super().__init__('replan_fsm_mock')
        self.trajectories = []
        self.odom_pub = self.create_publisher(Odometry, '/localization/odometry', 10)
        self.goal_pub = self.create_publisher(PointStamped, '/clicked_point', 10)
        self.replan_pub = self.create_publisher(Bool, '/replan_flag', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/filted_topic_3d', 10)
        self.tf_static_pub = self.create_publisher(
            TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                  reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(TrajectoryPoly, '/global_trajectory', self.on_trajectory, 10)

    def on_trajectory(self, message):
        assert message.duration
        assert len(message.coef_x) == 4 * len(message.duration)
        assert len(message.coef_y) == 4 * len(message.duration)
        assert all(math.isfinite(value) for value in message.coef_x + message.coef_y + message.duration)
        assert all(value > 0.0 for value in message.duration)
        self.trajectories.append(message)

    def wait_for(self, count, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.trajectories) >= count:
                return
        raise RuntimeError(f'timed out waiting for trajectory {count}; got {len(self.trajectories)}')

    def wait_for_subscribers(self, timeout):
        deadline = time.monotonic() + timeout
        publishers = (self.odom_pub, self.goal_pub, self.replan_pub, self.cloud_pub)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(publisher.get_subscription_count() > 0 for publisher in publishers):
                return
        raise RuntimeError('planner subscriptions were not discovered')

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_odom(self):
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = -3.82
        odom.pose.pose.position.y = 2.40
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

    def publish_missing_tf_cloud(self):
        cloud = PointCloud2()
        cloud.header.frame_id = 'missing_cloud'
        cloud.height = 1
        cloud.width = 0
        self.cloud_pub.publish(cloud)

    def publish_static_transform(self):
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'test_lidar'
        transform.transform.rotation.w = 1.0
        self.tf_static_pub.publish(TFMessage(transforms=[transform]))

    def publish_transformed_dynamic_cloud(self):
        cloud = PointCloud2()
        cloud.header.frame_id = 'test_lidar'
        cloud.height = 1
        cloud.width = 1
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12
        cloud.is_dense = True
        # It lies within the ROS1 six-metre local-map radius.  Arrival of a
        # TF-valid cloud in EXEC_TRAJ must route through ReplanFSM.
        cloud.data = struct.pack('<fff', -3.50, 1.45, 0.10)
        self.cloud_pub.publish(cloud)

    def publish_initial_inputs(self):
        # The node must have odometry before it processes dynamic clouds.
        self.publish_odom()
        self.spin_for(0.4)
        # A cloud without map <- missing_cloud must be discarded; static-map
        # planning below must still succeed.
        self.publish_missing_tf_cloud()
        self.spin_for(0.4)

        goal = PointStamped()
        goal.header.frame_id = 'map'
        goal.point.x = -1.35
        goal.point.y = -4.20
        self.goal_pub.publish(goal)

    def publish_replan(self, enabled):
        message = Bool()
        message.data = enabled
        self.replan_pub.publish(message)


def trajectory_metrics(message):
    total_time = sum(message.duration)
    length = 0.0
    previous = None
    for segment, duration in enumerate(message.duration):
        for sample in range(21):
            time_value = duration * sample / 20.0
            coefficients_x = message.coef_x[segment * 4:(segment + 1) * 4]
            coefficients_y = message.coef_y[segment * 4:(segment + 1) * 4]
            x = ((coefficients_x[0] * time_value + coefficients_x[1]) * time_value + coefficients_x[2]) * time_value + coefficients_x[3]
            y = ((coefficients_y[0] * time_value + coefficients_y[1]) * time_value + coefficients_y[2]) * time_value + coefficients_y[3]
            if previous is not None:
                length += math.hypot(x - previous[0], y - previous[1])
            previous = (x, y)
    return {'end': [previous[0], previous[1]], 'length': length, 'total_time': total_time,
            'segments': len(message.duration)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline-output')
    arguments = parser.parse_args()
    rclpy.init()
    node = ReplanFsmMock()
    try:
        node.wait_for_subscribers(5.0)
        node.publish_initial_inputs()
        node.wait_for(1, 12.0)
        if arguments.baseline_output:
            with open(arguments.baseline_output, 'w', encoding='utf-8') as output:
                json.dump(trajectory_metrics(node.trajectories[0]), output, indent=2, sort_keys=True)
        # Cooldown starts in ReplanFSM initialisation, matching ROS1.
        time.sleep(1.1)
        node.publish_static_transform()
        node.spin_for(0.4)
        node.publish_transformed_dynamic_cloud()
        node.wait_for(2, 12.0)
        node.publish_replan(False)
        node.publish_replan(True)
        # Must be ignored by the one-second cooldown after the cloud replan.
        time.sleep(0.2)
        rclpy.spin_once(node, timeout_sec=0.1)
        assert len(node.trajectories) == 2
        time.sleep(1.1)
        node.publish_replan(True)
        node.wait_for(3, 12.0)
        print(f'PASS trajectories={len(node.trajectories)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(f'FAIL: {error}', file=sys.stderr)
        sys.exit(1)
