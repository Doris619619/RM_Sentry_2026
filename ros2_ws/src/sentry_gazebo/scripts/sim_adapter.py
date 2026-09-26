#!/usr/bin/env python3
"""Body-frame command guard and ground-truth localization; never integrate commands."""
import copy
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMsg
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class SimAdapter(Node):
    def __init__(self):
        super().__init__('sentry_sim_adapter')
        self.declare_parameter('publish_truth', True)
        self.publish_truth=bool(self.get_parameter('publish_truth').value)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 0.5)
        self.timeout = float(self.get_parameter('command_timeout').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.timeout, self.max_linear, self.max_angular)):
            raise ValueError('Limits and timeout must be finite and positive')
        self.command = Twist()
        self.last_command = -math.inf
        self.last_progress = -math.inf
        self.sim_ns = None
        self.last_raw_stamp = None
        self.odom_ready_ns = 0
        self.command_pub = self.create_publisher(Twist, '/sim/guarded_cmd_vel', 1)
        self.odom_pub = self.create_publisher(Odometry, '/localization/odometry', 10) if self.publish_truth else None
        self.tf = TransformBroadcaster(self) if self.publish_truth else None
        self.static_tf = StaticTransformBroadcaster(self) if self.publish_truth else None
        identity = TransformStamped()
        identity.header.frame_id = 'map'
        identity.child_frame_id = 'odom'
        identity.transform.rotation.w = 1.0
        if self.publish_truth:self.static_tf.sendTransform(identity)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 1)
        if self.publish_truth:self.create_subscription(Odometry, '/sim/ground_truth/odometry', self.on_odom, 10)
        clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(ClockMsg, '/clock', self.on_clock, clock_qos)
        # Safety MUST keep ticking while simulated time is paused.
        self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('0.5 s command watchdog; truth localization '+('enabled' if self.publish_truth else 'disabled'))

    def clear(self):
        self.command = Twist()
        self.last_command = -math.inf

    def on_clock(self, msg):
        now = time.monotonic()
        ns = msg.clock.sec * 1000000000 + msg.clock.nanosec
        if self.sim_ns is None or ns != self.sim_ns:
            if now - self.last_progress > 0.2 or (self.sim_ns is not None and ns < self.sim_ns):
                self.clear()
            self.last_progress = now
        self.sim_ns = ns

    def on_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        now = time.monotonic()
        if not all(math.isfinite(v) for v in values) or now - self.last_progress > 0.2:
            self.clear()
            self.command_pub.publish(Twist())
            return
        out = Twist()
        speed = math.hypot(msg.linear.x, msg.linear.y)
        scale = min(1.0, self.max_linear / speed) if speed else 1.0
        out.linear.x = msg.linear.x * scale
        out.linear.y = msg.linear.y * scale
        out.angular.z = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        self.command = out
        self.last_command = now

    def tick(self):
        now = time.monotonic()
        if now - self.last_command > self.timeout or now - self.last_progress > 0.2:
            self.clear()
        self.command_pub.publish(self.command)

    def on_odom(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        if not all(math.isfinite(x) for x in (
                p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                v.linear.x, v.linear.y, v.linear.z,
                v.angular.x, v.angular.y, v.angular.z)):
            self.clear()
            self.get_logger().error('Rejected non-finite Gazebo odometry')
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if abs(norm - 1.0) > 0.01:
            self.clear()
            return
        stamp_ns = msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec
        if self.sim_ns is None or not -.5e9 <= stamp_ns-self.sim_ns <= .05e9:
            self.clear()
            return
        first_or_reset = self.last_raw_stamp is None or stamp_ns <= self.last_raw_stamp
        self.last_raw_stamp = stamp_ns
        if first_or_reset:
            # Fortress averages ten physics samples; wait 0.2 simulated seconds
            # so its initial origin-to-spawn derivative has left the native window.
            self.odom_ready_ns = stamp_ns + 200000000
        if stamp_ns < self.odom_ready_ns:
            self.clear()
            self.command_pub.publish(Twist())
            if first_or_reset:
                self.get_logger().info('Waiting 0.2 simulated seconds for native odometry startup window')
            return
        odom = copy.deepcopy(msg)
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        # Fortress OdometryPublisher dimensions=2 already expresses twist in body axes.
        self.odom_pub.publish(odom)
        tf = TransformStamped()
        tf.header = copy.deepcopy(odom.header)
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = p.x
        tf.transform.translation.y = p.y
        tf.transform.translation.z = p.z
        tf.transform.rotation = q
        self.tf.sendTransform(tf)


def main():
    """Run safety/odometry adaptation and treat ROS signal shutdown as a normal exit."""
    rclpy.init()
    node = SimAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.command_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
