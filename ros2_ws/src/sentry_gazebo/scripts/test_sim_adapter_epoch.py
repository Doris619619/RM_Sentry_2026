#!/usr/bin/env python3
"""Isolated contracts for native-odometry startup/reset filtering; not motion acceptance."""
import copy,unittest
from unittest.mock import Mock
import rclpy
from nav_msgs.msg import Odometry
from sim_adapter import SimAdapter

class EpochTests(unittest.TestCase):
    # Capture output without a running simulator or any public-domain publishers.
    def setUp(self):
        self.node=SimAdapter()
        self.node.odom_pub.publish=Mock()
        self.node.tf.sendTransform=Mock()

    # Release all DDS resources created by the contract fixture.
    def tearDown(self):
        self.node.destroy_node()

    # Build a labelled native fixture and advance its independent simulation timestamp.
    def sample(self, nanoseconds, speed=0.):
        m=Odometry();m.header.stamp.sec=nanoseconds//1000000000
        m.header.stamp.nanosec=nanoseconds%1000000000
        m.pose.pose.orientation.w=1.;m.pose.pose.position.x=-.919;m.pose.pose.position.y=-4.454
        m.twist.twist.linear.x=speed
        self.node.sim_ns=nanoseconds
        return m

    # Initial finite-difference rolling window is dropped, never replaced with invented zero.
    def test_startup_window_preserves_later_payload(self):
        first=self.sample(20000000,454.78);self.node.on_odom(first)
        self.node.odom_pub.publish.assert_not_called();self.node.tf.sendTransform.assert_not_called()
        self.node.on_odom(self.sample(40000000,151.59))
        self.node.odom_pub.publish.assert_not_called()
        second=self.sample(220000000,.12);self.node.on_odom(second)
        output=self.node.odom_pub.publish.call_args.args[0]
        self.assertEqual(output.header.stamp,second.header.stamp)
        self.assertEqual(output.twist,second.twist)
        self.assertEqual(output.pose,second.pose)
        self.assertEqual(output.header.frame_id,'map')

    # Clock rollback discards the startup window again and preserves later native samples.
    def test_reset_window_then_recovers(self):
        self.node.on_odom(self.sample(5000000000))
        self.node.on_odom(self.sample(5220000000,.1))
        count=self.node.odom_pub.publish.call_count
        self.node.on_odom(self.sample(20000000,454.78))
        self.assertEqual(self.node.odom_pub.publish.call_count,count)
        self.node.on_odom(self.sample(220000000))
        self.assertEqual(self.node.odom_pub.publish.call_count,count+1)

    # An old-epoch payload arriving after /clock resets must never refill downstream TF caches.
    def test_delayed_old_epoch_rejected(self):
        old=self.sample(5020000000,.2)
        self.node.sim_ns=20000000
        self.node.on_odom(old)
        self.node.odom_pub.publish.assert_not_called();self.node.tf.sendTransform.assert_not_called()

if __name__=='__main__':
    rclpy.init()
    try:unittest.main()
    finally:rclpy.shutdown()
