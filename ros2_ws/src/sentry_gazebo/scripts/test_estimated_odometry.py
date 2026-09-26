#!/usr/bin/env python3
"""Test estimated-only velocity frames, confidence gating and truth-source isolation in a private ROS domain."""
import math,unittest
from unittest.mock import Mock,patch
import rclpy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from hdl_localization.msg import ScanMatchingStatus
from estimated_odometry import EstimatedOdometry
from sim_adapter import SimAdapter

class EstimateTests(unittest.TestCase):
    # Capture publications; no simulator or fake public runtime topic is needed.
    def setUp(self):
        self.node=EstimatedOdometry();self.node.pub.publish=Mock();self.node.tf.sendTransform=Mock()
        self.now=patch.object(self.node.get_clock(),'now');self.clock=self.now.start()

    def tearDown(self):
        self.now.stop();self.node.destroy_node()

    # Pair synthetic estimator output and matching status at exactly the same test timestamp.
    def sample(self,t,x=0.,angle=0.,inlier=.95,error=.003,z=0.):
        self.clock.return_value=Time(nanoseconds=int(t*1e9))
        m=Odometry();m.header.stamp=Time(nanoseconds=int(t*1e9)).to_msg()
        m.header.frame_id='map';m.child_frame_id='base_link';m.pose.pose.position.x=x;m.pose.pose.position.z=z
        m.pose.pose.orientation.z=math.sin(angle/2);m.pose.pose.orientation.w=math.cos(angle/2)
        quality=ScanMatchingStatus();quality.header=m.header;quality.inlier_fraction=inlier;quality.matching_error=error
        self.node.on_quality(quality);self.node.on_pose(m)
        return m

    # Map +X motion at a 90-degree yaw is body -Y; copied raw zero twist cannot satisfy this test.
    def test_body_velocity_from_estimates(self):
        self.sample(10.,angle=math.pi/2);m=self.sample(10.2,x=.04,angle=math.pi/2)
        output=self.node.pub.publish.call_args.args[0]
        self.assertEqual(output.header.stamp,m.header.stamp)
        self.assertAlmostEqual(output.twist.twist.linear.x,0.,places=6)
        self.assertAlmostEqual(output.twist.twist.linear.y,-.1,places=5)

    # Bad registration cannot refresh odometry or TF even when its messages are recent.
    def test_quality_rejected(self):
        self.sample(10.);self.sample(10.2,x=.01,inlier=.2);self.sample(10.4,x=.02,error=.5)
        self.node.pub.publish.assert_not_called();self.node.tf.sendTransform.assert_not_called()

    # A repeated measurement and an implausible position jump must not become controller feedback.
    def test_replay_and_jump(self):
        self.sample(10.);self.sample(10.2,x=.01);count=self.node.pub.publish.call_count
        self.sample(10.2,x=.01);self.sample(10.4,x=2.)
        self.assertEqual(self.node.pub.publish.call_count,count)

    # Planar assumptions are explicit and gross vertical mislocalization is rejected.
    def test_wrong_vertical_solution(self):
        self.sample(10.);self.sample(10.2,z=1.)
        self.node.pub.publish.assert_not_called()

    # With truth disabled, the command adapter creates no localization publisher or TF broadcaster.
    def test_truth_disabled(self):
        adapter=SimAdapter()
        try:
            self.assertFalse(adapter.publish_truth)
            self.assertIsNone(adapter.odom_pub);self.assertIsNone(adapter.tf);self.assertIsNone(adapter.static_tf)
        finally:adapter.destroy_node()

if __name__=='__main__':
    rclpy.init(args=['--ros-args','-p','publish_truth:=false'])
    try:unittest.main()
    finally:rclpy.shutdown()
