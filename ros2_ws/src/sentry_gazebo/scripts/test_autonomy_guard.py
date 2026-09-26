#!/usr/bin/env python3
"""Isolated synthetic safety contract tests; these are not Gazebo motion acceptance."""
import math
import numpy as np
from autonomy_geometry import StaticSafetyGrid
import unittest
from unittest.mock import patch
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, UInt8
from trajectory_generation.msg import TrajectoryPoly
from autonomy_guard import AutonomyGuard, trajectory_limits

class GuardTests(unittest.TestCase):
    # Use a private domain in the runner; wall stamps avoid an external simulated clock dependency.
    def setUp(self):
        self.node=AutonomyGuard()
        self.node.static_grid=StaticSafetyGrid(np.zeros((400,400),dtype=bool))
        self.refresh()
        self.goal=PoseStamped()
        self.goal.header.frame_id='map'
        self.goal.pose.position.x=1.
        self.goal.pose.orientation.w=1.

    # Always tear down the DDS entity after a case.
    def tearDown(self):
        self.node.destroy_node()

    # Provide explicit synthetic inputs solely for contract-level fault injection.
    def refresh(self):
        odom=Odometry();odom.header.frame_id='map';odom.child_frame_id='base_link'
        odom.header.stamp=self.node.get_clock().now().to_msg();odom.pose.pose.orientation.w=1.
        self.node.on_odom(odom)
        cloud=PointCloud2();cloud.header.frame_id='map';cloud.header.stamp=odom.header.stamp
        cloud.width=1;cloud.height=1
        self.node.on_cloud(cloud)

    # Build a simple bounded polynomial for guard validation, never for end-to-end motion claims.
    def path(self):
        msg=TrajectoryPoly();msg.start_time=self.node.get_clock().now().to_msg()
        msg.duration=[10.];msg.coef_x=[0.,0.,.1,0.];msg.coef_y=[0.,0.,0.,0.]
        return msg

    # A valid goal and reference arm tracking but do not themselves supply a velocity command.
    def test_valid_reference(self):
        self.node.on_goal(self.goal);self.node.on_trajectory(self.path())
        self.assertEqual(self.node.state,'tracking')

    # A target outside map-frame semantics must cancel the previous trajectory.
    def test_wrong_frame_cancels(self):
        self.node.on_goal(self.goal);self.node.on_trajectory(self.path())
        self.goal.header.frame_id='base_link';self.node.on_goal(self.goal)
        self.assertEqual(self.node.state,'stopped')

    # Localization replay cannot satisfy the measurement-age requirement.
    def test_stale_odom(self):
        self.node.odom.header.stamp.sec-=3
        self.node.on_goal(self.goal)
        self.assertIn('localization',self.node.reason)
        self.assertEqual(self.node.state,'stopped')

    # Missing aligned clouds also covers measurement-time TF failure upstream.
    def test_missing_cloud(self):
        self.node.cloud_received=-math.inf
        self.node.on_goal(self.goal)
        self.assertIn('cloud',self.node.reason)

    # Changing mode clears active output and rejects all old controller messages.
    def test_manual_takeover(self):
        self.node.on_goal(self.goal);self.node.on_trajectory(self.path())
        self.node.on_mode(String(data='manual'))
        cmd=Twist();cmd.linear.x=.4
        self.node.on_auto(cmd)
        self.assertEqual(self.node.command.linear.x,0.)
        self.assertIsNone(self.node.goal)

    # A delayed polynomial for an old request cannot arm a new goal.
    def test_old_reference(self):
        self.node.on_goal(self.goal)
        old=self.path();old.start_time.sec-=5;self.node.on_trajectory(old)
        self.assertEqual(self.node.state,'planning')

    # Collision rejection uses a synthetic occupied cell on a known line, not a replica of planner internals.
    def test_occupied_reference(self):
        col=int((.5+13.394)/.05);row=399-int(12.079/.05)
        self.node.static_grid.mask[row,col]=True
        self.node.on_goal(self.goal);self.node.on_trajectory(self.path())
        self.assertEqual(self.node.state,'stopped')
        self.assertIn('occupied',self.node.reason)

    # Independently finite pieces must also agree in position and velocity at the join.
    def test_discontinuous_reference(self):
        self.node.on_goal(self.goal)
        msg=self.path();msg.duration=[5.,5.];msg.coef_x=[0.,0.,.1,0.,0.,0.,.1,2.];msg.coef_y=[0.]*8
        self.node.on_trajectory(msg)
        self.assertIn('discontinuous',self.node.reason)

    # A stalled sensor stream during planning must cancel, not resume a queued goal automatically.
    def test_fault_while_planning(self):
        self.node.on_goal(self.goal);self.node.cloud_received=-math.inf
        self.node.tick()
        self.assertEqual(self.node.state,'stopped')
        self.assertIn('cloud',self.node.reason)

    # Analytic velocity checks reject fast paths rather than silently clipping only actuator output.
    def test_fast_reference(self):
        self.node.on_goal(self.goal)
        msg=self.path();msg.duration=[1.];msg.coef_x=[0.,0.,1.,0.]
        self.node.on_trajectory(msg)
        self.assertEqual(self.node.state,'stopped')
        self.assertIn('limits',self.node.reason)

    # Reference expiration stops even if fresh localization and sensor input remain available.
    def test_expired_reference(self):
        self.node.on_goal(self.goal);self.node.on_trajectory(self.path())
        self.node.valid_until=0
        self.node.tick()
        self.assertEqual(self.node.reason,'trajectory expired')

    # A suspended supervisor must require a fresh goal after its own scheduling gap.
    def test_supervisor_stall(self):
        self.node.on_goal(self.goal);self.node.on_trajectory(self.path())
        self.node.last_tick_wall-=2
        self.node.tick()
        self.assertEqual(self.node.state,'stopped')
        self.assertIn('supervisor',self.node.reason)

    # Unsupported Fortress rewind must require a complete restart, even after explicit auto selection.
    def test_clock_rewind_requires_restart(self):
        self.node.on_goal(self.goal);self.node.on_trajectory(self.path())
        self.node.sim_previous=self.node.get_clock().now().nanoseconds*1e-9+1
        self.node.tick();self.node.on_mode(String(data='auto'));self.node.on_goal(self.goal)
        self.assertTrue(self.node.epoch_fault)
        self.assertEqual(self.node.state,'stopped')
        self.assertIn('restart',self.node.reason)

    # A stopped simulation latches the autonomous trajectory off rather than resuming on clock progress.
    def test_pause_latches(self):
        self.node.on_goal(self.goal);self.node.on_trajectory(self.path())
        self.node.tick()
        stamp=rclpy.time.Time(nanoseconds=self.node.get_clock().now().nanoseconds)
        self.node.sim_previous=stamp.nanoseconds*1e-9
        with patch.object(self.node.get_clock(),'now') as now:
            now.return_value=stamp
            self.node.clock_progress-=1
            self.node.tick()
        self.assertEqual(self.node.state,'stopped')

    # An otherwise valid reference intersecting measured occupancy stays stopped while requesting a new plan.
    def test_dynamic_reference_cannot_reach_controller(self):
        self.node.on_goal(self.goal)
        self.node.dynamic_grid.mask[:]=True
        self.node.on_trajectory(self.path())
        self.assertEqual(self.node.state,'planning')
        self.assertIn('dynamic obstacle',self.node.reason)
        self.assertIsNone(self.node.endpoint)


    # Full-system referee state must be fresh and active before any automatic reference is accepted.
    def test_referee_gate_and_game_stop(self):
        self.node.require_referee=True
        self.node.on_goal(self.goal)
        self.assertIn('referee serial timeout',self.node.reason)
        self.node.on_referee(UInt8(data=4));self.node.on_goal(self.goal);self.node.on_trajectory(self.path())
        self.assertEqual(self.node.state,'tracking')
        self.node.on_referee(UInt8(data=0));self.node.tick()
        self.assertEqual(self.node.state,'stopped')
        self.assertIn('not active',self.node.reason)

    # A corrupted serial receive stream cannot keep authorizing motion using previously decoded state.
    def test_referee_timeout(self):
        self.node.require_referee=True;self.node.on_referee(UInt8(data=4))
        self.node.on_goal(self.goal);self.node.on_trajectory(self.path())
        self.node.referee_wall-=1.;self.node.tick()
        self.assertEqual(self.node.state,'stopped')
        self.assertIn('referee serial timeout',self.node.reason)


if __name__=='__main__':
    rclpy.init()
    try:unittest.main()
    finally:rclpy.shutdown()
