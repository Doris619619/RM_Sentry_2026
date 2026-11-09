"""Exercise the standalone navigation launch with remapped synthetic inputs, not physics acceptance."""
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, Bool
from trajectory_generation.msg import TrajectoryPoly


# Start only the three navigation processes, deliberately replacing all primary topic names.
def generate_test_description():
    entry = Path(get_package_share_directory('sentry_bringup')) / 'launch/navigation.launch.py'
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(entry)), launch_arguments={
            'odom_topic': '/external/odom', 'cloud_topic': '/external/cloud',
            'goal_topic': '/external/goal', 'trajectory_topic': '/external/trajectory',
            'cmd_vel_topic': '/external/cmd_vel'}.items()),
        launch_testing.actions.ReadyToTest()])


class ExternalInputsTest(unittest.TestCase):
    # All publishers are explicitly synthetic; this test never claims physical motion or LIO precision.
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('external_navigation_provider')
        cls.odom = cls.node.create_publisher(Odometry, '/external/odom', qos_profile_sensor_data)
        cls.cloud = cls.node.create_publisher(PointCloud2, '/external/cloud', qos_profile_sensor_data)
        cls.goal = cls.node.create_publisher(PoseStamped, '/external/goal', 10)
        cls.replan = cls.node.create_publisher(Bool, '/replan_flag', 10)
        cls.commands, cls.trajectories, cls.solvers = [], [], []
        cls.node.create_subscription(Twist, '/external/cmd_vel', cls.commands.append, 10)
        cls.node.create_subscription(TrajectoryPoly, '/external/trajectory', cls.trajectories.append, 10)
        cls.node.create_subscription(Bool, '/solver_status', cls.solvers.append, 10)

    # Remove this test's DDS participants even when a contract assertion fails.
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    # Feed stationary map-frame observations while the actual planner and SQP controller compute.
    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            odom = Odometry()
            odom.header.frame_id = 'map'
            odom.header.stamp = self.node.get_clock().now().to_msg()
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = -3.82, 2.4
            odom.pose.pose.orientation.w = 1.
            self.odom.publish(odom)
            self.cloud.publish(create_cloud_xyz32(Header(stamp=odom.header.stamp, frame_id='map'), [(0., 0., 2.)]))
            rclpy.spin_once(self.node, timeout_sec=.05)

    # Verify both a usable remapped pipeline and cancellation of a wrongly framed replacement goal.
    def test_remapped_pipeline_and_frame_rejection(self):
        self.pump(3.)
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.pose.position.x, target.pose.position.y = -1.35, -4.2
        target.pose.orientation.w = 1.
        self.goal.publish(target)
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            self.pump(.2)
            if any(abs(c.linear.x) + abs(c.linear.y) > 1e-5 for c in self.commands):
                break
        self.assertTrue(any(t.duration for t in self.trajectories))
        self.assertTrue(any(s.data for s in self.solvers))
        self.assertTrue(any(abs(c.linear.x) + abs(c.linear.y) > 1e-5 for c in self.commands))
        self.assertFalse(any(n in self.node.get_node_names() for n in ['strategy_node', 'point_lio', 'mcu_communicator']))
        target.header.frame_id = 'base_link'
        self.goal.publish(target)
        self.pump(1.)
        self.replan.publish(Bool(data=True))
        self.pump(.5)
        self.assertFalse(self.trajectories[-1].duration)
        self.assertAlmostEqual(self.commands[-1].linear.x, 0.)
        self.assertAlmostEqual(self.commands[-1].linear.y, 0.)
