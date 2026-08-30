"""Software vehicle loop: HK PTY -> Decision -> Planning -> Tracking -> HK PTY."""

import os
import pty
import select
import sys
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_testing
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt8
from trajectory_generation.msg import TrajectoryPoly

sys.path.insert(0, os.path.dirname(__file__))
from test_mcu_pty import game_frame, hk_frames, read_available


def generate_test_description():
    master_fd, slave_fd = pty.openpty()
    os.set_blocking(master_fd, True)
    planning = get_package_share_directory("trajectory_generation")
    tracking = get_package_share_directory("trajectory_tracking")
    decision = {
        "tick_hz": 40,
        "central_threshold": 999,
        # These are existing planner regression coordinates, not production goals.
        "goals.occupy.point_0.x": -1.35, "goals.occupy.point_0.y": -4.20,
        "goals.occupy.point_1.x": -1.35, "goals.occupy.point_1.y": -4.20,
        "goals.occupy.point_2.x": -1.35, "goals.occupy.point_2.y": -4.20,
        "goals.occupy.point_3.x": -1.35, "goals.occupy.point_3.y": -4.20,
    }
    return launch.LaunchDescription([
        launch_ros.actions.Node(package="trajectory_generation", executable="trajectory_generator_node", parameters=[
            os.path.join(planning, "config", "global_planning.yaml"),
            os.path.join(planning, "config", "map_metadata.yaml")]),
        launch_ros.actions.Node(package="trajectory_tracking", executable="trajectory_tracking_node", parameters=[
            os.path.join(tracking, "config", "tracking.yaml"),
            os.path.join(planning, "config", "map_metadata.yaml")]),
        launch_ros.actions.Node(package="trajectory_tracking", executable="hit_bridge"),
        launch_ros.actions.Node(package="decision_node", executable="strategy_node", parameters=[decision]),
        launch_ros.actions.Node(package="decision_node", executable="mcu_communicator", parameters=[{
            "serial_port": os.ttyname(slave_fd), "baudrate": 921600, "nav_frequency": 100.0,
            "cmd_vel_timeout": 0.5, "reconnect_interval": 0.05,
        }]),
        launch_testing.actions.ReadyToTest(),
    ]), {"serial_master": master_fd, "serial_slave": slave_fd}


class TestSystemE2E(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("sentry_system_e2e_client")
        cls.goals, cls.trajectories, cls.commands, cls.arrivals, cls.motion = [], [], [], [], []
        cls.node.create_subscription(TrajectoryPoly, "/global_trajectory", lambda m: cls.trajectories.append(m), 10)
        cls.node.create_subscription(Twist, "/cmd_vel", lambda m: cls.commands.append(m), 10)
        cls.node.create_subscription(Bool, "/dstar_status", lambda m: cls.arrivals.append(m.data), 10)
        cls.node.create_subscription(UInt8, "/motion", lambda m: cls.motion.append(m.data), 10)
        cls.games = []
        cls.node.create_subscription(UInt8, "/referee/game_progress", lambda m: cls.games.append(m.data), 10)
        cls.odom = cls.node.create_publisher(Odometry, "/localization/odometry", 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def publish_odom(self, x, y):
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.w = 1.0
        self.odom.publish(odom)

    def wait_for(self, predicate, serial_master, position=(-3.82, 2.40), timeout=25.0):
        deadline = time.monotonic() + timeout
        data = bytearray()
        while time.monotonic() < deadline:
            self.publish_odom(*position)
            rclpy.spin_once(self.node, timeout_sec=0.04)
            readable, _, _ = select.select([serial_master], [], [], 0)
            if readable:
                data.extend(read_available(serial_master, 0))
            if predicate(data):
                return data
        self.fail("timed out waiting for end-to-end condition")

    def test_hk_decision_planning_tracking_and_return(self, serial_master, serial_slave):
        # Repeat a valid real wire frame until all volatile state subscribers have discovered it.
        frame = game_frame()
        start = time.monotonic()
        last_frame = 0.0
        data = bytearray()
        while time.monotonic() - start < 11.0:
            current = time.monotonic()
            # Let the map-heavy Planner/Tracking pair and Decision finish DDS
            # discovery before the first volatile referee state is emitted.
            if current - start >= 3.0 and current - last_frame >= 0.10:
                os.write(serial_master, frame)
                last_frame = current
            self.publish_odom(-3.82, 2.40)
            rclpy.spin_once(self.node, timeout_sec=0.04)
            data.extend(read_available(serial_master, 0))
        self.assertIn(4, self.games, "HK RX frame did not reach the MCU ROS publisher")
        self.assertTrue(self.trajectories, "Decision did not drive the actual planner")
        self.assertTrue(any(message.duration for message in self.trajectories), "planner emitted malformed trajectory")

        def command_reaches_mcu(serial):
            navigation = [packet for packet in hk_frames(serial) if len(packet) == 21]
            return any(abs(command.linear.x) + abs(command.linear.y) > 1e-4 and any(
                int.from_bytes(packet[11:13], "little", signed=True) == int(command.linear.x * 1000) and
                int.from_bytes(packet[13:15], "little", signed=True) == int(command.linear.y * 1000)
                for packet in navigation) for command in self.commands)

        data = self.wait_for(command_reaches_mcu, serial_master, timeout=20.0)
        navigation = [packet for packet in hk_frames(data) if len(packet) == 21]
        self.assertTrue(navigation, "Tracking / hit_bridge cmd_vel never reached MCU")

        # Arrive at the same test goal: Tracking -> hit_bridge -> Decision must return to OCCUPY.
        self.wait_for(lambda _: True in self.arrivals and 1 in self.motion, serial_master,
                      position=(-1.35, -4.20), timeout=20.0)
        os.close(serial_master)
        os.close(serial_slave)
