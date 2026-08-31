
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import rclpy


def generate_test_description():
    share = get_package_share_directory('sentry_bringup')
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'fixture.launch.py'))),
        launch_testing.actions.ReadyToTest(),
    ])


class TestFixtureLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('sentry_fixture_launch_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_fixture_node_and_topics_start(self):
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            names = {name for name, _ in self.node.get_node_names_and_namespaces()}
            topics = {name for name, _ in self.node.get_topic_names_and_types()}
            if 'sentry_fixture_inputs' in names and '/filted_topic_3d' in topics:
                return
        self.fail('fixture.launch.py did not start sentry_fixture_inputs and its cloud topic')
