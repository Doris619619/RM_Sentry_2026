"""Launch production with the physical sensor gate enabled and assert one process per node."""

import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import rclpy


def make_map():
    handle = tempfile.NamedTemporaryFile(mode="w", suffix=".pcd", delete=False)
    points = [(0.25 * i, 0.25 * j, 0.1 * ((i + j) % 4)) for i in range(16) for j in range(16)]
    handle.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
    handle.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
    for x, y, z in points:
        handle.write(f"{x} {y} {z}\n")
    handle.close()
    return handle.name


def generate_test_description():
    share = get_package_share_directory("sentry_bringup")
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "production.launch.py")),
            launch_arguments={
                "globalmap_pcd": make_map(),
                "enable_sensor_pipeline": "true",
                "enable_global_localization": "false",
                "enable_mcu": "false",
                "allow_motion_output": "false",
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestProductionSensorPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("sentry_sensor_pipeline_graph_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_sensor_pipeline_starts_once_and_keeps_mcu_closed(self):
        deadline = time.monotonic() + 20.0
        required = {"livox_lidar_publisher", "threeD_lidar_filter_pointcloud", "laserMapping", "hdl_localization"}
        names = []
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            names = [name for name, _ in self.node.get_node_names_and_namespaces()]
            if required <= set(names):
                break
        self.assertTrue(required <= set(names), f"sensor-gated nodes not all running: {names}")
        for name in required:
            self.assertEqual(names.count(name), 1, f"{name} was launched more than once")
        self.assertNotIn("mcu_communicator", names)
