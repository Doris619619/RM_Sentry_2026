
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
from rcl_interfaces.srv import GetParameters


def make_map():
    handle = tempfile.NamedTemporaryFile(mode='w', suffix='.pcd', delete=False)
    points = [(0.25 * i, 0.25 * j, 0.1 * ((i + j) % 4)) for i in range(16) for j in range(16)]
    handle.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
    handle.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
    for x, y, z in points:
        handle.write(f"{x} {y} {z}\n")
    handle.close()
    return handle.name


def production_launch(global_enabled):
    bringup = get_package_share_directory('sentry_bringup')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'production.launch.py')),
        launch_arguments={
            'globalmap_pcd': make_map(),
            'enable_sensor_pipeline': 'false',
            'enable_global_localization': 'true' if global_enabled else 'false',
            'enable_mcu': 'false',
            'allow_motion_output': 'false',
        }.items())


class GraphTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('sentry_bringup_graph_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for(self, predicate, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def node_names(self):
        return {name for name, _ in self.node.get_node_names_and_namespaces()}

    def hdl_global_localization_enabled(self):
        client = self.node.create_client(GetParameters, '/hdl_localization/get_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), 'HDL parameter service unavailable')
        request = GetParameters.Request()
        request.names = ['use_global_localization']
        future = client.call_async(request)
        self.assertTrue(self.wait_for(future.done, timeout=10.0), 'HDL parameter query timed out')
        values = future.result().values
        self.assertEqual(len(values), 1)
        return values[0].bool_value

def generate_test_description():
    return launch.LaunchDescription([production_launch(True), launch_testing.actions.ReadyToTest()])


class TestProductionGlobalLocalization(GraphTest):
    def test_global_enabled_starts_server_and_connects_hdl_without_sensor_nodes(self):
        self.assertTrue(self.wait_for(lambda: {'hdl_localization', 'hdl_global_localization'} <= self.node_names()))
        self.assertTrue(self.wait_for(lambda: '/relocalize' in [name for name, _ in self.node.get_service_names_and_types()]))
        services = [name for name, _ in self.node.get_service_names_and_types()]
        self.assertIn('/hdl_global_localization/query', services)
        names = self.node_names()
        self.assertNotIn('livox_lidar_publisher', names)
        self.assertNotIn('threeD_lidar_filter_pointcloud', names)
        self.assertNotIn('laserMapping', names)
        self.assertTrue(self.hdl_global_localization_enabled())
