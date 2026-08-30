"""Real service-level BBS and FPFH+RANSAC coverage with deterministic clouds."""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import rclpy
from hdl_global_localization.srv import (
    QueryGlobalLocalization,
    SetGlobalLocalizationEngine,
    SetGlobalMap,
)
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def cloud(points, frame_id='map'):
    header = Header()
    header.frame_id = frame_id
    return point_cloud2.create_cloud_xyz32(header, points)


def bbs_map_points():
    return [(i * 0.31 - 2.5, j * 0.29 - 2.3, 2.2 + 0.02 * ((i + 2 * j) % 3))
            for i in range(18) for j in range(18)]


def bbs_scan_points():
    return [(x, y, 0.0) for x, y, _ in bbs_map_points()]


def ransac_points():
    # Non-symmetric deterministic 3D geometry; query is an exact copy.
    return [(0.17 * i, 0.19 * j, 0.13 * k + 0.01 * ((3 * i + j) % 5))
            for i in range(7) for j in range(6) for k in range(4)]


def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='hdl_global_localization', executable='hdl_global_localization_node',
            name='hdl_global_localization', output='screen',
            parameters=[{
                'global_localization_engine': 'BBS',
                'globalmap_downsample_resolution': 0.01,
                'query_downsample_resolution': 0.01,
                'bbs.map_min_z': 2.0, 'bbs.map_max_z': 2.4,
                'bbs.scan_min_z': -0.2, 'bbs.scan_max_z': 0.2,
                'bbs.map_width': 128, 'bbs.map_height': 128,
                'bbs.map_resolution': 0.25, 'bbs.map_pyramid_level': 4,
                'bbs.min_tx': -10.0, 'bbs.max_tx': 10.0,
                'bbs.min_ty': -10.0, 'bbs.max_ty': 10.0,
                'fpfh.normal_estimation_radius': 0.35,
                'fpfh.search_radius': 0.6,
                'ransac.voxel_based': False,
                'ransac.max_iterations': 2500,
                'ransac.matching_budget': 300,
                'ransac.correspondence_randomness': 3,
                'ransac.max_correspondence_distance': 0.2,
                'ransac.similarity_threshold': 0.8,
                'ransac.inlier_fraction': 0.2,
                'ransac.random_seed': 20260830,
            }]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestGlobalLocalizationServices(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('hdl_global_algorithm_test_client')
        cls.set_map = cls.node.create_client(SetGlobalMap, '/hdl_global_localization/set_global_map')
        cls.query = cls.node.create_client(QueryGlobalLocalization, '/hdl_global_localization/query')
        cls.set_engine = cls.node.create_client(SetGlobalLocalizationEngine, '/hdl_global_localization/set_engine')
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and not all(client.wait_for_service(timeout_sec=0.2)
                                                   for client in (cls.set_map, cls.query, cls.set_engine)):
            pass
        if not all(client.service_is_ready() for client in (cls.set_map, cls.query, cls.set_engine)):
            raise RuntimeError('global localization services did not become available')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, client, request, timeout=30.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(future.done(), f'{client.srv_name} timed out')
        return future.result()

    def set_map_cloud(self, points):
        request = SetGlobalMap.Request()
        request.global_map = cloud(points)
        self.call(self.set_map, request)

    def set_engine_name(self, name):
        request = SetGlobalLocalizationEngine.Request()
        request.engine_name = name
        return self.call(self.set_engine, request)

    def query_cloud(self, points):
        request = QueryGlobalLocalization.Request()
        request.max_num_candidates = 1
        request.cloud = cloud(points, 'base_link')
        return self.call(self.query, request)

    def assert_candidate(self, response):
        self.assertTrue(response.poses, 'algorithm returned no candidate pose')
        self.assertEqual(len(response.poses), len(response.errors))
        self.assertEqual(len(response.poses), len(response.inlier_fractions))
        self.assertTrue(all(math.isfinite(value) for value in response.errors))
        self.assertTrue(all(math.isfinite(value) for value in response.inlier_fractions))

    def test_1_query_without_map_fails_cleanly(self):
        response = self.query_cloud(bbs_scan_points())
        self.assertFalse(response.poses)
        self.assertFalse(response.errors)
        self.assertFalse(response.inlier_fractions)

    def test_2_bbs_set_map_and_query_returns_candidate(self):
        self.assertTrue(self.set_engine_name('BBS').success)
        self.set_map_cloud(bbs_map_points())
        self.assert_candidate(self.query_cloud(bbs_scan_points()))

    def test_3_fpfh_ransac_set_map_and_query_returns_candidate(self):
        self.assertTrue(self.set_engine_name('FPFH_RANSAC').success)
        points = ransac_points()
        self.set_map_cloud(points)
        self.assert_candidate(self.query_cloud(points))

    def test_4_teaser_is_explicitly_unavailable(self):
        response = self.set_engine_name('FPFH_TEASER')
        self.assertFalse(response.success)
        self.assertIn('unavailable', response.message)
