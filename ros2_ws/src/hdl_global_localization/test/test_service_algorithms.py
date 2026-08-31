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


def transform_points(points, translation, yaw, z_offset=0.0):
    """Return map-frame points from a scan-frame cloud using a known SE(3) pose."""
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        (cosine * x - sine * y + translation[0],
         sine * x + cosine * y + translation[1],
         z + translation[2] + z_offset)
        for x, y, z in points
    ]


def bbs_scan_points():
    # A deliberately asymmetric 2D scene.  The irregular omissions prevent a
    # 180-degree/lattice ambiguity while retaining enough points for BBS.
    return [
        (0.31 * i - 2.7, 0.29 * j - 2.3, 0.0)
        for i in range(19) for j in range(17)
        if (7 * i + 11 * j) % 5 != 0
    ]


BBS_GRID_RESOLUTION = 0.25
BBS_MAX_RANGE = 15.0
BBS_YAW = 8.0 * math.acos(1.0 - BBS_GRID_RESOLUTION ** 2 / (2.0 * BBS_MAX_RANGE ** 2))
BBS_TRANSLATION = (1.5, -1.0, 0.0)


def bbs_map_points():
    # BBS slices the map at z=2.0..2.4 and the scan at z=-0.2..0.2.
    return transform_points(bbs_scan_points(), BBS_TRANSLATION, BBS_YAW, z_offset=2.2)


def ransac_points():
    # Non-symmetric deterministic 3D geometry with distinct local FPFH
    # neighborhoods.  It is intentionally not a regular cuboid/lattice.
    points = []
    for i in range(9):
        for j in range(7):
            for k in range(4):
                if (5 * i + 3 * j + 7 * k) % 6 == 0:
                    continue
                points.append((
                    0.16 * i + 0.013 * j,
                    0.21 * j + 0.007 * k + 0.004 * i * j,
                    0.14 * k + 0.011 * ((2 * i + j + 3 * k) % 5),
                ))
    return points


RANSAC_TRANSLATION = (1.35, -0.85, 0.45)
RANSAC_YAW = 0.41


def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='hdl_global_localization', executable='hdl_global_localization_node',
            name='hdl_global_localization', output='screen',
            additional_env={'OMP_NUM_THREADS': '1'},
            parameters=[{
                'global_localization_engine': 'BBS',
                'globalmap_downsample_resolution': 0.01,
                'query_downsample_resolution': 0.01,
                'bbs.map_min_z': 2.0, 'bbs.map_max_z': 2.4,
                'bbs.scan_min_z': -0.2, 'bbs.scan_max_z': 0.2,
                'bbs.map_width': 128, 'bbs.map_height': 128,
                'bbs.map_resolution': 0.25, 'bbs.map_pyramid_level': 4,
                'bbs.min_tx': -3.0, 'bbs.max_tx': 3.0,
                'bbs.min_ty': -3.0, 'bbs.max_ty': 3.0,
                'bbs.min_theta': -0.6, 'bbs.max_theta': 0.6,
                'fpfh.normal_estimation_radius': 0.35,
                'fpfh.search_radius': 0.6,
                'ransac.voxel_based': False,
                'ransac.max_iterations': 10000,
                'ransac.matching_budget': 1200,
                'ransac.correspondence_randomness': 1,
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

    def assert_pose_close(self, response, expected_translation, expected_yaw,
                          translation_tolerance, yaw_tolerance, algorithm):
        self.assert_candidate(response)
        pose = response.poses[0]
        translation_error = math.dist(
            (pose.position.x, pose.position.y, pose.position.z), expected_translation)
        measured_yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2))
        yaw_error = abs(math.atan2(math.sin(measured_yaw - expected_yaw),
                                   math.cos(measured_yaw - expected_yaw)))
        self.assertLessEqual(
            translation_error, translation_tolerance,
            f'{algorithm} translation error {translation_error:.4f} m exceeds '
            f'{translation_tolerance:.4f} m; expected={expected_translation}, '
            f'actual=({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f})')
        self.assertLessEqual(
            yaw_error, yaw_tolerance,
            f'{algorithm} yaw error {yaw_error:.4f} rad exceeds {yaw_tolerance:.4f} rad; '
            f'expected={expected_yaw:.4f}, actual={measured_yaw:.4f}')

    def test_1_query_without_map_fails_cleanly(self):
        response = self.query_cloud(bbs_scan_points())
        self.assertFalse(response.poses)
        self.assertFalse(response.errors)
        self.assertFalse(response.inlier_fractions)

    def test_2_bbs_recovers_known_planar_translation_and_yaw(self):
        self.assertTrue(self.set_engine_name('BBS').success)
        self.set_map_cloud(bbs_map_points())
        self.assert_pose_close(
            self.query_cloud(bbs_scan_points()), BBS_TRANSLATION, BBS_YAW,
            translation_tolerance=0.36, yaw_tolerance=0.035, algorithm='BBS')

    def test_3_fpfh_ransac_recovers_known_3d_translation_and_yaw(self):
        self.assertTrue(self.set_engine_name('FPFH_RANSAC').success)
        scan = ransac_points()
        self.set_map_cloud(transform_points(scan, RANSAC_TRANSLATION, RANSAC_YAW))
        self.assert_pose_close(
            self.query_cloud(scan), RANSAC_TRANSLATION, RANSAC_YAW,
            translation_tolerance=0.12, yaw_tolerance=0.08, algorithm='FPFH+RANSAC')

    def test_4_teaser_is_explicitly_unavailable(self):
        response = self.set_engine_name('FPFH_TEASER')
        self.assertFalse(response.success)
        self.assertIn('unavailable', response.message)
