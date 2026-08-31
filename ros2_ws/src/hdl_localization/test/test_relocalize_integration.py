"""Exercise global-map -> scan -> /relocalize -> async candidate -> PoseEstimator reset."""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from hdl_global_localization.srv import QueryGlobalLocalization, SetGlobalMap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from std_srvs.srv import Empty


EXPECTED_TRANSLATION = (1.35, -0.85, 0.45)
EXPECTED_YAW = 0.41
POSITION_TOLERANCE = 0.12
YAW_TOLERANCE = 0.08


def points():
    """Asymmetric deterministic scan-frame geometry for FPFH/RANSAC and NDT."""
    result = []
    for i in range(9):
        for j in range(7):
            for k in range(4):
                if (5 * i + 3 * j + 7 * k) % 6 == 0:
                    continue
                result.append((
                    0.16 * i + 0.013 * j,
                    0.21 * j + 0.007 * k + 0.004 * i * j,
                    0.14 * k + 0.011 * ((2 * i + j + 3 * k) % 5),
                ))
    return result


def transformed_map_points():
    cosine, sine = math.cos(EXPECTED_YAW), math.sin(EXPECTED_YAW)
    return [
        (cosine * x - sine * y + EXPECTED_TRANSLATION[0],
         sine * x + cosine * y + EXPECTED_TRANSLATION[1],
         z + EXPECTED_TRANSLATION[2])
        for x, y, z in points()
    ]


def cloud(frame='map', map_frame=False):
    header = Header()
    header.frame_id = frame
    return point_cloud2.create_cloud_xyz32(header, transformed_map_points() if map_frame else points())


def generate_test_description():
    engine_parameters = {
        'global_localization_engine': 'FPFH_RANSAC',
        'globalmap_downsample_resolution': 0.01,
        'query_downsample_resolution': 0.01,
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
    }
    hdl_parameters = {
        'use_global_localization': True,
        'points_topic': '/relocalize_fixture/scan',
        'globalmap_topic': '/relocalize_fixture/map',
        'odom_topic': '/relocalize_fixture/odom',
        'send_tf_transforms': False,
        'enable_robot_odometry_prediction': False,
        'specify_init_pose': True,
        'downsample_resolution': 0.01,
        'ndt_resolution': 0.5,
    }
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='hdl_global_localization', executable='hdl_global_localization_node',
                                name='hdl_global_localization', output='screen',
                                additional_env={'OMP_NUM_THREADS': '1'}, parameters=[engine_parameters]),
        launch_ros.actions.Node(package='hdl_localization', executable='hdl_localization_node',
                                name='hdl_localization', output='screen', parameters=[hdl_parameters]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestRelocalizeIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('relocalize_integration_client')
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.map_pub = cls.node.create_publisher(PointCloud2, '/relocalize_fixture/map', map_qos)
        cls.scan_pub = cls.node.create_publisher(PointCloud2, '/relocalize_fixture/scan', 10)
        cls.applied = []
        cls.odometry = []
        cls.node.create_subscription(Bool, '/hdl_localization/relocalize_applied', cls.applied.append, 10)
        cls.node.create_subscription(Odometry, '/localization/odometry', cls.odometry.append, 10)
        cls.set_map = cls.node.create_client(SetGlobalMap, '/hdl_global_localization/set_global_map')
        cls.query = cls.node.create_client(QueryGlobalLocalization, '/hdl_global_localization/query')
        cls.relocalize = cls.node.create_client(Empty, '/relocalize')
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and not all(c.wait_for_service(timeout_sec=0.2)
                                                       for c in (cls.set_map, cls.query, cls.relocalize)):
            pass
        if not all(c.service_is_ready() for c in (cls.set_map, cls.query, cls.relocalize)):
            raise RuntimeError('relocalize services did not become ready')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=25.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def call(self, client, request, timeout=20.0):
        future = client.call_async(request)
        self.assertTrue(self.spin_until(future.done, timeout), f'{client.srv_name} timed out')
        return future.result()

    def publish_inputs(self, count=6):
        for _ in range(count):
            self.map_pub.publish(cloud('map', map_frame=True))
            self.scan_pub.publish(cloud('base_link'))
            self.spin_until(lambda: False, timeout=0.15)

    def assert_pose_close(self, pose, label):
        translation_error = math.dist(
            (pose.position.x, pose.position.y, pose.position.z), EXPECTED_TRANSLATION)
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2))
        yaw_error = abs(math.atan2(math.sin(yaw - EXPECTED_YAW), math.cos(yaw - EXPECTED_YAW)))
        self.assertLessEqual(
            translation_error, POSITION_TOLERANCE,
            f'{label} translation error {translation_error:.4f} m exceeds '
            f'{POSITION_TOLERANCE:.4f} m; expected={EXPECTED_TRANSLATION}, '
            f'actual=({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f})')
        self.assertLessEqual(
            yaw_error, YAW_TOLERANCE,
            f'{label} yaw error {yaw_error:.4f} rad exceeds {YAW_TOLERANCE:.4f} rad; '
            f'expected={EXPECTED_YAW:.4f}, actual={yaw:.4f}')

    def test_relocalize_completes_asynchronously_and_resets_pose_estimator(self):
        request = SetGlobalMap.Request()
        request.global_map = cloud('map', map_frame=True)
        self.call(self.set_map, request)
        self.publish_inputs()
        self.assertTrue(self.spin_until(lambda: bool(self.odometry)), 'HDL did not process fixture scan')

        query = QueryGlobalLocalization.Request()
        query.max_num_candidates = 1
        query.cloud = cloud('base_link')
        response = self.call(self.query, query)
        self.assertTrue(response.poses, 'global query did not produce a candidate before /relocalize')
        self.assertEqual(len(response.poses), len(response.errors))
        self.assertTrue(all(math.isfinite(v) for v in response.errors))
        self.assert_pose_close(response.poses[0], 'FPFH+RANSAC pre-relocalize candidate')

        odometry_before = len(self.odometry)
        start = time.monotonic()
        self.call(self.relocalize, Empty.Request(), timeout=5.0)
        self.assertLess(time.monotonic() - start, 5.0, '/relocalize service blocked instead of queuing')
        self.assertTrue(self.spin_until(lambda: any(msg.data for msg in self.applied), timeout=25.0),
                        'async QueryGlobalLocalization did not apply a candidate pose')
        self.publish_inputs(3)
        self.assertTrue(self.spin_until(lambda: len(self.odometry) > odometry_before, timeout=10.0),
                        'PoseEstimator did not resume odometry after relocalize reset')
        self.assert_pose_close(self.odometry[-1].pose.pose, 'PoseEstimator reset odometry')
