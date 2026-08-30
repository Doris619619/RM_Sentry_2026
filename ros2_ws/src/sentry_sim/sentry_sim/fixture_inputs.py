#!/usr/bin/env python3
"""Publish deterministic odometry, cloud and referee/radar interface fixtures.

This is deliberately interface-level software simulation. It does not model a
robot, chassis, sensors, or a competition world.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import UInt8, UInt16


class FixtureInputs(Node):
    def __init__(self):
        super().__init__('sentry_fixture_inputs')
        qos = 10
        self.odom = self.create_publisher(Odometry, '/localization/odometry', qos)
        self.cloud = self.create_publisher(PointCloud2, '/filted_topic_3d', qos)
        self.imu = self.create_publisher(Imu, '/livox/imu_192_168_1_3', qos)
        self.livox = self.create_publisher(CustomMsg, '/livox/lidar', qos)
        self.game = self.create_publisher(UInt8, '/referee/game_progress', qos)
        self.hp = self.create_publisher(UInt16, '/referee/remain_hp', qos)
        self.bullets = self.create_publisher(UInt16, '/referee/bullet_remain', qos)
        self.target = self.create_publisher(UInt8, '/radar/suggested_target', qos)
        self.hero = self.create_publisher(Point, '/enemy/hero_position', qos)
        self.timer = self.create_timer(0.05, self.publish)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = -3.82
        odom.pose.pose.position.y = 2.40
        odom.pose.pose.orientation.w = 1.0
        self.odom.publish(odom)

        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = 'base_link'
        self.cloud.publish(cloud)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        imu.orientation.w = 1.0
        self.imu.publish(imu)

        livox = CustomMsg()
        livox.header.stamp = stamp
        livox.header.frame_id = 'base_link'
        livox.timebase = stamp.sec * 1000000000 + stamp.nanosec
        livox.point_num = 0
        self.livox.publish(livox)

        self.game.publish(UInt8(data=4))
        self.hp.publish(UInt16(data=400))
        self.bullets.publish(UInt16(data=100))
        self.target.publish(UInt8(data=0))
        self.hero.publish(Point(x=-1.35, y=-4.20, z=0.0))


def main():
    rclpy.init()
    node = FixtureInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
