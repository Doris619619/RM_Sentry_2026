#!/usr/bin/env python
"""ROS1 trajectory-generation baseline client for the ROS2 migration test."""

import argparse
import json
import math
import os
import sys
import time

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from trajectory_generation.msg import trajectoryPoly

from select_baseline_points import select_points


class BaselineClient(object):
    def __init__(self, odom_topic='/odom_local'):
        self.trajectory = None
        # global_searcher.launch remaps the raw ROS1 /odometry_imu input to
        # /odom_local.  Keep it configurable because hardware deployments can
        # retain the raw name.
        self.odom = rospy.Publisher(odom_topic, Odometry, queue_size=1)
        self.goal = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
        # The ROS1 node owns a private NodeHandle, so its relative output is
        # namespaced under the node name.  ROS2 intentionally normalises this
        # public interface to /global_trajectory.
        rospy.Subscriber('/trajectory_generation/global_trajectory', trajectoryPoly, self.on_trajectory, queue_size=1)

    def on_trajectory(self, message):
        if message.duration and len(message.coef_x) == 4 * len(message.duration) and len(message.coef_y) == 4 * len(message.duration):
            self.trajectory = message

    def publish_inputs(self, points):
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.pose.pose.position.x = points['start'][0]
        odom.pose.pose.position.y = points['start'][1]
        odom.pose.pose.orientation.w = 1.0
        self.odom.publish(odom)
        rospy.sleep(0.2)
        goal = PointStamped()
        goal.header.frame_id = 'map'
        goal.point.x = points['goal'][0]
        goal.point.y = points['goal'][1]
        self.goal.publish(goal)


def metrics(message):
    total_time = sum(message.duration)
    length = 0.0
    previous = None
    for segment, duration in enumerate(message.duration):
        for sample in range(21):
            time_value = duration * sample / 20.0
            x_coef = message.coef_x[segment * 4:(segment + 1) * 4]
            y_coef = message.coef_y[segment * 4:(segment + 1) * 4]
            x = ((x_coef[0] * time_value + x_coef[1]) * time_value + x_coef[2]) * time_value + x_coef[3]
            y = ((y_coef[0] * time_value + y_coef[1]) * time_value + y_coef[2]) * time_value + y_coef[3]
            if previous is not None:
                length += math.hypot(x - previous[0], y - previous[1])
            previous = (x, y)
    return {'end': [previous[0], previous[1]], 'length': length, 'total_time': total_time,
            'segments': len(message.duration)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--map-dir', default=os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'map')))
    parser.add_argument('--odom-topic', default='/odom_local')
    arguments = parser.parse_args()
    points = select_points(arguments.map_dir)
    rospy.init_node('ros1_planning_baseline', anonymous=True)
    client = BaselineClient(arguments.odom_topic)
    deadline = time.time() + 20.0
    while not rospy.is_shutdown() and (client.odom.get_num_connections() == 0 or client.goal.get_num_connections() == 0):
        if time.time() > deadline:
            raise RuntimeError('trajectory_generation subscriptions were not discovered')
        rospy.sleep(0.05)
    client.publish_inputs(points)
    while not rospy.is_shutdown() and client.trajectory is None:
        if time.time() > deadline:
            raise RuntimeError('timed out waiting for ROS1 global_trajectory')
        rospy.sleep(0.05)
    with open(arguments.output, 'w') as output:
        result = metrics(client.trajectory)
        result['selection'] = points
        json.dump(result, output, indent=2, sort_keys=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('FAIL: {}'.format(error), file=sys.stderr)
        sys.exit(1)
