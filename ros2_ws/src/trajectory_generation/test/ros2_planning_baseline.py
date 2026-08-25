#!/usr/bin/env python3
"""Produce a ROS2 baseline using the largest connected inflated free region."""

import argparse
import json
import os
import sys

import rclpy
from geometry_msgs.msg import PointStamped

from mock_replan_fsm import ReplanFsmMock, trajectory_metrics
from select_baseline_points import select_points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--map-dir', default=os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'map')))
    arguments = parser.parse_args()
    selection = select_points(arguments.map_dir)

    rclpy.init()
    node = ReplanFsmMock()
    try:
        node.wait_for_subscribers(5.0)
        node.publish_odom(*selection['start'])
        node.spin_for(0.4)
        goal = PointStamped()
        goal.header.frame_id = 'map'
        goal.point.x, goal.point.y = selection['goal']
        node.goal_pub.publish(goal)
        node.wait_for(1, 20.0)
        result = trajectory_metrics(node.trajectories[0])
        result['selection'] = selection
        with open(arguments.output, 'w', encoding='utf-8') as output:
            json.dump(result, output, indent=2, sort_keys=True)
            output.write('\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('FAIL: {}'.format(error), file=sys.stderr)
        sys.exit(1)
