#!/usr/bin/env python3
"""Verify one explicit map-frame goal using actual simulation feedback; never reset or move the model."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from trajectory_generation.msg import TrajectoryPoly
from autonomy_geometry import StaticSafetyGrid


# Send one target after localization/cloud readiness and retain real observations until arrival/failure.
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', nargs=2, type=float, required=True)
    parser.add_argument('--timeout', type=float, default=120.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('navigation_step2_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {'odom': None, 'cloud': None, 'status': {}, 'samples': [], 'trajectories': 0, 'unsafe_references': 0}
    grid = StaticSafetyGrid.load()

    # Record measured motion rather than deriving pose from commands or target values.
    def odometry(message):
        state['odom'] = message
        p, v = message.pose.pose.position, message.twist.twist.linear
        state['samples'].append({'time': message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                                 'x': p.x, 'y': p.y, 'speed': math.hypot(v.x, v.y)})

    # Check only admitted references; rejected planner candidates do not drive the robot.
    def trajectory(message):
        if message.duration:
            state['trajectories'] += 1
            state['unsafe_references'] += int(not grid.trajectory_free(message))

    node.create_subscription(Odometry, '/localization/odometry', odometry, 10)
    node.create_subscription(PointCloud2, '/aligned_points', lambda m: state.update(cloud=m), qos_profile_sensor_data)
    node.create_subscription(String, '/sim/autonomy/status', lambda m: state.update(status=json.loads(m.data)), 10)
    node.create_subscription(TrajectoryPoly, '/global_trajectory', trajectory, 10)
    goal_pub = node.create_publisher(PoseStamped, '/goal', 10)
    mode_pub = node.create_publisher(String, '/sim/control_mode', 10)
    result = {'goal': args.goal, 'passed': False}
    started = time.monotonic()
    try:
        while time.monotonic() - started < 20:
            rclpy.spin_once(node, timeout_sec=.05)
            if state['odom'] is not None and state['cloud'] is not None and goal_pub.get_subscription_count():
                break
        if state['odom'] is None or state['cloud'] is None:
            raise RuntimeError('No localization and map-frame cloud within 20 seconds')
        if not grid.points_free([args.goal]):
            raise RuntimeError('Requested goal is not in static free space')
        mode_pub.publish(String(data='auto'))
        ready_until = time.monotonic() + .5
        while time.monotonic() < ready_until:
            rclpy.spin_once(node, timeout_sec=.02)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = args.goal
        goal.pose.orientation.w = 1.
        state['samples'].clear()
        state['status'] = {}
        goal_pub.publish(goal)
        issued = time.monotonic()
        while time.monotonic() - issued < args.timeout:
            rclpy.spin_once(node, timeout_sec=.02)
            status = state['status'].get('state')
            if status == 'arrived':
                break
            if status in ('failed', 'fault', 'stopped'):
                raise RuntimeError(str(state['status']))
        samples = state['samples']
        end = samples[-1]
        error = math.hypot(end['x'] - args.goal[0], end['y'] - args.goal[1])
        occupied = sum(not grid.points_free([[p['x'], p['y']]]) for p in samples)
        result.update(duration=time.monotonic()-issued, endpoint_error=error,
                      max_speed=max(p['speed'] for p in samples), occupied_samples=occupied,
                      odom_frame=state['odom'].header.frame_id, child_frame=state['odom'].child_frame_id,
                      cloud_frame=state['cloud'].header.frame_id, status=state['status'],
                      trajectories=state['trajectories'], unsafe_references=state['unsafe_references'], samples=samples)
        result['passed'] = (state['status'].get('state') == 'arrived' and error <= .15 and end['speed'] < .03
                            and occupied == 0 and state['trajectories'] > 0 and state['unsafe_references'] == 0)
        if not result['passed']:
            raise RuntimeError('Navigation acceptance failed; see the recorded measurements')
    except Exception as error:
        result.update(error=str(error), status=state['status'], samples=state['samples'])
        raise
    finally:
        mode_pub.publish(String(data='stop'))
        rclpy.spin_once(node, timeout_sec=.1)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2))
        print(json.dumps({k: v for k, v in result.items() if k != 'samples'}, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
