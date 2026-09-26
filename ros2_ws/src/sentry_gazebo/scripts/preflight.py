#!/usr/bin/env python3
"""Fail closed on conflicting publishers in the selected simulation ROS domain."""
import time
import rclpy
rclpy.init()
node = rclpy.create_node('sentry_sim_preflight')
end = time.monotonic() + 2.0
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
conflicts = {topic: node.count_publishers(topic) for topic in
             ('/clock', '/localization/odometry', '/sim/ground_truth/odometry', '/tf', '/tf_static')}
conflicts = {k: v for k, v in conflicts.items() if v}
node.destroy_node()
rclpy.shutdown()
if conflicts:
    raise SystemExit('检测到冲突发布者，请先停止旧仿真/定位节点：' + str(conflicts))
