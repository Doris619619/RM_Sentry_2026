#!/usr/bin/env python3
"""Publish real lidar returns in declared sensor/body/map frames at measurement time."""
import copy
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class CloudAdapter(Node):
    """Own only lidar extrinsics and cloud conversion, never robot localization."""
    def __init__(self):
        """Create publishers and a bounded queue so slightly late TF can arrive."""
        super().__init__('sentry_cloud_adapter')
        self.declare_parameter('native_frame','sentry/base_link/lidar_sensor')
        self.native_frame=self.get_parameter('native_frame').value
        self.raw=self.create_publisher(PointCloud2,'/sim/lidar/points',qos_profile_sensor_data)
        self.body=self.create_publisher(PointCloud2,'/filted_topic_3d',qos_profile_sensor_data)
        self.aligned=self.create_publisher(PointCloud2,'/aligned_points',qos_profile_sensor_data)
        self.tf=Buffer(cache_time=Duration(seconds=10))
        self.listener=TransformListener(self.tf,self)
        self.static=StaticTransformBroadcaster(self)
        extrinsic=TransformStamped();extrinsic.header.frame_id='base_link';extrinsic.child_frame_id='lidar_link'
        extrinsic.transform.translation.z=0.7;extrinsic.transform.rotation.w=1.0
        self.static.sendTransform(extrinsic)
        self.pending=deque(maxlen=8);self.last_warning=0.
        self.create_subscription(PointCloud2,'/sim/lidar/native',self.receive,qos_profile_sensor_data)
        self.create_timer(.02,self.flush,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self,message):
        """Canonicalize the known SDF sensor frame; retain actual samples and stamp."""
        if message.header.frame_id != self.native_frame:
            self.warn('Rejected unexpected sensor frame: '+message.header.frame_id)
            return
        raw=copy.deepcopy(message);raw.header.frame_id='lidar_link';self.raw.publish(raw)
        points=point_cloud2.read_points(raw,field_names=('x','y','z'),skip_nans=False)
        # Gazebo also carries integer ring metadata; structured extraction keeps mixed fields valid.
        xyz=np.column_stack([points[name].reshape(-1) for name in ('x','y','z')])
        valid=np.isfinite(xyz).all(axis=1)
        xyz=xyz[valid];ranges=np.linalg.norm(xyz,axis=1)
        xyz=xyz[(ranges>=.2)&(ranges<9.999)]
        clean=point_cloud2.create_cloud_xyz32(raw.header,xyz)
        self.pending.append((time.monotonic(),clean))
        self.flush()

    def flush(self):
        """Transform at the original stamp; wait at most 0.5 wall seconds for TF."""
        keep=deque(maxlen=8)
        for received,cloud in self.pending:
            try:
                stamp=Time.from_msg(cloud.header.stamp)
                body_tf=self.tf.lookup_transform('base_link','lidar_link',stamp)
                map_tf=self.tf.lookup_transform('map','lidar_link',stamp)
                self.body.publish(do_transform_cloud(cloud,body_tf))
                self.aligned.publish(do_transform_cloud(cloud,map_tf))
            except TransformException:
                if time.monotonic()-received<.5:keep.append((received,cloud))
                else:self.warn('Dropped lidar frame: no TF at measurement time')
        self.pending=keep

    def warn(self,message):
        """Throttle diagnostic output by wall time, including while simulation pauses."""
        if time.monotonic()-self.last_warning>2:
            self.get_logger().warning(message);self.last_warning=time.monotonic()

def main():
    """Run until launch shutdown; no command or localization publisher is created."""
    rclpy.init();node=CloudAdapter()
    try:rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
