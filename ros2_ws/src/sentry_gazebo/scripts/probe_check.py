#!/usr/bin/env python3
"""Validate actual lidar ranges and transformations against a separate known-wall world."""
import json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def xyz(message):
    """Extract XYZ without assuming Gazebo's integer ring field shares the float type."""
    points=point_cloud2.read_points(message,field_names=('x','y','z'),skip_nans=False)
    return np.column_stack([points[n].reshape(-1) for n in ('x','y','z')])

class Probe:
    """Collect all cloud frames and drive only small known-safe motions."""
    def __init__(self):
        """Subscribe to actual returns and actual model odometry for independent checks."""
        self.node=rclpy.create_node('sentry_probe_check');self.clouds={};self.odom=None
        for topic in ['/sim/lidar/points','/filted_topic_3d','/aligned_points']:
            self.node.create_subscription(PointCloud2,topic,lambda m,t=topic:self.clouds.__setitem__(t,m),qos_profile_sensor_data)
        self.node.create_subscription(Odometry,'/localization/odometry',self.receive_odom,10)
        self.pub=self.node.create_publisher(Twist,'/cmd_vel',10)
    def receive_odom(self,message):
        """Retain model state; no command integration is used as feedback."""
        self.odom=message
    def run(self,seconds,vx=0.,wz=0.):
        """Publish at 20 Hz for a wall-time interval and drain feedback callbacks."""
        end=time.monotonic()+seconds;next_pub=0
        while time.monotonic()<end:
            if time.monotonic()>=next_pub:
                m=Twist();m.linear.x=vx;m.angular.z=wz;self.pub.publish(m);next_pub=time.monotonic()+.05
            rclpy.spin_once(self.node,timeout_sec=.01)
    def wall(self):
        """Measure known x=3 wall using map-frame returns, avoiding other distant objects."""
        pts=xyz(self.clouds['/aligned_points'])
        wall=pts[(abs(pts[:,1])<1.5)&(pts[:,0]>2.5)&(pts[:,0]<3.5)]
        assert len(wall)>10,len(wall)
        return float(np.max(abs(wall[:,0]-3))),len(wall)

def main():
    """Check stationary range then actual translation/rotation and transformed wall alignment."""
    rclpy.init();p=Probe();records=[]
    try:
        p.run(5)
        assert len(p.clouds)==3 and p.odom is not None
        raw=p.clouds['/sim/lidar/points'];before=xyz(raw)
        assert raw.width*raw.height==1440,(raw.width,raw.height)
        e,n=p.wall();assert e<=.1
        records.append({'name':'stationary_wall','passed':True,'max_error_m':e,'returns':n,'raw_frame':raw.header.frame_id,'dimensions':[raw.width,raw.height]})
        p.run(2,vx=.2);p.run(.6)
        x=p.odom.pose.pose.position.x;e,n=p.wall()
        assert x>.15 and e<=.1,(x,e)
        after=xyz(p.clouds['/sim/lidar/points'])
        difference=bool(not np.allclose(before,after,equal_nan=True))
        assert difference
        records.append({'name':'translation','passed':True,'actual_x':x,'wall_error_m':e,'returns_changed':difference})
        p.run(2,wz=.3);p.run(.6)
        q=p.odom.pose.pose.orientation;yaw=2*math.atan2(q.z,q.w);e,n=p.wall()
        assert yaw>.2 and e<=.1,(yaw,e)
        records.append({'name':'rotation','passed':True,'actual_yaw':yaw,'wall_error_m':e})
        for topic,frame in [('/filted_topic_3d','base_link'),('/aligned_points','map')]:
            m=p.clouds[topic];assert m.header.frame_id==frame and np.isfinite(xyz(m)).all()
        records.append({'name':'finite_and_frames','passed':True})
    finally:
        p.run(.4);p.node.destroy_node();rclpy.shutdown()
        out=Path(__file__).resolve().parents[4]/'docs/simulation/part2/evidence/probe.json'
        out.write_text(json.dumps(records,indent=2));print(json.dumps(records,indent=2))
if __name__=='__main__':main()
