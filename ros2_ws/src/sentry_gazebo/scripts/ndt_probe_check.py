#!/usr/bin/env python3
"""Compare real HDL/NDT outputs with measurement-time Gazebo truth; truth is never fed to the estimator."""
import argparse,json,math,time
from collections import deque
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from hdl_localization.msg import ScanMatchingStatus
from closed_loop_check import ClosedLoop,autonomy_cases
import numpy as np

# Extract planar angle only for planar accuracy metrics; keep z/tilt as separate reported errors.
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

def main():
    """Move slowly using manual simulation commands after stationary estimator acquisition."""
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
    rclpy.init();live=ClosedLoop();truth=deque(maxlen=400);estimates=[];statuses=[];records=[]
    live.node.create_subscription(Odometry,'/sim/ground_truth/odometry',truth.append,50)
    live.node.create_subscription(Odometry,'/sim/ndt/odometry',estimates.append,20)
    live.node.create_subscription(ScanMatchingStatus,'/sim/ndt/status',lambda m:statuses.append(
        {'sim':stamp(m),'error':m.matching_error,'inlier':m.inlier_fraction}),20)
    pub=live.node.create_publisher(Twist,'/sim/manual_cmd_vel',1)
    live.mode.publish(String(data='manual'))
    began=time.monotonic();last=0;result={}
    try:
        while time.monotonic()-began<24:
            elapsed=time.monotonic()-began;cmd=Twist()
            if 8<elapsed<13:cmd.linear.x=-.15;cmd.linear.y=.10
            if 15<elapsed<19:cmd.angular.z=.15
            pub.publish(cmd);live.spin(.04)
            while last<len(estimates):
                m=estimates[last];last+=1
                if not truth:continue
                t=min(truth,key=lambda t:abs(stamp(t)-stamp(m)))
                if abs(stamp(t)-stamp(m))>.025:continue
                p1,p2=m.pose.pose.position,t.pose.pose.position;q1,q2=m.pose.pose.orientation,t.pose.pose.orientation
                angle=math.atan2(math.sin(yaw(q1)-yaw(q2)),math.cos(yaw(q1)-yaw(q2)))
                records.append({'wall':elapsed,'sim':stamp(m),'xy_error':math.hypot(p1.x-p2.x,p1.y-p2.y),
                    'yaw_error':abs(angle),'z_error':p1.z-p2.z,'estimate':[p1.x,p1.y,p1.z,yaw(q1)],
                    'truth':[p2.x,p2.y,p2.z,yaw(q2)],'stamp_delta':stamp(m)-stamp(t)})
        settled=[r for r in records if r['wall']>5]
        result={'samples':records,'statuses':statuses,'initial_prior_offset_m':math.hypot(.15,.1),
            'max_xy_error_after_5s':max((r['xy_error'] for r in settled),default=None),
            'rms_xy_error_after_5s':float(np.sqrt(np.mean([r['xy_error']**2 for r in settled]))) if settled else None,
            'max_yaw_error_after_5s':max((r['yaw_error'] for r in settled),default=None),
            'max_abs_z_error':max((abs(r['z_error']) for r in settled),default=None),
            'estimated_hz':(len(settled)-1)/(settled[-1]['sim']-settled[0]['sim']) if len(settled)>1 else 0}
        result['passed']=len(settled)>50 and result['max_xy_error_after_5s']<.15 and result['max_yaw_error_after_5s']<.1
    finally:
        pub.publish(Twist());live.mode.publish(String(data='stop'));live.spin(.3)
        Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ('samples','statuses')}))
        live.node.destroy_node();rclpy.shutdown()
    if not result.get('passed'):raise SystemExit(1)
if __name__=='__main__':main()
