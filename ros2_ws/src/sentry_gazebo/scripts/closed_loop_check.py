#!/usr/bin/env python3
"""Measure actual Gazebo closed-loop motion against published polynomial paths and occupancy."""
import argparse
import subprocess
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from trajectory_generation.msg import TrajectoryPoly
from planning_check import LivePlanning, map_cases, ROOT

class ClosedLoop(LivePlanning):
    # Add measured truth and explicit safety status to the existing actual-scene client.
    def __init__(self):
        super().__init__()
        self.status = {}
        self.samples = []
        self.paths = []
        self.current_path = None
        self.yaw = 0.
        self.mode = self.node.create_publisher(String, '/sim/control_mode', 10)
        self.node.create_subscription(String, '/sim/autonomy/status', self.on_status, 10)
        self.node.create_subscription(Odometry, '/sim/ground_truth/odometry', self.on_truth, 10)
        self.node.create_subscription(TrajectoryPoly, '/global_trajectory', self.on_path, 10)

    # Record the supervisor's state separately from controller arrival claims.
    def on_status(self, message):
        self.status = json.loads(message.data)

    # Retain actual polynomial coefficients and a dense projection for cross-track error.
    def on_path(self, message):
        if not message.duration:
            self.current_path = None
            return
        self.paths.append({'duration':list(message.duration),'x':list(message.coef_x),'y':list(message.coef_y)})
        points=[]
        for i,d in enumerate(message.duration):
            ts=np.linspace(0,d,max(2,math.ceil(d/.025)))
            points.extend(zip(np.polyval(message.coef_x[4*i:4*i+4],ts),np.polyval(message.coef_y[4*i:4*i+4],ts)))
        self.current_path=np.asarray(points)

    # Sample only real Gazebo feedback; preserve the path active at each observation.
    def on_truth(self, message):
        if not self.samples or time.monotonic()-self.samples[-1]['wall'] >= .08:
            p=message.pose.pose.position;v=message.twist.twist
            error=None if self.current_path is None else float(np.linalg.norm(self.current_path-[p.x,p.y],axis=1).min())
            self.samples.append({'wall':time.monotonic(),'sim':message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                'x':p.x,'y':p.y,'speed':math.hypot(v.linear.x,v.linear.y),'wz':v.angular.z,'cross_track':error})

    # Reset between cases while stopped; no teleport is permitted during a timed route.
    def prepare(self, start):
        self.mode.publish(String(data='stop'));self.spin(.5)
        if abs(self.yaw)<1e-9:
            self.teleport(start)
        else:
            request=f'name: "sentry", position: {{x: {start[0]}, y: {start[1]}, z: 0}}, orientation: {{z: {math.sin(self.yaw/2)}, w: {math.cos(self.yaw/2)}}}'
            response=subprocess.run(['ign','service','-s','/world/sentry_planning/set_pose','--reqtype','ignition.msgs.Pose',
                '--reptype','ignition.msgs.Boolean','--timeout','3000','--req',request],capture_output=True,text=True,timeout=5)
            if response.returncode or 'true' not in response.stdout:raise RuntimeError('set_pose failed')
            self.spin(1.4)
            p=self.odom.pose.pose.position
            if math.hypot(p.x-start[0],p.y-start[1])>.01:raise RuntimeError('actual spawn differs')
        self.mode.publish(String(data='auto'));self.spin(.5)
        self.samples=[];self.paths=[];self.current_path=None

# Run selected deterministic routes and persist even partial or failed attempts.
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--single',action='store_true')
    parser.add_argument('--yaw',type=float,default=0.)
    parser.add_argument('--timeout',type=float,default=180)
    parser.add_argument('--output',default=str(ROOT/'docs/simulation/autonomy/evidence/closed-loop.json'))
    args=parser.parse_args()
    pairs,mask=map_cases()
    if args.single:pairs=pairs[:1]
    rclpy.init();live=ClosedLoop();live.yaw=args.yaw;records=[]
    try:
        ready_until=time.monotonic()+30
        while time.monotonic()<ready_until and (live.odom is None or not live.status):
            live.spin(.1)
        if live.odom is None or not live.status: raise RuntimeError('simulation did not become ready in 30 seconds')
        live.spin(1)
        for index,(start,goal) in enumerate(pairs):
            for repetition in range(1 if args.single else 3):
                live.prepare(start)
                start_wall=time.monotonic()
                try:
                    live.request(start,goal)
                    while time.monotonic()-start_wall<args.timeout:
                        live.spin(.1)
                        if live.status.get('state') in ('arrived','stopped'):break
                    sample=list(live.samples)
                    end=sample[-1]
                    distance=math.hypot(end['x']-goal[0],end['y']-goal[1])
                    occupied=0
                    for pt in sample:
                        col=int(math.floor((pt['x']+13.394)/.05));row=399-int(math.floor((pt['y']+12.079)/.05))
                        occupied+=int(not(0<=col<400 and 0<=row<400) or (0<=col<400 and 0<=row<400 and mask[row,col]))
                    errors=[pt['cross_track'] for pt in sample if pt['cross_track'] is not None]
                    result={'case':index+1,'repeat':repetition+1,'start':start,'goal':goal,
                        'wall_duration':time.monotonic()-start_wall,'status':live.status,'endpoint_error':distance,
                        'max_speed':max(pt['speed'] for pt in sample),'occupied_samples':occupied,
                        'max_cross_track':max(errors) if errors else None,
                        'p95_cross_track':float(np.percentile(errors,95)) if errors else None,
                        'samples':sample,'trajectories':live.paths,
                        'passed':live.status.get('state')=='arrived' and distance<=.15 and occupied==0 and max(pt['speed'] for pt in sample)<=.501}
                except Exception as error:
                    result={'case':index+1,'repeat':repetition+1,'passed':False,'error':str(error),'status':live.status,
                            'samples':live.samples,'trajectories':live.paths}
                records.append(result)
                Path(args.output).write_text(json.dumps(records,indent=2))
                print(json.dumps({k:v for k,v in result.items() if k not in ('samples','trajectories')}),flush=True)
                if not result['passed']:raise RuntimeError('closed-loop acceptance failed')
    finally:
        live.mode.publish(String(data='stop'));live.spin(.3)
        Path(args.output).write_text(json.dumps(records,indent=2))
        live.node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
