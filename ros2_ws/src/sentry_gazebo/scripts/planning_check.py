#!/usr/bin/env python3
"""Exercise five deterministic real-scene start/goal pairs and inspect published polynomial paths."""
import json,math,time,subprocess,yaml
from collections import deque
from pathlib import Path
import numpy as np
from PIL import Image
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from trajectory_generation.msg import TrajectoryPoly
import sys
ROOT=Path(__file__).resolve().parents[4]
LIMITS=yaml.safe_load((ROOT/'ros2_ws/src/trajectory_generation/config/global_planning.yaml').read_text())['trajectory_generation']['ros__parameters']
sys.path.insert(0,str(ROOT/'ros2_ws/src/trajectory_generation/test'))
import select_baseline_points as baseline

def map_cases():
    """Use the repository's inflation and maximum free component, then select five route pairs."""
    folder=ROOT/'ros2_ws/src/trajectory_generation/map'
    occ=baseline._load_gray(str(folder/'occfinal.png'));topo=baseline._load_gray(str(folder/'occtopo.png'))
    component,radius=baseline._largest_component(occ,topo)
    start,_=baseline._farthest(component,min(component));goal,_=baseline._farthest(component,start)
    parents={start:None};queue=deque([start])
    while queue and goal not in parents:
        r,c=queue.popleft()
        for dr in [-1,0,1]:
            for dc in [-1,0,1]:
                q=(r+dr,c+dc)
                if q in component and q not in parents:parents[q]=(r,c);queue.append(q)
    path=[];q=goal
    while q is not None:path.append(q);q=parents[q]
    path.reverse()
    pairs=[(path[0],path[-1]),(path[-1],path[0]),(path[10],path[-11]),(path[20],path[-21]),(path[40],path[-41])]
    mask=np.array(occ)>10;inflated=mask.copy()
    for dr in range(-radius,radius+1):
        for dc in range(-radius,radius+1):
            if dr*dr+dc*dc>radius*radius:continue
            r0=max(0,dr);r1=min(400,400+dr);c0=max(0,dc);c1=min(400,400+dc)
            inflated[r0:r1,c0:c1]|=mask[r0-dr:r1-dr,c0-dc:c1-dc]
    return [(baseline._coord(a),baseline._coord(b)) for a,b in pairs],inflated

def metrics(message,mask,goal):
    """Bound arc-length samples, solve speed extrema and check acceleration, joins, endpoint and occupancy."""
    durations=np.array(message.duration);x=np.array(message.coef_x).reshape(-1,4);y=np.array(message.coef_y).reshape(-1,4)
    assert len(x)==len(y)==len(durations)>0 and np.isfinite(durations).all() and (durations>0).all()
    assert np.isfinite(x).all() and np.isfinite(y).all()
    samples=[]
    for cx,cy,d in zip(x,y,durations):
        boundx=3*abs(cx[0])*d*d+2*abs(cx[1])*d+abs(cx[2])
        boundy=3*abs(cy[0])*d*d+2*abs(cy[1])*d+abs(cy[2])
        count=max(2,int(math.ceil(d*math.hypot(boundx,boundy)/.02))+1)
        ts=np.linspace(0,d,count)
        samples.extend(np.column_stack([np.polyval(cx,ts),np.polyval(cy,ts)]))
    pts=np.array(samples);spacing=np.linalg.norm(np.diff(pts,axis=0),axis=1)
    col=np.floor((pts[:,0]-baseline.LOWER_X)/baseline.RESOLUTION).astype(int)
    row=399-np.floor((pts[:,1]-baseline.LOWER_Y)/baseline.RESOLUTION).astype(int)
    inside=(col>=0)&(col<400)&(row>=0)&(row<400)
    collisions=int(np.count_nonzero(~inside)+np.count_nonzero(mask[row[inside],col[inside]]))
    end_error=float(np.linalg.norm(pts[-1]-goal))
    acceleration=join_gap=join_velocity_gap=speed=0.
    for index,(cx,cy,duration) in enumerate(zip(x,y,durations)):
        for t in [0.,duration]:
            acceleration=max(acceleration,math.hypot(np.polyval(np.polyder(cx,2),t),np.polyval(np.polyder(cy,2),t)))
        vx,vy=np.polyder(cx),np.polyder(cy)
        speed_slope=np.polyadd(np.polymul(vx,np.polyder(vx)),np.polymul(vy,np.polyder(vy)))
        extrema=[0.,duration]+[float(t.real) for t in np.roots(speed_slope) if abs(t.imag)<1e-8 and 0<t.real<duration]
        speed=max(speed,max(math.hypot(np.polyval(vx,t),np.polyval(vy,t)) for t in extrema))
        if index+1<len(durations):
            join_gap=max(join_gap,math.hypot(np.polyval(cx,duration)-x[index+1,3],np.polyval(cy,duration)-y[index+1,3]))
            join_velocity_gap=max(join_velocity_gap,math.hypot(np.polyval(np.polyder(cx),duration)-x[index+1,2],np.polyval(np.polyder(cy),duration)-y[index+1,2]))
    result={'segments':len(durations),'sample_count':len(pts),'max_sample_spacing':float(spacing.max()),
            'occupied_samples':collisions,'endpoint_error':end_error,'start':pts[0].tolist(),'end':pts[-1].tolist(),
            'duration':durations.tolist(),'coef_x':message.coef_x.tolist(),'coef_y':message.coef_y.tolist()}
    result.update(max_acceleration=acceleration,max_join_gap=join_gap,max_join_velocity_gap=join_velocity_gap,max_speed=speed,acceleration_limit=LIMITS['planner.reference_a_max'],speed_limit=LIMITS['planner.reference_v_max'])
    result['passed']=speed<=LIMITS['planner.reference_v_max']+1e-4 and acceleration<=LIMITS['planner.reference_a_max']+1e-4 and join_gap<=1e-4 and join_velocity_gap<=1e-4 and end_error<=.15 and collisions==0 and result['max_sample_spacing']<=.025
    return result

class LivePlanning:
    """Use actual Gazebo pose services and actual ROS planner output, never mock odometry."""
    def __init__(self):
        """Discover the existing planner and retain time-stamped trajectories."""
        self.node=rclpy.create_node('planning_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.odom=None;self.messages=[]
        self.node.create_subscription(Odometry,'/localization/odometry',self.on_odom,10)
        self.node.create_subscription(TrajectoryPoly,'/global_trajectory',lambda m:self.messages.append(m),10)
        self.pub=self.node.create_publisher(PoseStamped,'/goal',10)
    def on_odom(self,message):
        """Keep the latest actual localization feedback for set-pose convergence."""
        self.odom=message
    def spin(self,seconds):
        """Drain DDS callbacks for a bounded wall interval."""
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.02)
    def teleport(self,point):
        """Move the real simulated entity between QA cases and wait for actual feedback."""
        req=f'name: "sentry", position: {{x: {point[0]}, y: {point[1]}, z: 0}}, orientation: {{w: 1}}'
        result=subprocess.run(['ign','service','-s','/world/sentry_planning/set_pose','--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','3000','--req',req],capture_output=True,text=True,timeout=5)
        assert result.returncode==0 and 'true' in result.stdout,result.stdout+result.stderr
        self.spin(1.4)
        p=self.odom.pose.pose.position
        assert math.hypot(p.x-point[0],p.y-point[1])<1e-5
    def request(self,start,goal):
        """Request a fresh target and reject stale or differently anchored trajectory messages."""
        self.spin(.2);self.messages.clear()
        stamp=self.node.get_clock().now()
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=stamp.to_msg()
        msg.pose.position.x=goal[0];msg.pose.position.y=goal[1];msg.pose.orientation.w=1.;self.pub.publish(msg)
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            self.spin(.05)
            for traj in self.messages:
                when=traj.start_time.sec*10**9+traj.start_time.nanosec
                if when<stamp.nanoseconds or not traj.duration:continue
                endx=np.polyval(traj.coef_x[-4:],traj.duration[-1]);endy=np.polyval(traj.coef_y[-4:],traj.duration[-1])
                if math.hypot(endx-goal[0],endy-goal[1])<.15 and math.hypot(traj.coef_x[3]-start[0],traj.coef_y[3]-start[1])<.15:
                    return traj
        raise RuntimeError(f'No fresh valid-endpoint trajectory for {start} -> {goal}')

def main():
    """Run 15 independent pose/goal requests; preserve partial failures for diagnosis."""
    pairs,mask=map_cases();rclpy.init();live=LivePlanning();records=[]
    out=ROOT/'docs/simulation/part2/evidence/planning-cases.json'
    try:
        live.spin(3)
        for index,(start,goal) in enumerate(pairs):
            for repetition in range(3):
                live.teleport(start);traj=live.request(start,goal);result=metrics(traj,mask,goal)
                result.update(pair=index+1,repetition=repetition+1,requested_start=start,requested_goal=goal)
                records.append(result);out.write_text(json.dumps(records,indent=2))
                print(json.dumps({k:v for k,v in result.items() if k not in ['coef_x','coef_y','duration']}),flush=True)
        failed=sum(not item['passed'] for item in records)
        assert failed==0, f'{failed}/{len(records)} planning cases failed; see {out}'
    finally:
        live.teleport(pairs[0][0]);live.request(*pairs[0])
        out.write_text(json.dumps(records,indent=2));live.node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
