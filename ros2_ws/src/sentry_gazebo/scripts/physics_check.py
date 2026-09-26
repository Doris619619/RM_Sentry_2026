#!/usr/bin/env python3
"""Measure genuine rigid-body gravity, effort, acceleration, braking, contact and ramp motion."""
import argparse,json,math,subprocess,time,signal
import psutil
from pathlib import Path
import numpy as np,rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class PhysicsCheck:
    # Only actual odometry is recorded; commands never become synthetic feedback.
    def __init__(self):
        self.node=rclpy.create_node('physics_acceptance');self.samples=[];self.force=0.
        self.pub=self.node.create_publisher(Twist,'/cmd_vel',1)
        self.node.create_subscription(Odometry,'/sim/ground_truth/odometry',self.odom,50)
        self.node.create_subscription(Float64,'/sim/drive_force',lambda m:setattr(self,'force',m.data),20)

    def odom(self,m):
        p,q,v=m.pose.pose.position,m.pose.pose.orientation,m.twist.twist
        pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.samples.append({'sim':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
            'x':p.x,'y':p.y,'z':p.z,'pitch':pitch,'yaw':yaw,'vx':v.linear.x,'vy':v.linear.y,
            'vz':v.linear.z,'wz':v.angular.z,'speed':math.hypot(v.linear.x,v.linear.y),'force_N':self.force})

    # Publish real bounded commands while processing the independent simulator feedback.
    def run(self,seconds,vx=0.,vy=0.,wz=0.,publish=True):
        cmd=Twist();cmd.linear.x=vx;cmd.linear.y=vy;cmd.angular.z=wz
        end=time.monotonic()+seconds;last=0.
        while time.monotonic()<end:
            if publish and time.monotonic()-last>.03:self.pub.publish(cmd);last=time.monotonic()
            rclpy.spin_once(self.node,timeout_sec=.01)

    # Teleports are limited to test setup, followed by a zero-command settling period.
    def reset(self,x=0.,y=-3.,z=.02,yaw=0.,settle=1.5):
        self.run(1.)
        request=f'name: "sentry" position: {{x: {x} y: {y} z: {z}}} orientation: {{z: {math.sin(yaw/2)} w: {math.cos(yaw/2)}}}'
        result=subprocess.run(['ign','service','-s','/world/sentry_physics/set_pose','--reqtype','ignition.msgs.Pose',
            '--reptype','ignition.msgs.Boolean','--timeout','3000','--req',request],capture_output=True,text=True,timeout=5)
        assert 'true' in result.stdout,result.stdout
        self.samples=[];self.run(settle)
        if settle>1:
            assert abs(self.samples[-1]['z'])<.02,self.samples[-1]
            self.samples=[]

    def gravity(self):
        self.reset(z=1.,settle=.03);self.run(2.)
        samples=self.samples
        peak=max(samples,key=lambda s:s['z'])
        falling=[s for s in samples if 0<=s['sim']-peak['sim']<=.2]
        fit=np.polyfit([s['sim']-peak['sim'] for s in falling],[s['z'] for s in falling],2)
        result={'fitted_gravity':-2*float(fit[0]),'height_start':peak['z'],'height_min':min(s['z'] for s in samples),'height_end':samples[-1]['z']}
        result['passed']=9.<result['fitted_gravity']<10.5 and result['height_start']>.8 and result['height_min']>-.02 and abs(result['height_end'])<.02
        return result

    def acceleration(self):
        self.reset();self.run(3.,vx=.4);moving=list(self.samples);begin=self.samples[-1]
        self.run(2.);end=self.samples[-1]
        acceleration=[(b['vx']-a['vx'])/(b['sim']-a['sim']) for a,b in zip(moving,moving[1:]) if b['sim']>a['sim']]
        steady=[s for s in moving if s['sim']>moving[-1]['sim']-.5]
        result={'max_speed':max(s['speed'] for s in moving),'max_acceleration':max(acceleration),
            'brake_distance':math.hypot(end['x']-begin['x'],end['y']-begin['y']),
            'steady_force_N':float(np.mean([s['force_N'] for s in steady])),'stop_speed':end['speed']}
        result['passed']=.35<result['max_speed']<=.51 and .1<result['max_acceleration']<1.5 and result['brake_distance']<.3 and end['speed']<.02 and result['steady_force_N']>.1
        return result

    def lateral_rotation(self):
        self.reset(yaw=math.pi/2);self.run(.1);before=self.samples[-1];self.run(2.,vx=.3);self.run(1.)
        after=self.samples[-1];self.run(2.,wz=.3);self.run(1.);turn=self.samples[-1]
        result={'world_x_displacement':after['x']-before['x'],'world_y_displacement':after['y']-before['y'],
            'rotation':math.atan2(math.sin(turn['yaw']-after['yaw']),math.cos(turn['yaw']-after['yaw']))}
        result['passed']=abs(result['world_x_displacement'])<.08 and result['world_y_displacement']>.4 and result['rotation']>.3
        return result

    def timeout(self):
        self.reset();self.run(2.,vx=.4);before=self.samples[-1];started=time.monotonic();stop=None
        while time.monotonic()-started<2.:
            self.run(.03,publish=False)
            if time.monotonic()-started>.3 and self.samples[-1]['speed']<.02 and stop is None:stop=time.monotonic()-started
        after=self.samples[-1]
        result={'stop_wall_seconds':stop,'travel_after_loss':math.hypot(after['x']-before['x'],after['y']-before['y']),'stop_speed':after['speed']}
        result['passed']=stop is not None and stop<1.5 and result['travel_after_loss']<.5 and after['speed']<.02
        return result

    def collision(self):
        self.reset(x=1.,y=0.);self.run(6.,vx=.4);impact=list(self.samples);self.run(1.)
        result={'wall_front_x':2.4,'body_half_x':.35,'max_center_x':max(s['x'] for s in impact),
            'final_center_x':impact[-1]['x'],'final_speed_under_command':impact[-1]['speed'],
            'max_force_N':max(s['force_N'] for s in impact)}
        result['passed']=2.0<result['max_center_x']<=2.075 and impact[-1]['speed']<.03 and result['max_force_N']<=30.001
        return result

    def ramp(self):
        self.reset(x=.1,y=3.);self.run(13.,vx=.3);top=self.samples[-1];self.run(3.);parked=self.samples[-1]
        result={'top_x':top['x'],'top_z':top['z'],'top_pitch_degrees':math.degrees(top['pitch']),
            'parked_speed':parked['speed'],'parked_z':parked['z'],'max_force_N':max(s['force_N'] for s in self.samples)}
        result['passed']=top['x']>2.5 and top['z']>.10 and 3.<abs(result['top_pitch_degrees'])<7. and parked['speed']<.02 and result['max_force_N']<=30.001
        return result

    # Suspend the actual ROS command adapter; only the Gazebo force plugin can then expire its input.
    def native_watchdog(self):
        self.reset();self.run(2.,vx=.4)
        root=psutil.Process(self.launch_pid)
        assert 'physics.launch.py' in root.cmdline()
        nodes=[p for p in root.children(recursive=True) if any(a.endswith('/sim_adapter.py') for a in p.cmdline())]
        assert len(nodes)==1
        adapter=nodes[0];before=self.samples[-1];started=time.monotonic();stop=None
        adapter.send_signal(signal.SIGSTOP)
        try:
            while time.monotonic()-started<2.:
                self.run(.03,publish=False)
                if self.samples[-1]['speed']<.02 and stop is None:stop=time.monotonic()-started
            after=self.samples[-1]
        finally:adapter.send_signal(signal.SIGCONT)
        result={'stop_wall_seconds':stop,'travel_after_adapter_hang':math.hypot(after['x']-before['x'],after['y']-before['y'])}
        result['passed']=stop is not None and stop<1. and result['travel_after_adapter_hang']<.35
        return result

    # A paused physical body keeps momentum, then actively brakes without replaying an old target.
    def pause_resume(self):
        self.reset();self.run(2.,vx=.3);before=self.samples[-1]
        def control(pause):
            value='true' if pause else 'false'
            r=subprocess.run(['ign','service','-s','/world/sentry_physics/control','--reqtype','ignition.msgs.WorldControl',
                '--reptype','ignition.msgs.Boolean','--timeout','3000','--req','pause: '+value],capture_output=True,text=True,timeout=5)
            assert 'true' in r.stdout
        control(True)
        try:
            self.run(.3,publish=False);paused_pose=self.samples[-1];frozen=paused_pose['sim'];self.run(.8,publish=False)
            assert self.samples[-1]['sim']==frozen
        finally:control(False)
        self.run(1.,publish=False);after=self.samples[-1]
        travel=math.hypot(after['x']-paused_pose['x'],after['y']-paused_pose['y'])
        request_motion=math.hypot(paused_pose['x']-before['x'],paused_pose['y']-before['y'])
        return {'clock_frozen':True,'motion_while_pause_service_pending':request_motion,'resume_braking_distance':travel,'stop_speed':after['speed'],
            'passed':travel<.08 and after['speed']<.02}


def main():
    """Persist every completed case, including failures, and always stop the real model."""
    p=argparse.ArgumentParser();p.add_argument('--only',default='');p.add_argument('--pid',type=int,required=True);p.add_argument('--output',required=True);a=p.parse_args()
    rclpy.init();qa=PhysicsCheck();qa.launch_pid=a.pid;records=[]
    try:
        ready=time.monotonic()+30
        while not qa.samples and time.monotonic()<ready:qa.run(.1)
        assert qa.samples,'No native physics odometry'
        qa.run(1.)
        for name in ['gravity','acceleration','lateral_rotation','timeout','native_watchdog','pause_resume','collision','ramp']:
            if a.only and a.only!=name:continue
            detail=getattr(qa,name)();record={'name':name,**detail,'samples':list(qa.samples)}
            records.append(record);Path(a.output).write_text(json.dumps(records,indent=2))
            print(json.dumps({k:v for k,v in record.items() if k!='samples'}),flush=True)
            if not record['passed']:raise RuntimeError(name+' failed')
    finally:
        qa.run(1.);qa.node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
