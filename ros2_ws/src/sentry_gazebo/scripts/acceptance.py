#!/usr/bin/env python3
"""Run against the real GUI simulation; measures Gazebo odometry, never integrates input."""
import copy
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import time
import threading
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

def yaw(m):
    q = m.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def angle(v):
    return math.atan2(math.sin(v), math.cos(v))

def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec*1e-9

class Acceptance(Node):
    def __init__(self):
        super().__init__('sentry_acceptance', parameter_overrides=[])
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.odom = None
        self.raw = {}
        self.samples = []
        self.pairs = 0
        self.max_error = 0.0
        self.create_subscription(Odometry, '/localization/odometry', self.on_odom, 100)
        self.create_subscription(Odometry, '/sim/ground_truth/odometry', self.on_raw, 100)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.results = []

    def on_raw(self, m):
        self.raw[stamp(m)] = m
        if len(self.raw) > 300: self.raw.pop(next(iter(self.raw)))

    def on_odom(self, m):
        self.odom = m
        self.samples.append(m)
        assert m.header.frame_id == 'map' and m.child_frame_id == 'base_link'
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        assert all(math.isfinite(x) for x in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.linear.x,v.linear.y,v.angular.z))
        raw = self.raw.get(stamp(m))
        if raw:
            self.pairs += 1
            self.max_error = max(self.max_error, abs(p.x-raw.pose.pose.position.x),
                                 abs(p.y-raw.pose.pose.position.y),
                                 abs(v.linear.x-raw.twist.twist.linear.x),
                                 abs(v.linear.y-raw.twist.twist.linear.y))

    def spin_wall(self, seconds, command=None):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if command is not None: self.pub.publish(command)
            rclpy.spin_once(self, timeout_sec=0.01)

    def move(self, x=0., y=0., z=0., seconds=1.):
        assert self.odom is not None
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = x,y,z
        start = copy.deepcopy(self.odom)
        sim_start = stamp(start)
        wall_end = time.monotonic()+max(20,seconds*5)
        samples_start = len(self.samples)
        while stamp(self.odom)-sim_start < seconds:
            if time.monotonic()>wall_end: raise RuntimeError('Simulation time stalled')
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.01)
        finish = copy.deepcopy(self.odom)
        segment = self.samples[samples_start:]
        self.spin_wall(0.3, Twist())
        return start, finish, segment

    def check(self, name, passed, **data):
        result=dict(name=name, passed=bool(passed), **data)
        self.results.append(result)
        print(json.dumps(result), flush=True)
        if not passed: raise AssertionError(name)

    def stop_check(self, name, send_zero):
        if send_zero: self.pub.publish(Twist())
        begin=time.monotonic()
        deadline=begin+1.0
        seen=False
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=0.01)
            v=self.odom.twist.twist
            if math.hypot(v.linear.x,v.linear.y)<0.02 and abs(v.angular.z)<0.05:
                seen=True
                break
        self.check(name,seen,stop_wall_seconds=time.monotonic()-begin)

    def service_pause(self, paused):
        r=subprocess.run(['ign','service','-s','/world/sentry_basic/control',
                          '--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean',
                          '--timeout','3000','--req','pause: '+str(paused).lower()],
                         capture_output=True,text=True,timeout=6)
        if r.returncode or 'true' not in r.stdout:
            raise RuntimeError('Pause service failed: '+r.stdout+r.stderr)

    def run(self):
        self.spin_wall(3)
        self.check('feedback_available',self.odom is not None)
        for topic in ('/clock','/sim/ground_truth/odometry','/localization/odometry','/tf','/tf_static'):
            self.check('one_publisher '+topic,self.count_publishers(topic)==1,count=self.count_publishers(topic))
        self.check('initial_pose',math.hypot(self.odom.pose.pose.position.x,self.odom.pose.pose.position.y)<0.03 and abs(yaw(self.odom))<0.03)
        for name,x,y,z,duration,expected in [
            ('forward',0.2,0.,0.,3.,0.6),('lateral',0.,0.2,0.,3.,0.6),
            ('yaw',0.,0.,0.3,2.,0.6)]:
            a,b,_=self.move(x,y,z,duration)
            dx=b.pose.pose.position.x-a.pose.pose.position.x
            dy=b.pose.pose.position.y-a.pose.pose.position.y
            da=angle(yaw(b)-yaw(a))
            measure=da if z else (dx if x else dy)
            self.check(name,abs(measure-expected)<=0.15,dx=dx,dy=dy,dyaw=da,sim_duration=stamp(b)-stamp(a))
        target=math.pi/2
        self.move(z=0.3,seconds=max(0.02,angle(target-yaw(self.odom))/0.3))
        a,b,segment=self.move(x=0.2,seconds=3.)
        dx=b.pose.pose.position.x-a.pose.pose.position.x
        dy=b.pose.pose.position.y-a.pose.pose.position.y
        steady=segment[len(segment)//3:]
        vx=sum(m.twist.twist.linear.x for m in steady)/len(steady)
        vy=sum(m.twist.twist.linear.y for m in steady)/len(steady)
        self.check('body_frame_at_90deg',abs(dx)<0.15 and abs(dy-0.6)<0.15 and vx>0.17 and abs(vy)<0.03,
                   initial_yaw=yaw(a),world_dx=dx,world_dy=dy,body_vx=vx,body_vy=vy)
        # Backward and right lateral signs at arbitrary heading.
        for name,x,y in [('backward',-0.2,0.),('right',0.,-0.2)]:
            a,b,_=self.move(x=x,y=y,seconds=1.)
            dx=b.pose.pose.position.x-a.pose.pose.position.x
            dy=b.pose.pose.position.y-a.pose.pose.position.y
            bx=math.cos(yaw(a))*dx+math.sin(yaw(a))*dy
            by=-math.sin(yaw(a))*dx+math.cos(yaw(a))*dy
            self.check(name,abs(bx-x)<0.08 and abs(by-y)<0.08,body_dx=bx,body_dy=by)
        m=Twist();m.linear.x=0.2;m.angular.z=0.3
        self.spin_wall(0.6,m);self.stop_check('zero_command_stop',True)
        self.spin_wall(0.6,m);self.stop_check('watchdog_stop',False)
        # Combined linear norm clamp and yaw clamp.
        a,b,segment=self.move(x=3.,y=4.,z=2.,seconds=1.)
        steady=segment[len(segment)//3:]
        max_v=max(math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y) for m in steady)
        max_w=max(abs(m.twist.twist.angular.z) for m in steady)
        self.check('limits',0.45<max_v<=0.51 and 0.45<max_w<=0.51,max_linear=max_v,max_angular=max_w)
        bad=Twist();bad.linear.x=float('nan')
        self.spin_wall(0.5,m);self.pub.publish(bad);self.stop_check('nonfinite_command_stop',False)
        self.test_pause_resume()
        self.test_keyboard()
        self.spin_wall(1)
        tf=self.buf.lookup_transform('map','base_link',Time())
        p=self.odom.pose.pose.position
        error=math.hypot(tf.transform.translation.x-p.x,tf.transform.translation.y-p.y)
        self.check('tf_connected',error<0.02,pose_error=error)
        self.check('raw_state_matches',self.pairs>100 and self.max_error<1e-9,
                   matched_samples=self.pairs,max_difference=self.max_error)
        recent=self.samples[-100:]
        hz=(len(recent)-1)/(stamp(recent[-1])-stamp(recent[0]))
        self.check('feedback_50hz',45<=hz<=55,hz_sim_time=hz)
        self.check('no_fixture',not any('fixture' in n for n in self.get_node_names()))

    def test_pause_resume(self):
        # Observe Gazebo's scene poses, including while paused. Odometry stops
        # publishing when paused and can lag the actual final physics step.
        latest = [None, 0]
        reader = subprocess.Popen(
            ['ign', 'topic', '-e', '--json-output', '-t',
             '/world/sentry_basic/dynamic_pose/info'],
            stdout=subprocess.PIPE, text=True, start_new_session=True)
        def consume():
            for line in reader.stdout:
                try:
                    data = json.loads(line)
                    latest[0] = next(p for p in data['pose'] if p['name'] == 'sentry')
                    latest[1] += 1
                except (ValueError, KeyError, StopIteration):
                    pass
        worker = threading.Thread(target=consume, daemon=True)
        worker.start()
        moving = Twist()
        moving.linear.x, moving.angular.z = 0.2, 0.3
        def pause(value, keep_command=None):
            process = subprocess.Popen(
                ['ign', 'service', '-s', '/world/sentry_basic/control',
                 '--reqtype', 'ignition.msgs.WorldControl', '--reptype',
                 'ignition.msgs.Boolean', '--timeout', '3000', '--req',
                 'pause: ' + str(value).lower()], stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True)
            deadline = time.monotonic() + 6
            try:
                while process.poll() is None:
                    if time.monotonic() > deadline:
                        raise RuntimeError('Pause service timed out')
                    self.spin_wall(0.02, keep_command)
                out, err = process.communicate()
                if process.returncode or 'true' not in out:
                    raise RuntimeError(out + err)
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=3)
        try:
            deadline = time.monotonic() + 8
            while latest[0] is None:
                if time.monotonic() > deadline:
                    raise RuntimeError('No Gazebo scene pose')
                self.spin_wall(0.05)
            self.spin_wall(1.0, moving)
            self.check('moving_before_pause', self.odom.twist.twist.linear.x > 0.15)
            # Keep commanding until the service actually pauses the world.
            pause(True, moving)
            self.spin_wall(0.4)
            count = latest[1]
            self.spin_wall(0.8, moving)
            before = copy.deepcopy(latest[0])
            paused_count = latest[1] - count
            self.check('fresh_scene_pose_while_paused', paused_count > 2, messages=paused_count)
            pause(False)
            self.spin_wall(0.8)
            after = copy.deepcopy(latest[0])
            d = math.hypot(
                after['position'].get('x', 0) - before['position'].get('x', 0),
                after['position'].get('y', 0) - before['position'].get('y', 0))
            def scene_yaw(p):
                q = p['orientation']
                return 2 * math.atan2(q.get('z', 0), q.get('w', 0))
            dyaw = angle(scene_yaw(after) - scene_yaw(before))
            self.check('pause_resume_no_stale_command', d < 1e-9 and abs(dyaw) < 1e-9,
                       resume_displacement=d, resume_yaw_change=dyaw,
                       paused_scene_pose=before, resumed_scene_pose=after)
            self.stop_check('resume_stopped', False)
        finally:
            self.pub.publish(Twist())
            try:
                os.killpg(reader.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            reader.wait(timeout=3)
            worker.join(timeout=1)

    def test_keyboard(self):
        # Real keyboard executable through a PTY, including actual Escape exit.
        master,slave=pty.openpty()
        proc=subprocess.Popen(['ros2','run','sentry_gazebo','keyboard.py','--ros-args','-p','use_sim_time:=true'],
                               stdin=slave,stdout=slave,stderr=slave,close_fds=True,start_new_session=True)
        os.close(slave)
        try:
            transcript = b''
            deadline = time.monotonic() + 10
            while '控制接口已连接'.encode() not in transcript:
                self.spin_wall(0.05)
                if select.select([master], [], [], 0)[0]:
                    transcript += os.read(master, 65536)
                if proc.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Keyboard not ready: ' + transcript.decode(errors='replace'))
            (Path(__file__).resolve().parents[4]/'docs/simulation/evidence/keyboard-transcript.txt').write_bytes(transcript)
            for key, expected in [(b'w',(0.2,0.,0.)), (b's',(-0.2,0.,0.)),
                                  (b'a',(0.,0.2,0.)), (b'd',(0.,-0.2,0.)),
                                  (b'q',(0.,0.,0.3)), (b'e',(0.,0.,-0.3))]:
                for _ in range(15):
                    os.write(master,key)
                    self.spin_wall(0.07)
                v=self.odom.twist.twist
                actual=(v.linear.x,v.linear.y,v.angular.z)
                self.check('keyboard_' + key.decode(),
                           all(abs(a-b)<0.06 for a,b in zip(actual,expected)),
                           actual=actual,expected=expected)
                os.write(master,b' ')
                self.spin_wall(0.3)
            for _ in range(10):
                os.write(master,b'w');self.spin_wall(0.07)
            os.write(master,b' ')
            self.stop_check('keyboard_space_stop',False)
            for _ in range(10):
                os.write(master,b'w');self.spin_wall(0.07)
            os.write(master,b'\x1b')
            self.stop_check('keyboard_escape_stop',False)
            proc.wait(timeout=4)
            self.check('keyboard_exit',proc.returncode==0,returncode=proc.returncode)
        finally:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            proc.wait(timeout=4)
            os.close(master)

def main():
    rclpy.init()
    node=Acceptance()
    out=Path(__file__).resolve().parents[4]/'docs/simulation/evidence/motion.json'
    try:
        node.run()
    finally:
        node.pub.publish(Twist())
        out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(node.results,indent=2))
        node.destroy_node()
        rclpy.shutdown()
if __name__=='__main__':
    main()
