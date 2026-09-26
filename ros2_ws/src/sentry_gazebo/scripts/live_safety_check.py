#!/usr/bin/env python3
"""Inject bounded faults into this launch's real nodes and verify Gazebo actually stops."""
import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path
import psutil
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from closed_loop_check import ClosedLoop, ROOT, autonomy_cases

# Call only the running test world's control service and require a positive acknowledgement.
def world_control(request):
    result=subprocess.run(['ign','service','-s','/world/sentry_planning/control',
        '--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean',
        '--timeout','3000','--req',request],capture_output=True,text=True,timeout=5)
    if result.returncode or 'true' not in result.stdout:
        raise RuntimeError(result.stdout+result.stderr)

class SafetyCheck:
    # Keep launch identity explicit so signals cannot target unrelated user processes.
    def __init__(self, launch_pid):
        self.live=ClosedLoop()
        self.root=psutil.Process(launch_pid)
        if 'autonomy.launch.py' not in ' '.join(self.root.cmdline()):
            raise RuntimeError('PID is not the autonomous simulation launch')
        self.manual=self.live.node.create_publisher(Twist,'/sim/manual_cmd_vel',1)
        self.records=[]

    # Locate exactly one intended descendant from verified command lines.
    def process(self, executable):
        matches=[p for p in self.root.children(recursive=True)
                 if any(arg.endswith('/'+executable) for arg in p.cmdline())]
        if len(matches)!=1:raise RuntimeError('ambiguous simulation process: '+executable)
        return matches[0]

    # Wait for real feedback; do not substitute integrated commands for measured velocity.
    def wait(self, predicate, timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.live.spin(.03)
            if predicate():return
        raise RuntimeError('condition timeout; status='+str(self.live.status))

    # Start a real current-scene route, requiring measured motion before injecting a fault.
    def moving(self):
        start,goal=autonomy_cases()[0][0]
        self.live.prepare(start)
        self.live.request(start,goal)
        self.wait(lambda:self.live.samples and self.live.samples[-1]['speed']>.08,8)

    # Bound motion after a stop and reject an unrequested restart after recovery.
    def stopped(self, timeout=1.5):
        self.wait(lambda:self.live.samples and self.live.samples[-1]['speed']<.02,timeout)
        point=self.live.samples[-1]
        self.live.spin(.8)
        end=self.live.samples[-1]
        displacement=math.hypot(point['x']-end['x'],point['y']-end['y'])
        if displacement>.03:raise RuntimeError('unrequested movement after stop')
        return displacement

    # Suspend exactly one owned node, then always restore it before leaving the test.
    def fault(self, executable, expected):
        self.moving();process=self.process(executable);started=time.monotonic()
        process.send_signal(signal.SIGSTOP)
        try:
            self.wait(lambda:self.live.status.get('state')=='stopped',2)
            reason=self.live.status.get('reason','')
            latency=time.monotonic()-started
            displacement=self.stopped()
            if expected not in reason:raise RuntimeError('wrong fault reason: '+reason)
        finally:
            process.send_signal(signal.SIGCONT)
        self.live.spin(1.2)
        self.stopped()
        return {'reason':reason,'stop_status_latency_wall':latency,'post_stop_displacement':displacement}

    # The ROS bridge itself is unavailable, so query actual Gazebo transport independently.
    def bridge_hang(self):
        self.moving();process=self.process('parameter_bridge');started=time.monotonic()
        process.send_signal(signal.SIGSTOP)
        def native():
            response=subprocess.run(['ign','topic','-e','-t','/model/sentry/odometry','-n','1','--json-output'],
                                    capture_output=True,text=True,timeout=5)
            if response.returncode:raise RuntimeError('native odometry probe failed')
            return json.loads(response.stdout)
        try:
            self.live.spin(.8)
            first=native();linear=first.get('twist',{}).get('linear',{})
            speed=math.hypot(linear.get('x',0.),linear.get('y',0.))
            if speed>.02:raise RuntimeError('native watchdog did not stop with bridge frozen')
            self.live.spin(.8);second=native()
            a=first['pose']['position'];b=second['pose']['position']
            drift=math.hypot(a.get('x',0.)-b.get('x',0.),a.get('y',0.)-b.get('y',0.))
            if drift>.03:raise RuntimeError('native pose drifted during bridge failure')
        finally:
            process.send_signal(signal.SIGCONT)
        self.live.spin(1.2);self.stopped()
        return {'native_speed':speed,'native_drift':drift,'native_samples':[first,second],
                'wall_duration':time.monotonic()-started,'old_motion_replayed':False}

    # A frozen supervisor cannot publish its own failure status; assess motion independently first.
    def supervisor_hang(self):
        self.moving();process=self.process('autonomy_guard.py')
        process.send_signal(signal.SIGSTOP);started=time.monotonic()
        try:
            displacement=self.stopped()
        finally:
            process.send_signal(signal.SIGCONT)
        self.live.spin(1.2);self.stopped()
        if self.live.status.get('state')!='stopped':raise RuntimeError('supervisor resumed old trajectory')
        return {'reason':self.live.status.get('reason'),'post_stop_displacement':displacement,
                'wall_duration':time.monotonic()-started}

    # A real physics pause must freeze simulation time and discard the active trajectory.
    def pause(self):
        self.moving()
        world_control('pause: true')
        self.live.spin(.2)
        stamp=self.live.node.get_clock().now().nanoseconds
        try:
            self.live.spin(.9)
            if self.live.node.get_clock().now().nanoseconds!=stamp:raise RuntimeError('clock advanced during pause')
        finally:
            world_control('pause: false')
        self.live.spin(.3);self.stopped()
        if self.live.status.get('state')!='stopped':raise RuntimeError('pause did not latch stop')
        return {'clock_frozen':True,'reason':self.live.status.get('reason'),'stale_motion_replayed':False}

    # Manual selection cancels automatic output; auto selection alone cannot resume a previous goal.
    def takeover(self):
        self.moving();self.live.mode.publish(String(data='manual'));self.live.spin(.3);self.stopped()
        start=self.live.samples[-1];cmd=Twist();cmd.linear.x=.2
        end=time.monotonic()+1.
        while time.monotonic()<end:
            self.manual.publish(cmd);self.live.spin(.03)
        self.manual.publish(Twist());self.live.spin(.3)
        measured=self.live.samples[-1]
        movement=math.hypot(measured['x']-start['x'],measured['y']-start['y'])
        if not .12<movement<.3:raise RuntimeError('manual takeover did not move as commanded')
        self.live.mode.publish(String(data='auto'));self.live.spin(.3);self.stopped()
        return {'manual_distance':movement,'auto_requires_new_goal':True}

    # An unreachable replacement goal immediately cancels old motion and later reports planning failure.
    def unreachable(self):
        self.moving()
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.live.node.get_clock().now().to_msg()
        msg.pose.position.x=-4.269;msg.pose.position.y=4.846;msg.pose.orientation.w=1.
        self.live.pub.publish(msg)
        self.live.spin(.3);self.stopped()
        self.wait(lambda:self.live.status.get('state')=='stopped',10)
        if 'no valid path' not in self.live.status.get('reason',''):raise RuntimeError('unexpected failure '+str(self.live.status))
        return dict(self.live.status)

    # Rewind while moving; latch stop and require a complete process restart.
    def reset(self):
        self.moving()
        self.wait(lambda:self.live.node.get_clock().now().nanoseconds*1e-9>15,25)
        before=self.live.samples[-1]['sim']
        world_control('reset: {all: true}')
        self.wait(lambda:self.live.samples[-1]['sim']<before,8)
        self.live.spin(2);self.stopped()
        self.live.mode.publish(String(data='auto'));self.live.spin(.5);self.stopped()
        if 'restart required' not in self.live.status.get('reason',''):
            raise RuntimeError('unsupported rewind did not require a full restart')
        return {'old_goal_resumed':False,'in_place_rewind_supported':False,
                'required_recovery':'complete launch restart','status':dict(self.live.status)}

    # Persist each actual outcome so a later test exception cannot erase earlier failures.
    def run(self, name, action, output):
        started=time.monotonic()
        try:
            detail=action();record={'name':name,'passed':True,'detail':detail}
        except Exception as error:
            record={'name':name,'passed':False,'error':str(error)}
        record['wall_seconds']=time.monotonic()-started
        record['actual_truth_samples']=list(self.live.samples)
        self.records.append(record);output.write_text(json.dumps(self.records,indent=2))
        print(json.dumps({k:v for k,v in record.items() if k!='actual_truth_samples'}),flush=True)
        if not record['passed']:raise RuntimeError(name+' failed')

# Fault tests affect only this isolated launch; teardown restores motion to stop.
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--pid',type=int,required=True)
    parser.add_argument('--only',default='')
    parser.add_argument('--output',default=str(ROOT/'docs/simulation/autonomy/evidence/live-safety.json'))
    args=parser.parse_args();rclpy.init();qa=SafetyCheck(args.pid);out=Path(args.output)
    try:
        qa.wait(lambda:qa.live.odom is not None and bool(qa.live.status),30)
        cases=[
            ('cloud_loss',lambda:qa.fault('cloud_adapter.py','cloud')),
            ('controller_hang',lambda:qa.fault('trajectory_tracking_node','controller')),
            ('localization_loss',lambda:qa.fault('sim_adapter.py','localization')),
            ('bridge_hang',qa.bridge_hang),('supervisor_hang',qa.supervisor_hang),
            ('pause_resume',qa.pause),('manual_takeover',qa.takeover),
            ('unreachable_goal',qa.unreachable),('clock_rewind_requires_restart',qa.reset)]
        if args.only and args.only not in dict(cases):raise ValueError('unknown safety case')
        for name,action in cases:
            if not args.only or args.only==name:qa.run(name,action,out)
    finally:
        qa.live.mode.publish(String(data='stop'));qa.live.spin(.3)
        qa.live.node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
