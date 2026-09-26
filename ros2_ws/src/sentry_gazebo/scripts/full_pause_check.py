#!/usr/bin/env python3
"""Verify paused simulation time and bounded physical braking through the full serial command path."""
import argparse,json,math,subprocess,time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from full_system_check import FullCheck
from full_fault_check import moving

# The native control service changes simulation lifecycle; it never replaces robot state.
def pause(value):
    subprocess.run(['ign','service','-s','/world/sentry_planning/control',
        '--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean',
        '--timeout','3000','--req','pause: '+str(value).lower()],check=True,capture_output=True,timeout=5)

def main():
    """Use the pose observed after pause took effect, separating service latency from resume braking."""
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
    rclpy.init();q=FullCheck();result={};paused=False
    try:
        q.wiring();moving(q);before=dict(q.samples[-1])
        pause(True);paused=True;q.spin(.8);frozen=dict(q.samples[-1]);q.spin(.5)
        result['frozen_sim_delta']=q.samples[-1]['sim']-frozen['sim']
        result['pending_service_motion']=math.hypot(frozen['x']-before['x'],frozen['y']-before['y'])
        q.samples=[];pause(False);paused=False;q.spin(2.)
        result['resume_braking_displacement']=max(math.hypot(s['x']-frozen['x'],s['y']-frozen['y']) for s in q.samples)
        result['final_speed']=q.samples[-1]['speed'];result['status']=q.status;result['samples']=q.samples
        result['passed']=result['frozen_sim_delta']==0 and result['resume_braking_displacement']<.08 and result['final_speed']<.02 and q.status.get('state')=='stopped'
        if not result['passed']:raise RuntimeError('full serial pause recovery failed')
    except Exception as e:result.update({'passed':False,'error':str(e)});raise
    finally:
        if paused:pause(False)
        q.scenario.publish(String(data='stop'));q.mode.publish(String(data='stop'));q.spin(.5)
        Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='samples'}))
        q.node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
