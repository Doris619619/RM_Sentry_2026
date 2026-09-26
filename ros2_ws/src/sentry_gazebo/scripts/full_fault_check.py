#!/usr/bin/env python3
"""Inject referee corruption, real MCU stalls and manual takeover into the complete physical loop."""
import argparse,json,math,signal,time
from pathlib import Path
import psutil,rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from full_system_check import FullCheck

# Start each fault from measured motion, using only actual referee wire scenarios.
def moving(q):
    q.scenario.publish(String(data='stop'));q.spin(1.)
    q.mode.publish(String(data='auto'));q.spin(.3)
    p=q.samples[-1]
    scenario='supply' if p['y']>-1 else 'patrol'
    q.scenario.publish(String(data=scenario))
    q.wait(lambda:q.status.get('state')=='tracking' and q.samples[-1]['speed']>.12,35)
    q.spin(.3)
    return scenario

# Require an observed low physical speed and inspect the actual stop reason.
def stopped(q,start):
    q.wait(lambda:q.samples[-1]['speed']<.02,4.)
    end=q.samples[-1]
    return {'stop_wall':time.monotonic()-start[0],
        'travel':math.hypot(end['x']-start[1]['x'],end['y']-start[1]['y']),'pose':end}

# Fault only the verified MCU child and always restore it and stop the full simulator.
def main():
    p=argparse.ArgumentParser();p.add_argument('--pid',type=int,required=True);p.add_argument('--output',required=True);a=p.parse_args()
    root=psutil.Process(a.pid)
    assert 'autonomy.launch.py' in ' '.join(root.cmdline())
    rclpy.init();q=FullCheck();records=[];paused=None
    try:
        wiring=q.wiring();mcu=psutil.Process(q.mcu['mcu_pid'])
        assert mcu.pid in [v.pid for v in root.children(recursive=True)]
        assert 'mcu_communicator' in ' '.join(mcu.cmdline())
        for kind in ['game_end','bad_referee_crc','mcu_process_stall']:
            moving(q);q.errors=[];initial=dict(q.samples[-1]);start=(time.monotonic(),initial)
            if kind=='game_end':q.scenario.publish(String(data='stop'))
            elif kind=='bad_referee_crc':q.scenario.publish(String(data='corrupt_referee'))
            else:mcu.send_signal(signal.SIGSTOP);paused=mcu
            result=stopped(q,start)
            q.wait(lambda:q.status.get('state')=='stopped',3.)
            result.update({'case':kind,'status':dict(q.status),'mcu_status':dict(q.mcu)})
            if paused:
                paused.send_signal(signal.SIGCONT);paused=None
            elif kind=='bad_referee_crc':
                q.scenario.publish(String(data='patrol'))
            q.spin(1.5)
            end=q.samples[-1]
            result['unrequested_restart_displacement']=math.hypot(end['x']-result['pose']['x'],end['y']-result['pose']['y'])
            result['passed']=result['stop_wall']<1.6 and result['travel']<.4 and result['unrequested_restart_displacement']<.03 and q.status.get('state')=='stopped'
            records.append(result);Path(a.output).write_text(json.dumps(records,indent=2));print(json.dumps(result),flush=True)
            if not result['passed']:raise RuntimeError(kind+' failed')
        moving(q)
        q.mode.publish(String(data='manual'));q.spin(.4)
        manual=q.node.create_publisher(Twist,'/sim/manual_cmd_vel',1)
        q.spin(.2);origin=dict(q.samples[-1]);cmd=Twist();cmd.linear.x=.1
        until=time.monotonic()+1.
        while time.monotonic()<until:manual.publish(cmd);q.spin(.04)
        moved=math.hypot(q.samples[-1]['x']-origin['x'],q.samples[-1]['y']-origin['y'])
        q.mode.publish(String(data='stop'));start=(time.monotonic(),dict(q.samples[-1]));result=stopped(q,start)
        result.update({'case':'manual_takeover','manual_distance':moved,'passed':moved>.03 and result['stop_wall']<.6})
        records.append(result);Path(a.output).write_text(json.dumps(records,indent=2));print(json.dumps(result),flush=True)
        if not result['passed']:raise RuntimeError('manual takeover failed')
    except Exception as error:
        records.append({'passed':False,'error':str(error),'status':q.status,'mcu':q.mcu})
        raise
    finally:
        if paused:paused.send_signal(signal.SIGCONT)
        q.scenario.publish(String(data='stop'));q.mode.publish(String(data='stop'));q.spin(.5)
        Path(a.output).write_text(json.dumps(records,indent=2));q.node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
