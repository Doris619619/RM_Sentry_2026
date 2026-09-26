#!/usr/bin/env python3
"""Live keyboard lifecycle regression: duplicate lock, Ctrl+C, disconnected terminal."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import time
import psutil
from acceptance import Acceptance, rclpy, Twist

ROOT=Path(__file__).resolve().parents[4]
COMMAND=['ros2','run','sentry_gazebo','keyboard.py','--ros-args','-p','use_sim_time:=true']

def run():
    rclpy.init()
    n=Acceptance()
    n.spin_wall(3)
    try:
        for mode in ('duplicate', 'sigint', 'eof'):
            master,slave=pty.openpty()
            p=subprocess.Popen(COMMAND,stdin=slave,stdout=slave,stderr=slave,start_new_session=True)
            os.close(slave)
            try:
                text=b'';end=time.monotonic()+10
                while '控制接口已连接'.encode() not in text:
                    n.spin_wall(0.05)
                    if select.select([master],[],[],0)[0]:text+=os.read(master,65536)
                    if p.poll() is not None or time.monotonic()>end:
                        raise RuntimeError(text.decode(errors='replace'))
                if mode=='duplicate':
                    m2,s2=pty.openpty()
                    second=subprocess.Popen(COMMAND,stdin=s2,stdout=s2,stderr=s2,start_new_session=True)
                    os.close(s2)
                    try:
                        second.wait(timeout=6)
                        response=os.read(m2,65536).decode(errors='replace')
                        n.check('duplicate_keyboard_rejected',second.returncode!=0 and '已有键盘' in response,
                                returncode=second.returncode,message=response.strip())
                    finally:
                        try:os.killpg(second.pid,signal.SIGTERM)
                        except ProcessLookupError:pass
                        os.close(m2)
                for _ in range(18):
                    os.write(master,b'w');n.spin_wall(0.07)
                n.check(mode+'_moving_before_exit',n.odom.twist.twist.linear.x>0.15,
                        vx=n.odom.twist.twist.linear.x)
                children=psutil.Process(p.pid).children(recursive=True)
                if mode=='eof':
                    os.close(master);master=None
                elif mode=='sigint':
                    os.killpg(p.pid,signal.SIGINT)
                else:
                    os.write(master,b'\x1b')
                n.stop_check(mode+'_stops',False)
                p.wait(timeout=5)
                n.spin_wall(0.3)
                remaining=[]
                for child in children:
                    try:
                        if child.is_running() and child.status()!=psutil.STATUS_ZOMBIE:remaining.append(child.pid)
                    except psutil.NoSuchProcess:pass
                n.check(mode+'_no_orphan',not remaining,remaining=remaining,wrapper_exit=p.returncode)
            finally:
                try:os.killpg(p.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                p.wait(timeout=5)
                if master is not None:os.close(master)
        n.spin_wall(2)
        count=sum(i.node_name=='sentry_keyboard' for i in n.get_publishers_info_by_topic('/cmd_vel'))
        n.check('no_keyboard_publishers_after_exit',count==0,count=count)
    finally:
        n.pub.publish(Twist())
        (ROOT/'docs/simulation/evidence/keyboard-lifecycle.json').write_text(json.dumps(n.results,indent=2))
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':
    run()
