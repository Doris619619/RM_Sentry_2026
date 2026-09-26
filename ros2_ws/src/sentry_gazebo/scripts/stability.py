#!/usr/bin/env python3
"""Wall-clock stability sample; aggregate RSS (shared pages may be counted twice)."""
import json
import os
from pathlib import Path
import statistics
import time
import psutil
import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
n=rclpy.create_node('sentry_stability_monitor')
sim=[None]
n.create_subscription(Clock,'/clock',lambda m: sim.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
root=psutil.Process(int(os.environ['SENTRY_LAUNCH_PID']))
processes={}
samples=[]
start=time.monotonic()
previous_wall=start
previous_sim=None
next_sample=start
while time.monotonic()-start < 310:
    rclpy.spin_once(n,timeout_sec=0.1)
    now=time.monotonic()
    if now < next_sample: continue
    alive=root.is_running()
    children=root.children(recursive=True) if alive else []
    rss=cpu=0.
    descriptions=[]
    for proc in [root]+children:
        try:
            if proc.pid not in processes:
                processes[proc.pid]=proc
                proc.cpu_percent()
            p=processes[proc.pid]
            cpu+=p.cpu_percent()
            rss+=p.memory_info().rss
            descriptions.append(dict(pid=p.pid,name=p.name()))
        except psutil.Error: pass
    rtf=None if previous_sim is None or sim[0] is None else (sim[0]-previous_sim)/(now-previous_wall)
    sample=dict(wall_seconds=now-start,sim_time=sim[0],rtf=rtf,cpu_percent_one_core=cpu,rss_mib=rss/1048576,alive=alive,processes=descriptions)
    samples.append(sample)
    print(json.dumps(sample),flush=True)
    previous_wall,previous_sim=now,sim[0]
    next_sample=now+5
valid=[s for s in samples[1:] if s['rtf'] is not None]
summary=dict(duration_wall=time.monotonic()-start,
             rtf_mean=statistics.mean(s['rtf'] for s in valid),
             rtf_min=min(s['rtf'] for s in valid),
             cpu_mean_one_core=statistics.mean(s['cpu_percent_one_core'] for s in valid),
             rss_peak_mib=max(s['rss_mib'] for s in samples),
             all_alive=all(s['alive'] for s in samples),
             samples=samples)
out=Path(__file__).resolve().parents[4]/'docs/simulation/evidence/stability.json'
out.write_text(json.dumps(summary,indent=2))
n.destroy_node()
rclpy.shutdown()
