#!/usr/bin/env python3
"""Measure GUI-inclusive process resources, clock progress and real lidar throughput."""
import argparse,json,time,statistics
from pathlib import Path
import psutil,rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data

def main():
    """Observe a specified launch tree without driving it; persist all wall-time samples."""
    parser=argparse.ArgumentParser();parser.add_argument('--pid',type=int,required=True)
    parser.add_argument('--duration',type=float,default=310);parser.add_argument('--output',required=True)
    args=parser.parse_args();rclpy.init();node=rclpy.create_node('sentry_runtime_monitor')
    sim=[None];clouds=[]
    node.create_subscription(Clock,'/clock',lambda m:sim.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
    node.create_subscription(PointCloud2,'/aligned_points',lambda m:clouds.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9),qos_profile_sensor_data)
    root=psutil.Process(args.pid);tracked={};samples=[];start=time.monotonic();last=start;last_sim=None;next_sample=start
    while time.monotonic()-start<args.duration:
        rclpy.spin_once(node,timeout_sec=.05);now=time.monotonic()
        if now<next_sample:continue
        processes=[root]+root.children(recursive=True) if root.is_running() else []
        cpu=rss=0;desc=[]
        for p in processes:
            try:
                if p.pid not in tracked:tracked[p.pid]=p;p.cpu_percent()
                cpu+=tracked[p.pid].cpu_percent();rss+=p.memory_info().rss
                desc.append({'pid':p.pid,'name':p.name(),'status':p.status()})
            except psutil.Error:pass
        sample={'wall':now-start,'sim':sim[0],'cpu_percent_one_core':cpu,'rss_mib':rss/2**20,
                'rtf':None if last_sim is None or sim[0] is None else (sim[0]-last_sim)/(now-last),'processes':desc}
        samples.append(sample);print(json.dumps(sample),flush=True)
        last=now;last_sim=sim[0];next_sample=now+5
    values=[s['rtf'] for s in samples if s['rtf'] is not None]
    output={'duration_wall':time.monotonic()-start,'rtf_mean':statistics.mean(values) if values else 0,
            'rtf_min':min(values) if values else 0,'rss_peak_mib':max(s['rss_mib'] for s in samples),
            'cpu_mean_one_core':statistics.mean(s['cpu_percent_one_core'] for s in samples[1:]),
            'cloud_hz_sim':(len(clouds)-1)/(clouds[-1]-clouds[0]) if len(clouds)>1 else 0,
            'all_alive':all(len(s['processes'])==len(samples[-1]['processes']) and all(p['status']!='zombie' for p in s['processes']) for s in samples[1:]),
            'samples':samples}
    Path(args.output).write_text(json.dumps(output,indent=2));print(json.dumps({k:v for k,v in output.items() if k!='samples'}))
    node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
