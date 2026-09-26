#!/usr/bin/env python3
"""Record only the owned Gazebo/RViz windows while a real referee-triggered full-system route runs."""
import argparse,json,signal,subprocess,time
from pathlib import Path
import psutil,rclpy
from std_msgs.msg import String
from full_system_check import FullCheck

# Resolve both visible windows from the verified launch descendants, without touching the pointer.
def windows(pid):
    process=psutil.Process(pid)
    if 'autonomy.launch.py' not in ' '.join(process.cmdline()):raise RuntimeError('unexpected launch')
    owned={p.pid for p in process.children(recursive=True)};found={}
    for line in subprocess.check_output(['wmctrl','-lp'],text=True).splitlines():
        parts=line.split(None,4)
        if len(parts)<5 or int(parts[2]) not in owned:continue
        role='gazebo' if parts[4]=='Gazebo' else 'rviz' if parts[4].endswith(' - RViz') else None
        if role:found[role]=parts[0]
    if len(found)!=2:raise RuntimeError('both GUI windows must be visible')
    return [found['gazebo'],found['rviz']]

def main():
    """Keep original frames and timestamped measured events; video is not a replacement for numerical acceptance."""
    p=argparse.ArgumentParser();p.add_argument('--pid',required=True,type=int);p.add_argument('--output-dir',required=True);a=p.parse_args()
    out=Path(a.output_dir);out.mkdir(parents=True,exist_ok=True);ids=windows(a.pid)
    pipeline=['gst-launch-1.0','-e','compositor','name=mix','background=black','sink_0::xpos=0','sink_1::xpos=960','!',
        'videoconvert','!','video/x-raw,format=I420,colorimetry=bt709,width=1920,height=680,framerate=5/1','!',
        'vp8enc','deadline=1','cpu-used=8','threads=2','target-bitrate=2000000','!','webmmux','!',
        'filesink','location='+str(out/'full-demo-raw.webm')]
    for i,xid in enumerate(ids):
        pipeline+=['ximagesrc','xid='+xid,'use-damage=false','show-pointer=false','num-buffers=475','!',
            'video/x-raw,framerate=5/1','!','videoscale','add-borders=true','!','videoconvert','!',
            'video/x-raw,format=I420,colorimetry=bt709,width=960,height=680,pixel-aspect-ratio=1/1','!','mix.sink_'+str(i)]
    rclpy.init();q=FullCheck();q.wiring();capture=None;result={};began=time.monotonic()
    try:
        capture=subprocess.Popen(pipeline,stdout=open(out/'full-demo-capture.log','w'),stderr=subprocess.STDOUT)
        q.spin(5.);target=(-3.219,2.146) if q.samples[-1]['y']<-1 else (-.919,-4.454)
        scenario='patrol' if target[1]>0 else 'supply'
        result['scenario_start_wall']=time.monotonic()-began;result['scenario']=scenario
        episode=q.episode(scenario,target)
        result['arrival_wall']=time.monotonic()-began;result['episode']=episode
        q.spin(max(0.,96.-(time.monotonic()-began)));capture.wait(timeout=15)
        result['capture_exit']=capture.returncode;result['passed']=episode['passed'] and capture.returncode==0
        if not result['passed']:raise RuntimeError('demo route/capture failed')
    finally:
        q.scenario.publish(String(data='stop'));q.mode.publish(String(data='stop'));q.spin(.3)
        if capture and capture.poll() is None:capture.send_signal(signal.SIGINT);capture.wait(timeout=10)
        Path(out/'full-demo-events.json').write_text(json.dumps(result,indent=2))
        q.node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
