#!/usr/bin/env python3
"""Inject a real NDT process stall and an inaccurate initial-pose hint; verify stop and scan-based recovery."""
import argparse,json,math,signal,time
from pathlib import Path
import psutil,rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from closed_loop_check import ClosedLoop,autonomy_cases
from dynamic_replan_check import goal,wait

def main():
    """Only signal the NDT child of the explicitly verified autonomous launch."""
    p=argparse.ArgumentParser();p.add_argument('--pid',type=int,required=True);p.add_argument('--output',required=True);a=p.parse_args()
    root=psutil.Process(a.pid);assert 'autonomy.launch.py' in root.cmdline()
    candidates=[c for c in root.children(recursive=True) if '/hdl_localization_node' in ' '.join(c.cmdline())]
    assert len(candidates)==1;ndt=candidates[0];suspended=False
    rclpy.init();live=ClosedLoop();result={}
    hints=live.node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    try:
        wait(live,lambda:live.odom is not None and live.samples and live.status,30)
        live.mode.publish(String(data='auto'));live.spin(.3);goal(live,autonomy_cases()[0][0][1])
        wait(live,lambda:live.samples[-1]['speed']>.1,20)
        began=time.monotonic();ndt.send_signal(signal.SIGSTOP);suspended=True
        wait(live,lambda:live.status.get('state')=='stopped' and live.samples[-1]['speed']<.02,4)
        latency=time.monotonic()-began;before=live.samples[-1];live.spin(1.2);after=live.samples[-1]
        drift=math.hypot(after['x']-before['x'],after['y']-before['y'])
        reason=live.status.get('reason');ndt.send_signal(signal.SIGCONT);suspended=False
        live.spin(2)
        assert live.status.get('state')=='stopped' and live.samples[-1]['speed']<.02
        hint=PoseWithCovarianceStamped();hint.header.frame_id='map';hint.header.stamp=live.node.get_clock().now().to_msg()
        hint.pose.pose=live.odom.pose.pose;hint.pose.pose.position.x+=.15;hint.pose.pose.position.y-=.10
        hints.publish(hint);live.spin(5)
        actual=live.samples[-1];est=live.odom.pose.pose.position
        recovered=math.hypot(actual['x']-est.x,actual['y']-est.y)
        result={'stop_latency_wall':latency,'reason':reason,'post_stop_drift':drift,
            'old_goal_resumed':False,'initialpose_hint_offset':[.15,-.10],'recovered_xy_error':recovered,
            'passed':latency<1.5 and drift<.03 and recovered<.15}
    finally:
        if suspended:ndt.send_signal(signal.SIGCONT)
        live.mode.publish(String(data='stop'));live.spin(.3)
        result['actual_samples']=live.samples
        Path(a.output).write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k!='actual_samples'}))
        live.node.destroy_node();rclpy.shutdown()
    if not result.get('passed'):raise SystemExit(1)
if __name__=='__main__':main()
