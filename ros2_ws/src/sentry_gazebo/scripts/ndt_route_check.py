#!/usr/bin/env python3
"""Drive complete routes using estimated localization and compare only in this evaluator with native truth."""
import argparse,json,math,time
from collections import deque
from pathlib import Path
import numpy as np,rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from closed_loop_check import ClosedLoop,autonomy_cases
from autonomy_geometry import StaticSafetyGrid
from dynamic_replan_check import goal,wait
from ndt_probe_check import stamp,yaw

def main():
    """Never teleport the robot or feed evaluator truth back into a runtime node."""
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    parser.add_argument('--return-trip',action='store_true');args=parser.parse_args()
    rclpy.init();live=ClosedLoop();truth=deque(maxlen=200);pending=deque();errors=[];records=[]
    live.node.create_subscription(Odometry,'/sim/ground_truth/odometry',truth.append,50)
    live.node.create_subscription(Odometry,'/localization/odometry',pending.append,20)
    grid=StaticSafetyGrid.load()
    # Pair estimates and actual poses by measurement time, without altering runtime messages.
    def collect():
        while pending:
            m=pending[0]
            if not truth or stamp(truth[-1])<stamp(m):break
            pending.popleft();t=min(truth,key=lambda t:abs(stamp(t)-stamp(m)))
            if abs(stamp(t)-stamp(m))>.025:continue
            p,q=m.pose.pose.position,t.pose.pose.position
            dyaw=math.atan2(math.sin(yaw(m.pose.pose.orientation)-yaw(t.pose.pose.orientation)),math.cos(yaw(m.pose.pose.orientation)-yaw(t.pose.pose.orientation)))
            errors.append({'sim':stamp(m),'xy':math.hypot(p.x-q.x,p.y-q.y),'yaw':abs(dyaw),
                'estimated':[p.x,p.y,yaw(m.pose.pose.orientation)],'truth':[q.x,q.y,yaw(t.pose.pose.orientation)]})
    try:
        wait(live,lambda:live.odom is not None and live.samples and live.status,30)
        live.spin(2);collect()
        publishers=live.node.get_publishers_info_by_topic('/localization/odometry')
        assert len(publishers)==1 and publishers[0].node_name=='sentry_estimated_odometry'
        forbidden=[n.node_name for n in live.node.get_subscriptions_info_by_topic('/sim/ground_truth/odometry')
            if n.node_name in ('sim_ndt','sentry_estimated_odometry','sentry_sim_adapter')]
        assert not forbidden
        targets=[autonomy_cases()[0][0][1]]
        if args.return_trip:targets.append(autonomy_cases()[0][0][0])
        for target in targets:
            live.mode.publish(String(data='auto'));live.spin(.4)
            live.samples=[];live.paths=[];live.current_path=None;errors.clear();pending.clear()
            start=time.monotonic();goal(live,target);live.spin(.4)
            while time.monotonic()-start<180:
                live.spin(.04);collect()
                if live.status.get('state') in ('stopped','arrived'):break
            sample=live.samples;end=sample[-1]
            endpoint=math.hypot(end['x']-target[0],end['y']-target[1])
            occupied=sum(not grid.points_free([[p['x'],p['y']]]) for p in sample)
            result={'goal':target,'status':live.status,'wall_duration':time.monotonic()-start,'endpoint_error':endpoint,
                'occupied_samples':occupied,'max_localization_error':max((e['xy'] for e in errors),default=math.inf),
                'rms_localization_error':float(np.sqrt(np.mean([e['xy']**2 for e in errors]))) if errors else None,
                'max_localization_yaw_error':max((e['yaw'] for e in errors),default=math.inf),
                'localization_publisher':publishers[0].node_name,'truth_input_consumers':forbidden,
                'samples':sample,'trajectories':live.paths,'localization_errors':list(errors)}
            result['passed']=live.status.get('state')=='arrived' and endpoint<=.15 and occupied==0 and result['max_localization_error']<.15
            records.append(result);Path(args.output).write_text(json.dumps(records,indent=2))
            print(json.dumps({k:v for k,v in result.items() if k not in ('samples','trajectories','localization_errors')}),flush=True)
            if not result['passed']:raise RuntimeError('NDT closed-loop acceptance failed')
    finally:
        live.mode.publish(String(data='stop'));live.spin(.3)
        Path(args.output).write_text(json.dumps(records,indent=2));live.node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
