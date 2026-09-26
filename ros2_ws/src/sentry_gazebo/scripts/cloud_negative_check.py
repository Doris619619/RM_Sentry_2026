#!/usr/bin/env python3
"""Check cloud error handling in isolated ROS domain 27, separate from the live simulation."""
import json,time,os
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cloud_adapter import CloudAdapter

def main():
    """Use labelled synthetic invalid inputs only in the test domain; require fail-closed TF behavior."""
    assert os.environ.get('ROS_DOMAIN_ID')=='27'
    rclpy.init();adapter=CloudAdapter();test=rclpy.create_node('cloud_negative_fixture')
    executor=SingleThreadedExecutor();executor.add_node(adapter);executor.add_node(test)
    received={'raw':[],'body':[],'map':[]}
    for key,topic in [('raw','/sim/lidar/points'),('body','/filted_topic_3d'),('map','/aligned_points')]:
        test.create_subscription(PointCloud2,topic,lambda m,k=key:received[k].append(m),qos_profile_sensor_data)
    pub=test.create_publisher(PointCloud2,'/sim/lidar/native',qos_profile_sensor_data)
    broadcaster=TransformBroadcaster(test);records=[]
    def spin(seconds):
        """Allow discovery, timer expiry and TF delivery using wall time."""
        end=time.monotonic()+seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.02)
    def send(stamp,frame='sentry/base_link/lidar_sensor'):
        """Publish a deliberately small fixture with one finite hit and one invalid return."""
        header=Header();header.frame_id=frame;header.stamp.sec=stamp
        pub.publish(point_cloud2.create_cloud_xyz32(header,[(1.,2.,3.),(float('inf'),0.,0.)]))
    spin(2);send(1);spin(.8)
    assert received['raw'] and not received['body'] and not received['map']
    records.append({'name':'missing_tf_drops_processed_cloud','passed':True})
    before=len(received['raw']);send(1,'wrong_sensor');spin(.3);assert len(received['raw'])==before
    records.append({'name':'unexpected_native_frame_rejected','passed':True})
    for stamp in [1,2]:
        tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='base_link';tf.header.stamp.sec=stamp
        tf.transform.translation.x=2.;tf.transform.rotation.w=1.;broadcaster.sendTransform(tf);spin(.2)
    send(1);spin(.4)
    assert len(received['body'])==1 and len(received['map'])==1
    body=received['body'][-1];world=received['map'][-1]
    assert body.header.stamp.sec==world.header.stamp.sec==1
    b=list(point_cloud2.read_points(body,field_names=('x','y','z')))[0]
    w=list(point_cloud2.read_points(world,field_names=('x','y','z')))[0]
    assert body.width==world.width==1 and abs(float(b[2])-3.7)<1e-5 and abs(float(w[0])-3)<1e-5
    records.append({'name':'measurement_stamp_filter_and_extrinsics','passed':True})
    send(100);spin(.8);assert len(received['map'])==1
    records.append({'name':'no_latest_tf_fallback','passed':True})
    out=Path(__file__).resolve().parents[4]/'docs/simulation/part2/evidence/cloud-negative.json'
    out.write_text(json.dumps(records,indent=2));print(json.dumps(records,indent=2))
    executor.shutdown();test.destroy_node();adapter.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
