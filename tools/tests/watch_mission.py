#!/usr/bin/env python3
"""Sends a mission, prints status, reports where the blocks ended up (asks Gazebo directly)."""
import sys, json, re, subprocess, time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from nav_msgs.msg import Odometry
def block(name):
    o = subprocess.run(['gz','model','-m',name,'-p'],capture_output=True,text=True,timeout=8).stdout
    m = re.findall(r'\[(-?[\d.e-]+) (-?[\d.e-]+) (-?[\d.e-]+)\]', o)
    return [round(float(v),3) for v in m[0]] if m else None
rclpy.init(); n=Node('watch',parameter_overrides=[rclpy.Parameter('use_sim_time',value=True)])
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
st=[]; dets=[None]; gt=[None]
n.create_subscription(String,'mission/status',lambda m:(st.append(json.loads(m.data)),print(time.strftime('%X'),json.loads(m.data)['status'],flush=True)),latched)
n.create_subscription(String,'object/detections',lambda m:dets.__setitem__(0,m.data),10)
n.create_subscription(Odometry,'ground_truth/odom',lambda m:gt.__setitem__(0,m),10)
pub=n.create_publisher(String,'mission/command',10)
last=0; track=sys.argv[2].split(',')[0]; secs=float(sys.argv[1]); blocks=sys.argv[2].split(','); missions=sys.argv[3:]
t=time.time()
while time.time()-t<8 or pub.get_subscription_count()==0 or dets[0] is None:
    rclpy.spin_once(n,timeout_sec=0.1)
    if time.time()-t>60: print('not ready'); break
print('detections at start:',dets[0]); print('blocks before',{b:block(b) for b in blocks})
if gt[0]: print('robot at',round(gt[0].pose.pose.position.x,3),round(gt[0].pose.pose.position.y,3))
for mission in missions:
    st.clear(); pub.publish(String(data=mission)); print('>>>',mission,flush=True); t=time.time()
    while time.time()-t<secs:
        rclpy.spin_once(n,timeout_sec=0.1)
        if track and time.time()-last>2.0:
            last=time.time(); print('   ',round(time.time()-t,1),'s',track,block(track),flush=True)
        if st and any(s['status'].startswith(('DONE','FAILED','ABORTED')) for s in st): break
    print('blocks now',{b:block(b) for b in blocks},flush=True)
    if gt[0]: print('robot at',round(gt[0].pose.pose.position.x,3),round(gt[0].pose.pose.position.y,3))
