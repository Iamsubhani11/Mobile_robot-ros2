#!/usr/bin/env python3
"""Prints commanded vs actual arm joints once a second while a mission runs."""
import sys,time,json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
A=['arm_waist_joint','arm_shoulder_joint','arm_elbow_joint','arm_wrist_joint','gripper_joint']
rclpy.init(); n=Node('trace')
js={};cmd={}
n.create_subscription(JointState,'joint_states',lambda m:js.update(zip(m.name,m.position)),20)
def c(m):
    if m.points: cmd.update(zip(m.joint_names,m.points[-1].positions))
n.create_subscription(JointTrajectory,'arm_controller/joint_trajectory',c,20)
n.create_subscription(JointTrajectory,'gripper_controller/joint_trajectory',c,20)
pub=n.create_publisher(String,'mission/command',10)
t=time.time()
while time.time()-t<4: rclpy.spin_once(n,timeout_sec=.1)
pub.publish(String(data=sys.argv[1])); t=time.time(); last=0
while time.time()-t<float(sys.argv[2]):
    rclpy.spin_once(n,timeout_sec=.05)
    if time.time()-last>1.5 and js:
        last=time.time(); print(f'{time.time()-t:5.1f}',' '.join(f'{js.get(j,0):+.2f}({cmd.get(j,0):+.2f})' for j in A),flush=True)
