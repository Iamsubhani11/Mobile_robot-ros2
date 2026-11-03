#!/usr/bin/env python3
"""Runs against a live simulation. Measures the real motion against what was commanded."""
import math, sys, time, json
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
ARM = ['arm_waist_joint','arm_shoulder_joint','arm_elbow_joint','arm_wrist_joint']
class T(Node):
    def __init__(s):
        super().__init__('test_motion', parameter_overrides=[rclpy.Parameter('use_sim_time', value=True)])
        s.gt=None; s.od=None; s.js={}; s.scan=None
        s.create_subscription(Odometry,'ground_truth/odom',lambda m:setattr(s,'gt',m),10)
        s.create_subscription(Odometry,'odom',lambda m:setattr(s,'od',m),10)
        s.create_subscription(JointState,'joint_states',lambda m:s.js.update(zip(m.name,m.position)),10)
        s.create_subscription(LaserScan,'scan',lambda m:setattr(s,'scan',m),rclpy.qos.qos_profile_sensor_data)
        s.cmd=s.create_publisher(Twist,'cmd_vel',10)
        s.arm=s.create_publisher(JointTrajectory,'arm_controller/joint_trajectory',10)
        s.grip=s.create_publisher(JointTrajectory,'gripper_controller/joint_trajectory',10)
    def spin(s,t):
        st=s.get_clock().now()
        while (s.get_clock().now()-st).nanoseconds*1e-9<t: rclpy.spin_once(s,timeout_sec=0.01)
    def pose(s):
        p=s.gt.pose.pose; q=p.orientation
        return p.position.x,p.position.y,p.position.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),q
    def drive(s,vx,vy,wz,t):
        m=Twist(); m.linear.x=float(vx);m.linear.y=float(vy);m.angular.z=float(wz)
        st=s.get_clock().now()
        while (s.get_clock().now()-st).nanoseconds*1e-9<t:
            s.cmd.publish(m); s.spin(0.05)
        s.cmd.publish(Twist()); s.spin(1.0)
    def move(s,vals,grip=None,t=2.5):
        d=Duration(sec=int(t),nanosec=int((t%1)*1e9))
        s.arm.publish(JointTrajectory(joint_names=ARM,points=[JointTrajectoryPoint(positions=[float(v) for v in vals],time_from_start=d)]))
        if grip is not None:
            s.grip.publish(JointTrajectory(joint_names=['gripper_joint'],points=[JointTrajectoryPoint(positions=[float(grip)],time_from_start=d)]))
        s.spin(t+1.0)
rclpy.init(); n=T()
n.spin(3.0)
while n.gt is None or not n.js: n.spin(0.5)
x,y,z,th,_=n.pose(); print(f'settled at z={z*1000:.2f} mm (expect about 0)')
results=[]
for name,(vx,vy,wz,t) in dict(forward=(.2,0,0,3),backward=(-.2,0,0,3),strafe_left=(0,.2,0,3),strafe_right=(0,-.2,0,3),turn_left=(0,0,.8,3),diagonal=(.15,.15,0,3)).items():
    a=n.pose(); n.drive(vx,vy,wz,t); b=n.pose()
    c,s=math.cos(a[3]),math.sin(a[3]); dx=b[0]-a[0]; dy=b[1]-a[1]
    bx,by=c*dx+s*dy,-s*dx+c*dy; dth=math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3]))
    print(f'{name:13s} cmd ({vx*t:+.2f},{vy*t:+.2f},{wz*t:+.2f}) got ({bx:+.3f},{by:+.3f},{dth:+.3f}) odom ({n.od.pose.pose.position.x:+.2f},{n.od.pose.pose.position.y:+.2f})')
for name,q,g in (('stow',[0,-.95,-.6,.9],0.),('home',[0,0,0,0],0.),('waist+shoulder',[1.2,-.4,.5,.3],.5),('reach',[0,.045,.089,.354],1.5),('home',[0,0,0,0],0.)):
    n.move(q,g,3.0)
    err=max(abs(n.js[j]-v) for j,v in zip(ARM,q))
    p=n.pose(); tilt=2*math.asin(math.hypot(p[4].x,p[4].y))
    print(f'arm {name:15s} max error {math.degrees(err):5.2f} deg, gripper {n.js["gripper_joint"]:.3f} (cmd {g}), finger_left {n.js["gripper_finger_left_joint_mimic"]:.3f} finger_r {n.js["gripper_finger_right_joint_mimic"]:.3f} finger_r2 {n.js["gripper_finger_right_joint_mimic"]:.3f}, body tilt {math.degrees(tilt):.2f} deg')
r=n.scan
vals=[(i,d) for i,d in enumerate(r.ranges) if math.isfinite(d)]
print(f'scan: {len(r.ranges)} beams, valid {len(vals)}, min {min(d for _,d in vals):.3f} m, frame {r.header.frame_id}, inf/below-min beams {len(r.ranges)-len(vals)}')
