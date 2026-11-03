#!/usr/bin/env python3
"""Drives the mock 'real' robot and checks the Gazebo twin under /sim follows it."""
import json, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState, LaserScan
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
ok = True
def check(name, cond, detail=''):
    global ok; ok &= bool(cond); print(('PASS ' if cond else 'FAIL ') + name, detail, flush=True)
A = ['arm_waist_joint', 'arm_shoulder_joint', 'arm_elbow_joint', 'arm_wrist_joint']
rclpy.init(); n = Node('tw'); st = [None]; real = {}; sim = {}; od = [None]; gt = [None]; c = dict(scan=0, cam=0)
n.create_subscription(String, 'twin/status', lambda m: st.__setitem__(0, json.loads(m.data)), 10)
n.create_subscription(JointState, 'joint_states', lambda m: real.update(zip(m.name, m.position)), 20)
n.create_subscription(JointState, 'sim/joint_states', lambda m: sim.update(zip(m.name, m.position)), 20)
n.create_subscription(Odometry, 'odom', lambda m: od.__setitem__(0, m), 20)
n.create_subscription(Odometry, 'sim/ground_truth/odom', lambda m: gt.__setitem__(0, m), 20)
n.create_subscription(LaserScan, 'sim/scan', lambda m: c.__setitem__('scan', c['scan'] + 1), qos_profile_sensor_data)
n.create_subscription(Image, 'sim/camera/image_raw', lambda m: c.__setitem__('cam', c['cam'] + 1), qos_profile_sensor_data)
cmd = n.create_publisher(Twist, 'cmd_vel_teleop', 10)
arm = n.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
def spin(t):
    e = time.time() + t
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.01)
def drive(vx, vy, wz, t):
    m = Twist(); m.linear.x = float(vx); m.linear.y = float(vy); m.angular.z = float(wz); e = time.time() + t
    while time.time() < e: cmd.publish(m); spin(.05)
    cmd.publish(Twist()); spin(1.0)
def dist():
    a, b = od[0].pose.pose.position, gt[0].pose.pose.position; return math.hypot(a.x - b.x, a.y - b.y)
spin(4)
check('twin receives sim sensors', c['scan'] > 3 and c['cam'] > 3, str(c))
arm.publish(JointTrajectory(joint_names=A, points=[JointTrajectoryPoint(positions=[.6, -.5, .4, .3], time_from_start=Duration(sec=1))]))
spin(3.5)
err = max(abs(real[j] - sim[j]) for j in A)
check('arm: Gazebo copies the real joints', err < .02, f'real {[round(real[j],2) for j in A]} sim {[round(sim[j],2) for j in A]} err {err:.4f}')
drive(.2, 0, 0, 2.5); drive(0, .2, 0, 2.5)
p = od[0].pose.pose.position
check('base: real robot moved', math.hypot(p.x, p.y) > .4, f'odom {p.x:.2f},{p.y:.2f}')
check('base: Gazebo follows', dist() < .1, f'position error {dist()*1000:.0f} mm')
spin(1.5); print('status', st[0])
check('twin/status reports errors', st[0] and st[0]['arm_error_rad'] is not None and st[0]['base_error_m'] is not None)
if len(sys.argv) > 1 and sys.argv[1] == 'drift':
    for _ in range(4):
        drive(.25, 0, 0, 2.0); drive(-.25, 0, 0, 2.0)
    print('after a slipping drive', st[0], f'now {dist()*1000:.0f} mm'); spin(3)
    print('final', st[0])
    check('drift gets corrected', st[0]['resyncs'] >= 1 and dist() < .12, f"resyncs {st[0]['resyncs']}, error {dist()*1000:.0f} mm")
print('ALL PASS' if ok else 'SOME FAILED')
