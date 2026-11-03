#!/usr/bin/env python3
"""The RViz flow: AUTO + 2D Goal Pose = fetch when empty, deliver when carrying. Also: goal is ignored outside AUTO, STOP aborts."""
import json, math, re, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
rclpy.init(); n = Node('gp'); st = []
n.create_subscription(String, 'mission/status', lambda m: (st.append(json.loads(m.data)), print(time.strftime('%X'), st[-1]['status'], flush=True)), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
g = n.create_publisher(PoseStamped, 'goal_pose', 10); mode = n.create_publisher(String, 'mode_cmd', 10)
def spin(t):
    e = time.time() + t
    while time.time() < e: rclpy.spin_once(n, timeout_sec=.05)
def goal(x, y, yaw):
    p = PoseStamped(); p.header.frame_id = 'map'; p.pose.position.x = float(x); p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2); p.pose.orientation.w = math.cos(yaw / 2); g.publish(p)
def wait(secs):
    e = time.time() + secs; st.clear()
    while time.time() < e:
        spin(.2)
        if any(s['status'].startswith(('DONE', 'FAILED', 'ABORTED')) for s in st): return st[-1]['status']
    return 'TIMEOUT'
def block(c):
    o = subprocess.run(['gz', 'model', '-m', f'block_{c}', '-p'], capture_output=True, text=True, timeout=8).stdout
    return re.findall(r'\[(-?[\d.e-]+) (-?[\d.e-]+) (-?[\d.e-]+)\]', o)[0]
spin(2)
mode.publish(String(data='MANUAL')); spin(.5); st.clear(); goal(1.8, -.28, 0); spin(2); print('MANUAL, goal ignored:', not any(s['status'].startswith('START') for s in st))
mode.publish(String(data='AUTO')); spin(.5)
goal(1.8, -.28, 0); r = wait(100); print('first goal, empty-handed ->', r, 'blue block', block('blue'))
if r.startswith('DONE'):
    goal(-1.3, -.3, 3.14159); r = wait(70); print('second goal, carrying ->', r, 'blue block', block('blue'))
