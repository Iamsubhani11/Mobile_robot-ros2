#!/usr/bin/env python3
"""Keyboard over a real pty, joystick by publishing /joy, against the running simulation: measures the effects."""
import math, os, pty, select, subprocess, sys, time, signal
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
ok = True
def check(name, cond, detail=''):
    global ok; ok &= bool(cond); print(('PASS ' if cond else 'FAIL ') + name, detail, flush=True)
rclpy.init(); n = Node('tt', parameter_overrides=[rclpy.Parameter('use_sim_time', value=True)])
gt = [None]; js = {}; mode = ['?']
n.create_subscription(Odometry, 'ground_truth/odom', lambda m: gt.__setitem__(0, m), 10)
n.create_subscription(JointState, 'joint_states', lambda m: js.update(zip(m.name, m.position)), 20)
n.create_subscription(String, 'mode', lambda m: mode.__setitem__(0, m.data), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
joy = n.create_publisher(Joy, 'joy', 10)
def spin(t):
    e = time.time() + t
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.01)
def xy():
    p = gt[0].pose.pose.position; return p.x, p.y
def yaw():
    q = gt[0].pose.pose.orientation; return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
spin(3)
# ------------- keyboard
pid, fd = pty.fork()
if pid == 0:
    os.execvp('ros2', ['ros2', 'run', 'amr_control', 'keyboard_teleop'])   # exactly how a user starts it
screen = ['']
def key(k, secs=1.0, rate=15):
    e = time.time() + secs
    while time.time() < e:
        os.write(fd, k.encode()); spin(1.0 / rate)
        if select.select([fd], [], [], 0)[0]:
            try: screen[0] += os.read(fd, 4096).decode(errors='ignore')
            except OSError: pass
spin(5)
a = xy(); key('i', 1.5); spin(1.2); b = xy(); check('keyboard i drives forward', b[0] - a[0] > .2, f'dx {b[0]-a[0]:.3f}')
c = xy(); spin(1.0); d = xy(); check('base stops after key release', math.dist(c, d) < .02, f'{math.dist(c,d)*1000:.1f} mm')
a = xy(); key(',', 1.5); spin(1.2); b = xy(); check('keyboard , drives backward', b[0] - a[0] < -.2, f'dx {b[0]-a[0]:.3f}')
a = xy(); key('\x1b[A', 1.5); spin(1.2); b = xy(); check('arrow up drives forward', b[0] - a[0] > .2, f'dx {b[0]-a[0]:.3f}')
a = xy(); key('\x1b[B', 1.5); spin(1.2); b = xy(); check('arrow down drives backward', b[0] - a[0] < -.2, f'dx {b[0]-a[0]:.3f}')
a = xy(); key('\x1b[D', 1.5); spin(1.2); b = xy(); check('arrow left strafes left', b[1] - a[1] > .2 and abs(b[0]-a[0]) < .03, f'dy {b[1]-a[1]:.3f}')
a = xy(); key('\x1b[C', 1.5); spin(1.2); b = xy(); check('arrow right strafes right', b[1] - a[1] < -.2 and abs(b[0]-a[0]) < .03, f'dy {b[1]-a[1]:.3f}')
a = xy(); key('L', 1.5); spin(1.2); b = xy(); check('keyboard L strafes right', b[1] - a[1] < -.2 and abs(b[0]-a[0]) < .03, f'dy {b[1]-a[1]:.3f}')
check('screen says which way it is driving', 'Driving: backward' in screen[0] and 'Driving: strafe right' in screen[0])
a = xy(); key('J', 1.5); spin(1.2); b = xy(); check('keyboard J strafes left', b[1] - a[1] > .2 and abs(b[0]-a[0]) < .03, f'dy {b[1]-a[1]:.3f}')
a = yaw(); key('j', 1.0); spin(1.0); check('keyboard j turns left', yaw() - a > .3, f'{yaw()-a:.2f} rad')
for k, j, sign in (('a', 'arm_waist_joint', 1), ('s', 'arm_shoulder_joint', -1), ('d', 'arm_elbow_joint', -1), ('f', 'arm_wrist_joint', -1)):
    s0 = js[j]; key(k, 1.0); spin(.5); check(f'keyboard {k} moves {j} {"+" if sign>0 else "-"}', (js[j] - s0) * sign > .3, f'{js[j]-s0:+.2f} rad')
g0 = js['gripper_joint']; key('b', 1.0); spin(.5); check('keyboard b closes the gripper', js['gripper_joint'] > g0 + .3, f'{js["gripper_joint"]:.2f}')
key('g', 1.5); spin(.5); check('keyboard g opens the gripper', js['gripper_joint'] < g0 + .2, f'{js["gripper_joint"]:.2f}')
key('h', 0.1); spin(5); check('keyboard h returns home', max(abs(js[j]) for j in ('arm_waist_joint','arm_shoulder_joint','arm_elbow_joint','arm_wrist_joint')) < .03)
key('n', 0.1); spin(5); check('keyboard n stows', abs(js['arm_shoulder_joint'] + .95) < .03, f'{js["arm_shoulder_joint"]:.2f}')
key('0', 0.1); spin(.5); check('keyboard 0 selects STOP', mode[0] == 'STOP', mode[0])
a = xy(); key('i', 1.0); spin(.5); check('STOP blocks driving', math.dist(a, xy()) < .01, f'{math.dist(a,xy())*1000:.1f} mm')
s0 = js['arm_elbow_joint']; key('c', .8); spin(.3); check('STOP blocks the arm', abs(js['arm_elbow_joint'] - s0) < .01)
key('c', .1); spin(.3); key('c', .1); check('screen says why keys are ignored in STOP', 'Press 1 for MANUAL' in screen[0])
key('1', 0.1); spin(.5); check('keyboard 1 selects MANUAL', mode[0] == 'MANUAL', mode[0])
key('k', .1); check('screen shows the mode', 'Mode: MANUAL' in screen[0])
a = xy(); key('i', 1.0); spin(1.); check('driving works again after STOP', math.dist(a, xy()) > .1)
os.write(fd, b'\x03'); spin(1.0)
try: os.waitpid(pid, os.WNOHANG); os.kill(pid, signal.SIGTERM)
except OSError: pass
# ------------- joystick
jp = subprocess.Popen(['ros2', 'run', 'amr_control', 'joy_teleop', '--ros-args', '-p', 'use_sim_time:=true'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
def pad(secs, axes=None, buttons=None):
    ax = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]; bt = [0] * 11
    for k, v in (axes or {}).items(): ax[k] = float(v)
    for k in (buttons or []): bt[k] = 1
    e = time.time() + secs
    while time.time() < e:
        joy.publish(Joy(axes=ax, buttons=bt)); spin(.05)
spin(4); pad(1)
a = xy(); pad(1.5, {1: 1.0}); spin(.2); check('joystick stick ignored without LB deadman', math.dist(a, xy()) < .01, f'{math.dist(a,xy())*1000:.1f} mm')
a = xy(); pad(1.5, {1: 1.0}, [4]); pad(.8); b = xy(); check('joystick LB + left stick drives', math.dist(a, b) > .15, f'{math.dist(a,b):.3f} m')
c = xy(); pad(1.0); check('joystick stops on LB release', math.dist(c, xy()) < .02)
a = xy(); pad(1.5, {0: 1.0}, [4]); pad(.8); b = xy(); check('joystick left stick X strafes', math.dist(a, b) > .15, f'{math.dist(a,b):.3f} m')
a = yaw(); pad(1.5, {3: 1.0}, [4]); pad(.8); check('joystick right stick turns', abs(yaw() - a) > .4, f'{yaw()-a:.2f} rad')
s0 = js['arm_waist_joint']; pad(1.0, {0: 1.0}, [5]); pad(.3); check('joystick RB + stick moves the arm, not the base', abs(js['arm_waist_joint'] - s0) > .3, f'{js["arm_waist_joint"]-s0:+.2f}')
g0 = js['gripper_joint']; pad(1.0, {5: -1.0}); pad(.3); check('joystick RT closes the gripper', js['gripper_joint'] > g0 + .3, f'{js["gripper_joint"]:.2f}')
pad(1.0, {2: -1.0}); pad(.3); check('joystick LT opens the gripper', js['gripper_joint'] < g0 + .4, f'{js["gripper_joint"]:.2f}')
pad(.2, {7: 1.0}); pad(5); check('joystick D-pad up goes home', abs(js['arm_shoulder_joint']) < .03)
pad(.2, buttons=[2]); pad(.3); check('joystick X selects STOP', mode[0] == 'STOP', mode[0])
pad(.2, buttons=[1]); pad(.3); check('joystick B selects AUTO', mode[0] == 'AUTO', mode[0])
a = xy(); pad(1.0, {1: 1.0}, [4]); check('AUTO ignores the stick', math.dist(a, xy()) < .01)
pad(.2, buttons=[0]); pad(.3); check('joystick A selects MANUAL', mode[0] == 'MANUAL', mode[0])
os.killpg(jp.pid, signal.SIGTERM)
print('ALL PASS' if ok else 'SOME FAILED')
