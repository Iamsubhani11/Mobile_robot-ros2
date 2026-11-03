#!/usr/bin/env python3
"""Runs the real firmware against the real ROS bridge over a pty. Needs: ros2 sourced, sil built."""
import os, pty, subprocess, sys, time, tty, json
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
HERE = os.path.dirname(os.path.abspath(__file__)); binp, logp = sys.argv[1], sys.argv[2]
m, s = pty.openpty(); tty.setraw(m); tty.setraw(s)
# the firmware holds one end of the pty, the bridge opens the other as a normal serial port
fw = subprocess.Popen([binp, logp], stdin=m, stdout=m, stderr=subprocess.DEVNULL)
br = subprocess.Popen(['ros2', 'run', 'amr_control', 'hardware_bridge', '--ros-args', '-p', 'backend:=serial', '-p', f'port:={os.ttyname(s)}'],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
import atexit, signal
def cleanup():
    for proc, group in ((fw, False), (br, True)):
        try:
            os.killpg(proc.pid, signal.SIGTERM) if group else proc.terminate()
        except ProcessLookupError:
            pass
atexit.register(cleanup)
rclpy.init(); n = Node('sil_test'); js = {}; info = []; count = [0]; arr = {}
def on_js(msg):
    js.update(zip(msg.name, msg.position)); arr.update(zip(msg.name, msg.velocity)); count[0] += 1
n.create_subscription(JointState, 'joint_states', on_js, 20)
n.create_subscription(String, 'hardware/status', lambda m: info.append(m.data), 10)
w = n.create_publisher(Float64MultiArray, 'wheel_controller/commands', 10)
a = n.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
g = n.create_publisher(JointTrajectory, 'gripper_controller/joint_trajectory', 10)
def spin(t):
    e = time.time() + t
    while time.time() < e: rclpy.spin_once(n, timeout_sec=0.01)
def wheels(v, t):
    e = time.time() + t
    while time.time() < e:
        w.publish(Float64MultiArray(data=v)); spin(0.05)
ok = True
def check(name, cond, detail=''):
    global ok; ok &= bool(cond); print(('PASS ' if cond else 'FAIL ') + name, detail, flush=True)
spin(6.0)
check('state stream reaches ROS', count[0] > 20, f'{count[0]} joint_states in 6 s')
wheels([10.0, -10.0, 5.0, 0.0], 1.0)
check('wheel commands echoed', abs(arr['front_left_wheel_joint'] - 10) < .01 and abs(arr['front_right_wheel_joint'] + 10) < .01, str([round(arr[k], 2) for k in arr if 'wheel' in k]))
target = [0.5, -0.4, 0.6, -0.3]
a.publish(JointTrajectory(joint_names=['arm_waist_joint', 'arm_shoulder_joint', 'arm_elbow_joint', 'arm_wrist_joint'],
                          points=[JointTrajectoryPoint(positions=target, time_from_start=Duration(nanosec=100000000))]))
g.publish(JointTrajectory(joint_names=['gripper_joint'], points=[JointTrajectoryPoint(positions=[0.5], time_from_start=Duration(nanosec=100000000))]))
wheels([10.0, -10.0, 5.0, 0.0], 0.15)
mid = js['arm_waist_joint']
check('servo speed is limited, no jump', mid < 0.45, f'waist {mid:.3f} shortly after a 0.5 rad command')
wheels([10.0, -10.0, 5.0, 0.0], 1.5)
errs = [abs(js[j] - t) for j, t in zip(['arm_waist_joint', 'arm_shoulder_joint', 'arm_elbow_joint', 'arm_wrist_joint'], target)]
check('arm reaches the target', max(errs) < 0.01, f'max error {max(errs):.4f} rad')
check('gripper reaches the target', abs(js['gripper_joint'] - .5) < .01)
a.publish(JointTrajectory(joint_names=['arm_waist_joint', 'arm_shoulder_joint', 'arm_elbow_joint', 'arm_wrist_joint'],
                          points=[JointTrajectoryPoint(positions=[3.0, 3.0, 3.0, 3.0], time_from_start=Duration(nanosec=100000000))]))
wheels([0.0, 0.0, 0.0, 0.0], 3.0)
check('firmware clamps to joint limits', abs(js['arm_shoulder_joint'] - .59) < .01 and abs(js['arm_waist_joint'] - 1.57) < .01, f"shoulder {js['arm_shoulder_joint']:.3f}, waist {js['arm_waist_joint']:.3f}")
wheels([12.0, 12.0, 12.0, 12.0], 0.5)
check('wheels moving before bridge dies', abs(arr['front_left_wheel_joint'] - 12) < .01)
os.killpg(br.pid, signal.SIGTERM); time.sleep(1.0)
# a corrupt line must be ignored, then the watchdog stops the motors with no wheel commands arriving
os.write(s, b'$V,9000,9000,9000,9000*00\n')
time.sleep(1.2)
fw.terminate(); fw.wait(timeout=3)
log = open(logp).read().splitlines()
motors = [l.split() for l in log if ' MOTOR ' in l]
check('motor PWM ran', any(int(l[3]) != 0 for l in motors), f'{len(motors)} changes')
check('correct sign per wheel', any(l[2] == '1' and int(l[3]) < 0 for l in motors) and any(l[2] == '0' and int(l[3]) > 0 for l in motors))
last = {l[2]: int(l[3]) for l in motors}
check('watchdog stopped all motors', all(v == 0 for v in last.values()), str(last))
check('corrupt command ignored', not any(abs(int(l[3])) > 150 for l in motors[-6:]))
servos = [l.split() for l in log if ' SERVO ' in l]
us = {l[2]: int(l[3]) for l in servos}
check('servo pulse within range', all(500 <= v <= 2500 for v in us.values()), str(us))
print('ALL PASS' if ok else 'SOME FAILED'); sys.exit(0 if ok else 1)
