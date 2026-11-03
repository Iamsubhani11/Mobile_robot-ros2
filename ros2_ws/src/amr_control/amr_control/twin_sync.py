#!/usr/bin/env python3
"""Digital twin: makes the Gazebo robot under /sim copy the real robot, measured state not commands.

 real /joint_states  -> /sim wheel velocities and arm/gripper positions
 real /odom vs /sim/ground_truth/odom -> drift, reported on /twin/status; optionally snapped back
"""
import json, math, subprocess, threading
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

WHEELS = ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']
ARM = ['arm_waist_joint', 'arm_shoulder_joint', 'arm_elbow_joint', 'arm_wrist_joint']
GRIP = ['gripper_joint']


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TwinSync(Node):
    def __init__(self):
        super().__init__('twin_sync')
        for p, d in (('sim_ns', 'sim'), ('resync_distance', 0.10), ('world', 'arena'), ('model', 'amr')):
            self.declare_parameter(p, d)
        ns = '/' + self.get_parameter('sim_ns').value
        self.resync = self.get_parameter('resync_distance').value
        self.world = self.get_parameter('world').value; self.model = self.get_parameter('model').value
        self.wheel_pub = self.create_publisher(Float64MultiArray, f'{ns}/wheel_controller/commands', 10)
        self.arm_pub = self.create_publisher(JointTrajectory, f'{ns}/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(JointTrajectory, f'{ns}/gripper_controller/joint_trajectory', 10)
        self.status_pub = self.create_publisher(String, 'twin/status', 10)
        self.real = {}; self.sim = {}; self.real_odom = None; self.sim_odom = None
        self.resyncs = 0; self.busy = False
        self.create_subscription(JointState, 'joint_states', self.on_real, 50)
        self.create_subscription(JointState, f'{ns}/joint_states', self.on_sim, 50)
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'real_odom', m), 20)
        self.create_subscription(Odometry, f'{ns}/ground_truth/odom', lambda m: setattr(self, 'sim_odom', m), 20)
        self.create_timer(1.0, self.report)

    def on_real(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        self.real = {n: msg.position[i] for n, i in idx.items()}
        if all(w in idx for w in WHEELS) and msg.velocity:
            self.wheel_pub.publish(Float64MultiArray(data=[msg.velocity[idx[w]] for w in WHEELS]))
        d = Duration(sec=0, nanosec=int(0.1e9))
        if all(j in idx for j in ARM):
            t = JointTrajectory(joint_names=ARM)
            t.points = [JointTrajectoryPoint(positions=[msg.position[idx[j]] for j in ARM], time_from_start=d)]
            self.arm_pub.publish(t)
        if all(j in idx for j in GRIP):
            t = JointTrajectory(joint_names=GRIP)
            t.points = [JointTrajectoryPoint(positions=[msg.position[idx[j]] for j in GRIP], time_from_start=d)]
            self.grip_pub.publish(t)

    def on_sim(self, msg):
        self.sim = dict(zip(msg.name, msg.position))

    def report(self):
        status = dict(arm_error_rad=None, base_error_m=None, yaw_error_rad=None, resyncs=self.resyncs)
        shared = [j for j in ARM + GRIP if j in self.real and j in self.sim]
        if shared:
            status['arm_error_rad'] = round(max(abs(self.real[j] - self.sim[j]) for j in shared), 4)
        if self.real_odom and self.sim_odom:
            a, b = self.real_odom.pose.pose, self.sim_odom.pose.pose
            dist = math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)
            status['base_error_m'] = round(dist, 4)
            status['yaw_error_rad'] = round(abs(math.atan2(math.sin(yaw(a.orientation) - yaw(b.orientation)), math.cos(yaw(a.orientation) - yaw(b.orientation)))), 4)
            if self.resync > 0 and dist > self.resync and not self.busy:
                self.snap(a)
        self.status_pub.publish(String(data=json.dumps(status)))

    def snap(self, pose):
        """Move the Gazebo model onto the real robot's odometry pose."""
        self.busy = True; self.resyncs += 1
        req = (f'name: "{self.model}", position: {{x: {pose.position.x}, y: {pose.position.y}, z: 0.002}}, '
               f'orientation: {{z: {pose.orientation.z}, w: {pose.orientation.w}}}')
        def run():
            try:
                subprocess.run(['gz', 'service', '-s', f'/world/{self.world}/set_pose', '--reqtype', 'gz.msgs.Pose',
                                '--reptype', 'gz.msgs.Boolean', '--timeout', '500', '--req', req], capture_output=True, timeout=3)
            except Exception as e:
                self.get_logger().warning(f'resync failed: {e}')
            self.busy = False
        threading.Thread(target=run, daemon=True).start()


def main():
    rclpy.init()
    node = TwinSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
