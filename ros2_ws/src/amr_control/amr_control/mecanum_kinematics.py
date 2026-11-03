#!/usr/bin/env python3
"""/cmd_vel -> wheel speeds, wheel feedback -> /odom + odom->base_link TF.

The same node runs in Gazebo and on the real robot; only what is behind
wheel_controller/commands and joint_states differs.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

WHEELS = ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']


def inverse(vx, vy, wz, r, k):
    """Body twist to wheel rad/s, order FL FR RL RR. k = half_wheelbase + half_track."""
    return [(vx - vy - k * wz) / r, (vx + vy + k * wz) / r, (vx + vy - k * wz) / r, (vx - vy + k * wz) / r]


def forward(w, r, k):
    fl, fr, rl, rr = w
    return ((fl + fr + rl + rr) * r / 4, (-fl + fr + rl - rr) * r / 4, (-fl + fr - rl + rr) * r / (4 * k))


class MecanumKinematics(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics')
        for name, default in (('wheel_radius', 0.02442), ('half_wheelbase', 0.0825), ('half_track', 0.1319),
                              ('max_wheel_speed', 25.0), ('accel_limit', 1.0), ('cmd_timeout', 0.5), ('publish_tf', True)):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        self.r = g('wheel_radius'); self.k = g('half_wheelbase') + g('half_track')
        self.max_w = g('max_wheel_speed'); self.acc = g('accel_limit'); self.timeout = g('cmd_timeout')
        self.publish_tf = g('publish_tf')
        self.target = [0.0, 0.0, 0.0]; self.current = [0.0, 0.0, 0.0]
        self.last_cmd = None; self.last_tick = None
        self.x = self.y = self.th = 0.0; self.last_js = None
        self.wheel_pub = self.create_publisher(Float64MultiArray, 'wheel_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 50)
        self.create_timer(0.02, self.tick)

    def on_cmd(self, msg):
        self.target = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.last_cmd = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        if self.last_tick is None:
            self.last_tick = now
            return
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        if dt <= 0:
            return
        if self.last_cmd is None or (now - self.last_cmd).nanoseconds * 1e-9 > self.timeout:
            self.target = [0.0, 0.0, 0.0]
        # ramp, with yaw scaled to a comparable tip speed
        for i, limit in enumerate((self.acc, self.acc, self.acc / max(self.k, 1e-3))):
            step = limit * dt
            self.current[i] += max(-step, min(step, self.target[i] - self.current[i]))
        w = inverse(*self.current, self.r, self.k)
        peak = max(abs(v) for v in w)
        if peak > self.max_w:      # scale everything so the direction of travel is preserved
            w = [v * self.max_w / peak for v in w]
        self.wheel_pub.publish(Float64MultiArray(data=w))

    def on_joint_states(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in WHEELS) or not msg.velocity:
            return
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.last_js is None:
            self.last_js = stamp
            return
        dt = (stamp - self.last_js).nanoseconds * 1e-9
        self.last_js = stamp
        if dt <= 0 or dt > 1.0:
            return
        vx, vy, wz = forward([msg.velocity[idx[n]] for n in WHEELS], self.r, self.k)
        c, s = math.cos(self.th + wz * dt / 2), math.sin(self.th + wz * dt / 2)
        self.x += (c * vx - s * vy) * dt; self.y += (s * vx + c * vy) * dt; self.th += wz * dt
        qz, qw = math.sin(self.th / 2), math.cos(self.th / 2)
        odom = Odometry()
        odom.header.stamp = msg.header.stamp; odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x; odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz; odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx; odom.twist.twist.linear.y = vy; odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)
        if self.publish_tf:
            t = TransformStamped()
            t.header = odom.header; t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x; t.transform.translation.y = self.y
            t.transform.rotation.z = qz; t.transform.rotation.w = qw
            self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = MecanumKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
