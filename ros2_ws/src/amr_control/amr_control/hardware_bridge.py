#!/usr/bin/env python3
"""Connects ROS to the real robot. Topics match Gazebo's exactly, so teleop and kinematics don't change.

  backend:=serial   Arduino Uno, UNO Q, ESP32 running firmware/amr_firmware, port:=/dev/ttyUSB0
  backend:=pi_gpio  Raspberry Pi driving motor drivers directly plus a PCA9685 for servos
  backend:=mock     no hardware: a simple model of the robot, for testing the twin
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory
from amr_control.backends import make_backend

WHEELS = ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']
ACTUATED = ['arm_waist_joint', 'arm_shoulder_joint', 'arm_elbow_joint', 'arm_wrist_joint', 'gripper_joint']


class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')
        for p, d in (('backend', 'mock'), ('port', '/dev/ttyUSB0'), ('baud', 115200), ('rate', 30.0), ('max_wheel_speed', 25.0),
                     ('i2c_bus', 1), ('pwm_frequency', 1000), ('mock_lag', 0.05), ('mock_wheel_gain', 1.0)):
            self.declare_parameter(p, d)
        params = {p: self.get_parameter(p).value for p in ('port', 'baud', 'max_wheel_speed', 'i2c_bus', 'pwm_frequency', 'mock_lag', 'mock_wheel_gain')}
        self.backend = make_backend(self.get_parameter('backend').value, params, self.get_logger())
        self.wheel_cmd = [0.0] * 4
        self.joint_cmd = None
        self.js_pub = self.create_publisher(JointState, 'joint_states', 20)
        self.info_pub = self.create_publisher(String, 'hardware/status', 10)
        self.create_subscription(Float64MultiArray, 'wheel_controller/commands', self.on_wheels, 10)
        self.create_subscription(JointTrajectory, 'arm_controller/joint_trajectory', self.on_traj, 10)
        self.create_subscription(JointTrajectory, 'gripper_controller/joint_trajectory', self.on_traj, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

    def on_wheels(self, msg):
        if len(msg.data) == 4:
            self.wheel_cmd = list(msg.data)

    def on_traj(self, msg):
        if not msg.points:
            return
        if self.joint_cmd is None:
            self.joint_cmd = {}
        self.joint_cmd.update(dict(zip(msg.joint_names, msg.points[-1].positions)))

    def tick(self):
        dt = 1.0 / self.get_parameter('rate').value
        joints = None
        if self.joint_cmd is not None:
            joints = [self.joint_cmd.get(j, math.nan) for j in ACTUATED]
        self.backend.write(self.wheel_cmd, joints)
        state = self.backend.read(dt)
        if state is None:
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = WHEELS + ACTUATED
        js.position = list(state['wheel_pos']) + list(state['joints'])
        js.velocity = list(state['wheel_vel']) + [0.0] * len(ACTUATED)
        self.js_pub.publish(js)
        if state.get('status'):
            self.info_pub.publish(String(data=state['status']))

    def destroy_node(self):
        self.backend.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
