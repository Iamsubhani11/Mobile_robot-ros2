#!/usr/bin/env python3
"""Joystick. Numbers are in config/robot.yaml: check yours with `ros2 topic echo /joy`.

 A MANUAL      B AUTO      X STOP (latched)      Y find and pick the object in front
 D-pad: up home, down stow, left reach, right place the object in front
 MANUAL, hold LB (deadman): left stick drives and strafes, right stick X turns.
 MANUAL, hold RB: left X waist, left Y shoulder, right Y elbow, right X wrist.
 LT / RT: open / close the gripper.
"""
import rclpy
from sensor_msgs.msg import Joy
from amr_control.teleop_core import TeleopCore

TICK = 0.05


class JoyTeleop(TeleopCore):
    def __init__(self):
        super().__init__('joy_teleop')
        for p, d in (('axis_lx', 0), ('axis_ly', 1), ('axis_lt', 2), ('axis_rx', 3), ('axis_ry', 4), ('axis_rt', 5),
                     ('axis_dx', 6), ('axis_dy', 7), ('btn_a', 0), ('btn_b', 1), ('btn_x', 2), ('btn_y', 3), ('btn_lb', 4), ('btn_rb', 5), ('joy_timeout', 0.5)):
            self.declare_parameter(p, d)
        self.g = lambda n: self.get_parameter(n).value
        self.joy = None; self.prev = []; self.prev_dpad = (0.0, 0.0); self.joy_t = None; self.trig_seen = {}
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_timer(TICK, self.update)

    def pressed(self, msg, param):
        i = self.g(param)
        return len(msg.buttons) > i and msg.buttons[i] and not (len(self.prev) > i and self.prev[i])

    def on_joy(self, msg):
        if self.pressed(msg, 'btn_a'):
            self.set_mode('MANUAL')
        if self.pressed(msg, 'btn_b'):
            self.set_mode('AUTO')
        if self.pressed(msg, 'btn_x'):
            self.set_mode('STOP')
        if self.pressed(msg, 'btn_y'):
            self.set_mode('AUTO'); self.send_mission('pick')
        dx_i, dy_i = self.g('axis_dx'), self.g('axis_dy')
        dpad = (msg.axes[dx_i] if dx_i < len(msg.axes) else 0.0, msg.axes[dy_i] if dy_i < len(msg.axes) else 0.0)
        if dpad != self.prev_dpad:
            if dpad[1] > 0.5:
                self.go_to('home')
            elif dpad[1] < -0.5:
                self.go_to('stow')
            elif dpad[0] > 0.5:     # D-pad left reads +1 on Xbox pads
                self.go_to('reach')
            elif dpad[0] < -0.5:
                self.set_mode('AUTO'); self.send_mission('place')
        self.prev_dpad = dpad
        self.prev = list(msg.buttons)
        self.joy = msg; self.joy_t = self.get_clock().now()

    def axis(self, name):
        i = self.g(name)
        return self.joy.axes[i] if i < len(self.joy.axes) else 0.0

    def button(self, name):
        i = self.g(name)
        return i < len(self.joy.buttons) and bool(self.joy.buttons[i])

    def trigger(self, name):
        """Triggers rest at 1 and go to -1, but read 0 until first pressed."""
        v = self.axis(name)
        if v != 0.0:
            self.trig_seen[name] = True
        return (1.0 - v) / 2.0 if self.trig_seen.get(name) else 0.0

    def update(self):
        stale = self.joy is None or (self.get_clock().now() - self.joy_t).nanoseconds * 1e-9 > self.g('joy_timeout')
        if stale:
            self.drive(0, 0, 0)
            return
        if self.mode != 'MANUAL':
            return
        if self.button('btn_lb'):
            self.drive(self.axis('axis_ly') * self.linear, self.axis('axis_lx') * self.linear, self.axis('axis_rx') * self.angular)
        else:
            self.drive(0, 0, 0)
        rates = {}
        if self.button('btn_rb'):
            s = self.arm_speed
            rates = {'arm_waist_joint': self.axis('axis_lx') * s, 'arm_shoulder_joint': self.axis('axis_ly') * s,
                     'arm_elbow_joint': -self.axis('axis_ry') * s, 'arm_wrist_joint': -self.axis('axis_rx') * s}
        grip = (self.trigger('axis_rt') - self.trigger('axis_lt')) * self.grip_speed
        self.nudge(rates, grip, TICK)
        self.step_goal(TICK)
        self.publish_arm()


def main():
    rclpy.init()
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
