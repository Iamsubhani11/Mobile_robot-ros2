#!/usr/bin/env python3
"""Keyboard control for the base and arm. Base keys follow RigBetel's rbl_teleop_keyboard layout."""
import os, select, sys, termios, time, tty
import rclpy
from amr_control.teleop_core import TeleopCore

HELP = r"""
 AMR keyboard teleop
 ---------------------------------------------------------------
 ARROWS   up / down = forward / backward    left / right = strafe
 BASE     u  i  o         hold shift to strafe:   U  I  O
          j  k  l                                 J  K  L
          m  ,  .         k or SPACE = stop       M  <  >

 ARM      a / z  waist       left / right
          s / x  shoulder    back / forward
          d / c  elbow       up / down
          f / v  wrist       up / down
          g / b  gripper     open / close

 POSES    h = home (CAD pose)   n = stow for driving   r = reach
 SPEED    - / =  slower / faster

 MODES    1 = MANUAL   2 = AUTO   0 = STOP (latched, press 1 or 2 to leave)
 AUTO     p = find and pick the object in front     t = place it in front
          RViz "2D Goal Pose" in AUTO: go there, then pick, or place if carrying
 CTRL-C quit
 ---------------------------------------------------------------
"""
# key: (vx, vy, wz) as fractions of top speed
BASE = {'i': (1, 0, 0), ',': (-1, 0, 0), 'j': (0, 0, 1), 'l': (0, 0, -1), 'u': (1, 0, 1), 'o': (1, 0, -1), 'm': (-1, 0, -1), '.': (-1, 0, 1),
        'I': (1, 0, 0), '<': (-1, 0, 0), 'J': (0, 1, 0), 'L': (0, -1, 0), 'U': (1, 1, 0), 'O': (1, -1, 0), 'M': (-1, 1, 0), '>': (-1, -1, 0)}
ARM = {'a': ('arm_waist_joint', 1), 'z': ('arm_waist_joint', -1), 's': ('arm_shoulder_joint', -1), 'x': ('arm_shoulder_joint', 1),
       'd': ('arm_elbow_joint', -1), 'c': ('arm_elbow_joint', 1), 'f': ('arm_wrist_joint', -1), 'v': ('arm_wrist_joint', 1)}
POSE = {'h': 'home', 'n': 'stow', 'r': 'reach'}
ARROWS = {'A': 'i', 'B': ',', 'C': 'L', 'D': 'J'}   # the last byte of the terminal's arrow key codes
DIRECTION = {(1, 0, 0): 'forward', (-1, 0, 0): 'backward', (0, 1, 0): 'strafe left', (0, -1, 0): 'strafe right',
             (0, 0, 1): 'turn left', (0, 0, -1): 'turn right', (1, 1, 0): 'diagonal front left', (1, -1, 0): 'diagonal front right',
             (-1, 1, 0): 'diagonal back left', (-1, -1, 0): 'diagonal back right', (1, 0, 1): 'forward and left',
             (1, 0, -1): 'forward and right', (-1, 0, 1): 'backward and left', (-1, 0, -1): 'backward and right'}
HOLD = 0.7      # a key counts as held this long, enough to bridge the terminal's auto-repeat delay
TICK = 0.05


class KeyboardTeleop(TeleopCore):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.scale = 1.0; self.base = (0, 0, 0); self.base_t = 0.0
        self.arm_key = None; self.arm_t = 0.0; self.grip_dir = 0; self.grip_t = 0.0
        self.shown_mode = None; self.hint_t = 0.0; self.shown_base = (0, 0, 0)

    def say(self, text):
        sys.stdout.write(f'\r\n >> {text}\r\n'); sys.stdout.flush()

    def on_mode(self, msg):
        super().on_mode(msg)
        if msg.data != self.shown_mode:
            self.shown_mode = msg.data
            self.say(f'Mode: {msg.data}' + ('' if msg.data == 'MANUAL' else '. Press 1 for MANUAL to drive.'))

    def on_key(self, k, now):
        if self.mode != 'MANUAL' and (k in BASE or k in ARM or k in POSE or k in ('g', 'b')) and now - self.hint_t > 2.0:
            self.hint_t = now
            self.say(f'Key ignored, the robot is in {self.mode} mode. Press 1 for MANUAL.')
        if k in BASE:
            self.base, self.base_t = BASE[k], now
        elif k in ('k', ' ', 'K'):
            self.base = (0, 0, 0); self.base_t = 0.0; self.arm_key = None; self.grip_dir = 0; self.goal = None
        elif k in ARM:
            self.arm_key, self.arm_t = ARM[k], now
        elif k in 'gb':
            self.grip_dir, self.grip_t = (-1 if k == 'g' else 1), now
        elif k in POSE:
            self.go_to(POSE[k])
        elif k == '1':
            self.set_mode('MANUAL')
        elif k == '2':
            self.set_mode('AUTO')
        elif k == '0':
            self.set_mode('STOP')
        elif k == 'p':
            self.set_mode('AUTO'); self.send_mission('pick')
        elif k == 't':
            self.set_mode('AUTO'); self.send_mission('place')
        elif k == '-':
            self.scale = max(0.1, self.scale - 0.1)
        elif k == '=':
            self.scale = min(1.0, self.scale + 0.1)

    def update(self, now):
        b = self.base if now - self.base_t < HOLD else (0, 0, 0)
        if self.mode != 'MANUAL':
            return
        if b != self.shown_base:
            self.shown_base = b
            self.say('Driving: ' + DIRECTION.get(b, 'stopped'))
        self.drive(b[0] * self.linear * self.scale, b[1] * self.linear * self.scale, b[2] * self.angular * self.scale)
        rates = {}
        if self.arm_key and now - self.arm_t < 0.25:
            rates[self.arm_key[0]] = self.arm_key[1] * self.arm_speed * self.scale
        grip = self.grip_dir * self.grip_speed if now - self.grip_t < 0.25 else 0.0
        self.nudge(rates, grip, TICK)
        self.step_goal(TICK)
        self.publish_arm()


def main():
    if not sys.stdin.isatty():
        sys.exit('keyboard_teleop needs a real terminal: run it directly with ros2 run, not from a launch file.')
    rclpy.init()
    node = KeyboardTeleop()
    saved = termios.tcgetattr(sys.stdin)
    print(HELP.replace('\n', '\r\n'))
    try:
        tty.setraw(sys.stdin.fileno())
        last = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            ready, _, _ = select.select([sys.stdin], [], [], 0.01)
            now = time.monotonic()
            if ready:
                k = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
                if k == '\x03':
                    break
                if k == '\x1b' and select.select([sys.stdin], [], [], 0.01)[0]:   # arrow keys send ESC [ A
                    seq = os.read(sys.stdin.fileno(), 2).decode(errors='ignore')
                    k = ARROWS.get(seq[-1:], '')
                node.on_key(k, now)
            if now - last >= TICK:
                last = now
                node.update(now)
    finally:
        node.drive(0, 0, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved)
        node.destroy_node()
        rclpy.try_shutdown()
