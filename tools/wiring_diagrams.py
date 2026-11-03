#!/usr/bin/env python3
"""Draws the wiring diagrams and the system diagram into images/."""
import os, sys
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
out = sys.argv[1]
INK = '#1f2630'; POWER = '#c0392b'; GND = '#222222'; SIG = '#1f6fb2'; I2C = '#2e8b57'; ENC = '#8e5bb5'; USB = '#c77d0a'
def box(ax, x, y, w, h, title, lines=(), color='#eef1f5'):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.01,rounding_size=0.12', fc=color, ec=INK, lw=1.4))
    ax.text(x + w / 2, y + h - 0.2, title, ha='center', va='top', fontsize=10.5, fontweight='bold', color=INK)
    for i, l in enumerate(lines):
        ax.text(x + 0.1, y + h - 0.55 - i * 0.27, l, ha='left', va='top', fontsize=7.6, family='monospace', color=INK)
def wire(ax, a, b, c, label=None, lw=1.8, at=0.5, dy=0.08):
    ax.plot([a[0], b[0]], [a[1], b[1]], color=c, lw=lw, solid_capstyle='round', zorder=0)
    if label:
        ax.text(a[0] + (b[0] - a[0]) * at, a[1] + (b[1] - a[1]) * at + dy, label, color=c, fontsize=7.6, ha='center', fontweight='bold',
                bbox=dict(fc='white', ec='none', pad=0.6, alpha=0.9))
def diagram(fn, title, board, board_lines, host_text, host_link, logic, note, enc_text, jumpers='jumpers off ENA/ENB'):
    fig, ax = plt.subplots(figsize=(14.5, 9)); ax.set_xlim(0, 16); ax.set_ylim(0, 10); ax.axis('off')
    ax.text(0.2, 9.75, title, fontsize=15, fontweight='bold', color=INK)
    box(ax, 0.3, 6.6, 3.4, 2.5, 'ROS computer', host_text, '#e3edf7')
    box(ax, 5.2, 3.0, 3.8, 5.9, board, board_lines, '#dfeee5')
    box(ax, 10.8, 6.6, 2.5, 2.3, 'L298N #1', ['Motor A: front left', 'Motor B: front right', jumpers], '#f5ecd9')
    box(ax, 10.8, 3.7, 2.5, 2.3, 'L298N #2', ['Motor A: rear left', 'Motor B: rear right', jumpers], '#f5ecd9')
    box(ax, 13.9, 3.7, 1.8, 5.2, '4 Johnson\nmotors', ['', '', 'FL', 'FR', 'RL', 'RR', '', 'swap the two', 'wires, or set', 'MOTOR_SIGN,', 'if one runs', 'backwards'], '#f4e0d6')
    box(ax, 5.2, 0.4, 3.8, 2.0, 'PCA9685 servo driver', ['CH0 waist    MG996R', 'CH1 shoulder 25-35 kg.cm', 'CH2 elbow    25 kg.cm', 'CH3 wrist    MG996R', 'CH4 gripper  MG996R'], '#ece3f4')
    box(ax, 0.3, 0.4, 3.6, 2.6, 'Battery: 3S LiPo 11.1 V', ['Main fuse 15 A + switch', '', '-> L298N +12 V  (direct)', '-> 6 V 10 A UBEC -> PCA V+', '-> 5 V 5 A UBEC -> Pi', '', 'ALL grounds joined'], '#f7dede')
    box(ax, 0.3, 4.0, 3.4, 1.9, 'USB on the ROS computer', ['YDLIDAR X2 (via its board)', 'Camera (or Pi CSI ribbon)', 'Joystick dongle'], '#fbf3dc')
    box(ax, 10.3, 0.4, 5.4, 2.3, 'Logic level', logic, '#f0f0f0')
    wire(ax, (3.7, 8.0), (5.2, 8.0), USB, host_link, lw=2.6)
    wire(ax, (1.9, 6.6), (1.9, 5.9), USB)
    wire(ax, (9.0, 8.1), (10.8, 8.1), SIG, 'FL: EN IN1 IN2')
    wire(ax, (9.0, 7.2), (10.8, 7.2), SIG, 'FR: EN IN3 IN4')
    wire(ax, (9.0, 5.2), (10.8, 5.2), SIG, 'RL: EN IN1 IN2')
    wire(ax, (9.0, 4.3), (10.8, 4.3), SIG, 'RR: EN IN3 IN4')
    wire(ax, (13.3, 7.7), (13.9, 7.7), POWER); wire(ax, (13.3, 4.9), (13.9, 4.9), POWER)
    wire(ax, (7.1, 3.0), (7.1, 2.4), I2C, 'SDA, SCL, VCC, GND', lw=2.2, dy=-0.07)
    wire(ax, (3.9, 1.0), (5.2, 1.0), POWER, '6 V to V+', lw=2.6)
    wire(ax, (3.9, 2.2), (4.7, 2.2), POWER, lw=2.6); wire(ax, (4.7, 2.2), (4.7, 9.3), POWER, lw=2.6); wire(ax, (4.7, 9.3), (12.0, 9.3), POWER, '11.1 V to both L298N +12 V', lw=2.6); wire(ax, (12.0, 9.3), (12.0, 8.9), POWER, lw=2.6)
    wire(ax, (9.6, 9.3), (9.6, 6.25), POWER, lw=2.6); wire(ax, (9.6, 6.25), (12.0, 6.25), POWER, lw=2.6); wire(ax, (12.0, 6.25), (12.0, 6.0), POWER, lw=2.6)
    ax.text(9.2, 3.25, enc_text, fontsize=8, color=ENC, fontweight='bold')
    ax.text(0.3, 3.75, note, fontsize=8.0, color=INK, va='top')
    fig.savefig(os.path.join(out, fn), dpi=130, bbox_inches='tight', facecolor='white'); plt.close(fig); print('wrote', fn)
RULE = ['Each L298N: 5 V pin not used to power the board.', 'Join: battery -, UBEC -, L298N GND, board GND, PCA GND.']
diagram('wiring_arduino_uno.png', 'Arduino Uno + Pi or laptop', 'Arduino Uno',
        ['D3  FL EN   D2  FL IN1   D4  FL IN2', 'D5  FR EN   D7  FR IN1   D8  FR IN2', 'D6  RL EN   D12 RL IN1   D13 RL IN2', 'D9  RR EN   D10 RR IN1   D11 RR IN2', '', 'A4  SDA to PCA9685', 'A5  SCL to PCA9685', '5V  to PCA9685 VCC', 'GND to PCA9685 GND', '', 'A0..A3 encoder phase A, optional', 'D0, D1 stay free: USB serial', '', 'Upload firmware/amr_firmware', 'Flash used: 29 %'],
        ['Raspberry Pi 4/5 or laptop', 'runs ROS 2, lidar, camera,', 'Nav2', 'real.launch.py', 'backend:=serial', 'port:=/dev/ttyACM0'], 'USB', ['5 V board. Everything here is 5 V', 'tolerant: wire encoders directly.', 'L298N and PCA9685 are happy at 5 V.', 'Only one encoder phase per wheel: the', 'Uno has no pins left. Direction comes', 'from the command.'],
        'With encoders every Uno pin is used. The Uno cannot run\nROS: it always needs the Pi or a laptop.', 'encoders: A0..A3, set USE_ENCODERS 1')
diagram('wiring_esp32.png', 'ESP32 DevKit + Pi or laptop', 'ESP32 DevKit',
        ['GPIO25 FL EN  16 FL IN1  17 FL IN2', 'GPIO26 FR EN  18 FR IN1  19 FR IN2', 'GPIO27 RL EN  23 RL IN1  13 RL IN2', 'GPIO14 RR EN  32 RR IN1  33 RR IN2', '', 'GPIO21 SDA to PCA9685', 'GPIO22 SCL to PCA9685', '3V3    to PCA9685 VCC', 'GND    to PCA9685 GND', '', 'Encoders, full A+B:', 'A 34 35 36 39   B 4 5 15 2', '', 'Upload firmware/amr_firmware', 'Board: ESP32 Dev Module'],
        ['Raspberry Pi 4/5 or laptop', 'runs ROS 2, lidar, camera,', 'Nav2', 'real.launch.py', 'backend:=serial', 'port:=/dev/ttyUSB0'], 'USB', ['3.3 V board, NOT 5 V tolerant.', '5 V encoder outputs MUST go through', 'a divider (2.2k / 3.3k) or level shifter.', 'L298N reads 3.3 V as HIGH, so its inputs', 'can be wired straight. GPIO 2, 5, 15 are', 'strapping pins: keep encoders off at boot.'],
        'Needs the Pi or a laptop for ROS. Best board if you\nwant full A+B encoders on all four wheels.', 'encoders need level shifting: see below')
diagram('wiring_arduino_uno_q.png', 'Arduino UNO Q + Pi or laptop', 'Arduino UNO Q (MCU side)',
        ['D3  FL EN   D2  FL IN1   D4  FL IN2', 'D5  FR EN   D7  FR IN1   D8  FR IN2', 'D6  RL EN   D12 RL IN1   D13 RL IN2', 'D9  RR EN   D10 RR IN1   D11 RR IN2', '', 'SDA, SCL to PCA9685', '3V3 to PCA9685 VCC', 'GND to PCA9685 GND', '', 'D0 RX, D1 TX to USB-UART adapter', 'A0..A3 encoders, optional', '', 'Upload firmware/amr_firmware', 'NOT compiled yet: see docs/WIRING.md'],
        ['Raspberry Pi 4/5 or laptop', 'runs ROS 2, lidar, camera,', 'Nav2', 'real.launch.py', 'backend:=serial', 'port:=/dev/ttyUSB0'], '3.3 V USB-UART', ['3.3 V logic: 5 V encoders need dividers.', 'Serial1 on D0/D1 talks to ROS. On reset the', 'board prints a boot message there: the ROS', 'bridge ignores it (fails the checksum).', 'Its USB-C goes to the Linux side, not to', 'the sketch, so use the UART on the pins.'],
        'Same pins as the Uno. ROS on its own Linux side is\nuntested: start with a Pi or laptop as the host.', 'encoders need level shifting')
diagram('wiring_raspberry_pi_direct.png', 'Raspberry Pi alone, no microcontroller', 'Raspberry Pi 4/5 GPIO',
        ['GPIO17 FL IN1  GPIO18 FL IN2', 'GPIO22 FR IN1  GPIO23 FR IN2', 'GPIO24 RL IN1  GPIO25 RL IN2', 'GPIO5  RR IN1  GPIO6  RR IN2', '(your existing AMR-SIH pinout)', 'ENA, ENB: keep the L298N jumpers ON', '', 'GPIO2 SDA to PCA9685', 'GPIO3 SCL to PCA9685', '3V3   to PCA9685 VCC', 'GND   to PCA9685 GND', '', 'real.launch.py backend:=pi_gpio', 'pip install smbus2', 'sudo apt install python3-lgpio'],
        ['The Pi is the ROS computer', 'No USB link to a board', 'Needs: enable I2C in', 'raspi-config'], 'same board', ['3.3 V GPIO. No encoders in this mode:', 'odometry is an estimate from commands,', 'so lidar SLAM does the correcting.', 'Software PWM: expect some jitter.', 'For encoders, add a board over USB.'],
        'Simplest wiring. Here the L298N ENA/ENB jumpers stay ON,\nbecause the Pi puts its PWM on the IN pins.', '', jumpers='jumpers ON ENA/ENB')
# ------------- system diagram
fig, ax = plt.subplots(figsize=(14.5, 8.5)); ax.set_xlim(0, 16); ax.set_ylim(0, 10); ax.axis('off')
ax.text(0.2, 9.7, 'How the software fits together', fontsize=15, fontweight='bold', color=INK)
for x, y, w, h, t, ls, c in [
    (0.3, 7.4, 2.8, 1.5, 'Keyboard', ['keyboard_teleop'], '#e3edf7'), (0.3, 5.5, 2.8, 1.5, 'Joystick', ['joy_node, joy_teleop'], '#e3edf7'),
    (0.3, 3.2, 2.8, 1.8, 'RViz or command line', ['2D Goal Pose', 'ros2 run amr_control', 'mission ...'], '#e3edf7'),
    (4.2, 3.0, 3.1, 2.5, 'mission_manager', ['AUTO: navigate, look,', 'line up, pick, carry, place', 'closed-form arm IK'], '#dfeee5'),
    (4.2, 6.2, 3.1, 2.7, 'mode_mux', ['MANUAL / AUTO / STOP', 'the only publisher', 'of /cmd_vel'], '#f7dede'),
    (8.3, 6.6, 3.1, 2.3, 'mecanum_kinematics', ['/cmd_vel to wheel speeds', 'wheel feedback to /odom'], '#dfeee5'),
    (4.2, 0.4, 3.1, 2.0, 'SLAM + Nav2', ['slam_toolbox, DWB holonomic', '/scan to /map to /cmd_vel_nav'], '#ece3f4'),
    (8.3, 3.4, 3.1, 2.1, 'object_detector', ['camera to block position', 'on the floor plane'], '#ece3f4'),
    (8.3, 0.4, 3.1, 2.1, 'scan_filter', ['/scan_raw to /scan', 'removes the robot itself'], '#ece3f4'),
    (12.4, 6.0, 3.3, 2.9, 'Gazebo: gz_ros2_control', ['or real robot:', 'hardware_bridge', 'Uno, UNO Q, ESP32, Pi', 'Same topics either way'], '#f5ecd9'),
    (12.4, 3.0, 3.3, 2.4, 'Digital twin', ['twin_sync copies real joints', 'and wheels into Gazebo /sim,', 'reports error, fixes drift'], '#f4e0d6'),
    (12.4, 0.4, 3.3, 2.1, 'Sensors', ['YDLIDAR X2, base camera', 'real or simulated'], '#fbf3dc')]:
    box(ax, x, y, w, h, t, ls, c)
for a, b, l in [((3.1, 8.1), (4.2, 8.1), 'cmd_vel_teleop'), ((3.1, 6.3), (4.2, 6.9), None), ((3.1, 4.1), (4.2, 4.1), 'goal'), ((5.7, 5.5), (5.7, 6.2), 'cmd_vel_auto'),
                ((5.7, 2.4), (5.7, 3.0), 'nav goal'), ((7.3, 7.7), (8.3, 7.7), '/cmd_vel'), ((11.4, 7.7), (12.4, 7.7), 'wheel speeds'), ((8.3, 4.4), (7.3, 4.4), 'blocks'),
                ((8.3, 1.4), (7.3, 1.4), '/scan'), ((12.4, 1.4), (11.4, 1.4), 'scan_raw'), ((14.0, 2.5), (14.0, 3.0), None), ((14.0, 5.4), (14.0, 6.0), 'joint states')]:
    wire(ax, a, b, INK, l)
fig.savefig(os.path.join(out, 'system_diagram.png'), dpi=130, bbox_inches='tight', facecolor='white'); plt.close(fig); print('wrote system')
