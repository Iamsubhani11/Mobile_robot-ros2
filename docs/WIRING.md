# Wiring

Pins below come from `firmware/amr_firmware/config.h`, and from `backends.py` for the Pi-only setup. If you change a pin, change it there.

None of this has been tested on real hardware yet, so check each step with the wheels off the ground.

## Choose a setup

| Setup | Diagram | Runs ROS on | Encoders |
|---|---|---|---|
| Arduino Uno + Pi or laptop | [diagram](../images/wiring_arduino_uno.png) | Pi or laptop | One channel per wheel, A0 to A3 |
| ESP32 + Pi or laptop | [diagram](../images/wiring_esp32.png) | Pi or laptop | A and B on all wheels, needs level shifting |
| Arduino UNO Q + Pi or laptop | [diagram](../images/wiring_arduino_uno_q.png) | Pi or laptop | One channel per wheel, needs level shifting |
| Raspberry Pi alone | [diagram](../images/wiring_raspberry_pi_direct.png) | Pi | None, odometry is estimated from commands |

The ESP32 is the best choice if you want full encoders on every wheel. The Pi alone is the simplest to wire.

## Power

| From | To |
|---|---|
| 3S LiPo 11.1 V, through a 15 A fuse and a switch | Both L298N +12 V inputs |
| Battery through a 6 V 10 A UBEC | PCA9685 V+ (servo power) |
| Battery through a 5 V 5 A UBEC | Raspberry Pi |

- Join all grounds: battery, UBECs, L298N, control board and PCA9685.
- Do not use the L298N 5 V pin to power the control board.
- Do not power the servos from the control board.

## Motors: two L298N

### Uno and UNO Q

| Wheel | EN (PWM) | IN1 | IN2 | Encoder (optional) |
|---|---|---|---|---|
| Front left | D3 | D2 | D4 | A0 |
| Front right | D5 | D7 | D8 | A1 |
| Rear left | D6 | D12 | D13 | A2 |
| Rear right | D9 | D10 | D11 | A3 |

### ESP32

| Wheel | EN (PWM) | IN1 | IN2 | Encoder A | Encoder B |
|---|---|---|---|---|---|
| Front left | GPIO25 | GPIO16 | GPIO17 | GPIO34 | GPIO4 |
| Front right | GPIO26 | GPIO18 | GPIO19 | GPIO35 | GPIO5 |
| Rear left | GPIO27 | GPIO23 | GPIO13 | GPIO36 | GPIO15 |
| Rear right | GPIO14 | GPIO32 | GPIO33 | GPIO39 | GPIO2 |

The ESP32 is not 5 V tolerant, so 5 V encoder outputs must go through level shifters or dividers.

### Raspberry Pi alone

| Wheel | IN1 | IN2 |
|---|---|---|
| Front left | GPIO17 | GPIO18 |
| Front right | GPIO22 | GPIO23 |
| Rear left | GPIO24 | GPIO25 |
| Rear right | GPIO5 | GPIO6 |

Keep the L298N ENA and ENB jumpers on, because the Pi puts its PWM on the IN pins. Enable I2C in `raspi-config`.

## Servos: PCA9685 at I2C address 0x40

| Channel | Joint |
|---|---|
| 0 | Waist |
| 1 | Shoulder |
| 2 | Elbow |
| 3 | Wrist |
| 4 | Gripper |

The gripper's build guide is in [hardware/README.md](../hardware/README.md).

| Board | SDA | SCL |
|---|---|---|
| Uno | A4 | A5 |
| ESP32 | GPIO21 | GPIO22 |
| UNO Q | SDA | SCL |

On the UNO Q, power the PCA9685 logic (VCC) from 3V3. To size your servos, run `python3 tools/servo_torque.py robot.urdf [payload_kg]`.

## Are there enough pins?

Yes, on every board.

| Uno pins | Use |
|---|---|
| D0, D1 | Link to ROS: USB or HC-05 (they share these pins) |
| D2 to D13 | Motors (12 pins) |
| A4, A5 | I2C to the PCA9685, which drives all 5 servos |
| A0 to A3 | One encoder channel per wheel, or free without encoders |

That is all 20 Uno pins with encoders, or 16 of 20 without. The servos use only the two I2C pins because the PCA9685 drives them.

- **ESP32:** GPIO34 to 39 are input only and have no internal pull-ups, so encoders on them need external pull-ups. GPIO2, 5 and 15 affect booting, so if upload fails, unplug the encoders on those pins.
- **UNO Q:** same pins as the Uno, but with 3.3 V logic.

## Arduino Uno with HC-05 Bluetooth

The HC-05 replaces the USB cable to the Pi or laptop. It runs on D0 and D1, the Uno's hardware serial port, so the firmware doesn't change. Don't use SoftwareSerial, because it can't keep up at this speed.

| HC-05 | Uno |
|---|---|
| VCC | 5V |
| GND | GND |
| TXD | D0 (RX) |
| RXD | D1 (TX), through a divider: 1 kΩ from D1 to RXD, 2 kΩ from RXD to GND |

1. **Set the speed once.** Hold the HC-05 button while powering it to enter AT mode, then send `AT+UART=115200,0,0` from a USB-serial adapter at 38400 baud.
2. **Upload:** always unplug the HC-05 from D0 and D1 while uploading, then plug it back in.
3. **Power the Uno** from the 5 V UBEC on its 5V pin, not from the L298N.
4. **Pair from the computer:**
   ```bash
   bluetoothctl                         # then: scan on, pair <MAC>, trust <MAC>
   sudo rfcomm bind 0 <MAC>
   ros2 launch amr_control real.launch.py backend:=serial port:=/dev/rfcomm0
   ```

If the link drops, the firmware stops the motors within 500 ms.

The HC-05 only carries motor and servo commands. The lidar and camera still need a USB connection to a computer running ROS, so for navigation keep a Pi on the robot. Bluetooth from a laptop suits keyboard or joystick driving.

## Connect the board to ROS

| Board | Connection | Launch option |
|---|---|---|
| Uno | USB | `port:=/dev/ttyACM0` |
| ESP32 | USB | `port:=/dev/ttyUSB0` |
| UNO Q | 3.3 V USB-UART adapter on D0 and D1 | `port:=/dev/ttyUSB0` |

The UNO Q's USB-C goes to its Linux side rather than the sketch, so use the UART pins. Its firmware has not been compiled yet: check it in the Arduino IDE first.

Upload `firmware/amr_firmware` with the Arduino IDE. It needs no extra libraries beyond the board's own `Wire`.

## First power-on

1. Keep the wheels off the ground and the arm clear.
2. If a wheel spins backwards, flip it in `MOTOR_SIGN` in `config.h`.
3. If a joint moves the wrong way, flip it in `SERVO_SIGN`.
4. Set each arm `SERVO_CENTER_US` so the arm matches the CAD pose at zero. The gripper's is 800 µs, fully open.
5. Measure the wheel speed at full PWM. Set it as `MAX_WHEEL_RAD_S` in `config.h` and `max_wheel_speed` in `robot.yaml`.
6. Raise `MIN_PWM` until the wheels start reliably.
7. If you wired encoders, set `USE_ENCODERS` to 1 and set `TICKS_PER_WHEEL_REV`.

The motors stop if no wheel command arrives for 500 ms.

## Lidar: YDLIDAR X2

Plug it into the computer running ROS, and install its driver there:

1. Build and install [YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK).
2. Build [ydlidar_ros2_driver](https://github.com/YDLIDAR/ydlidar_ros2_driver) in your workspace.
3. Launch with `lidar:=true lidar_port:=/dev/ttyUSB1`.

Settings are in `ros2_ws/src/amr_control/config/ydlidar_x2.yaml`.

## Camera

Install `v4l2_camera` (`sudo apt install ros-humble-v4l2-camera`), then launch with `camera:=true camera_device:=/dev/video0`.

## Serial protocol

Lines are `$BODY*CS`, where `CS` is the two-digit hex XOR of `BODY`. The baud rate is 115200, and lines with a bad checksum are ignored.

| Direction | Message | Meaning |
|---|---|---|
| To board | `V,fl,fr,rl,rr` | Wheel speeds in rad/s × 1000 |
| To board | `J,waist,shoulder,elbow,wrist,gripper` | Joint targets in rad × 1000 |
| To board | `Q` | Ask for board info |
| From board | `S,...` | State: wheel positions, wheel speeds, joints, flags |
