# Robot code for each board

All boards speak the same protocol to the same ROS code, so Gazebo, the twin and the real robot work the same way.

| Board | Code | Upload or run | Launch |
|---|---|---|---|
| Arduino Uno | `amr_firmware/` | Arduino IDE, board "Arduino Uno" | `backend:=serial port:=/dev/ttyACM0` |
| Arduino Uno + HC-05 | `amr_firmware/` | Same sketch, unplug HC-05 to upload | `backend:=serial port:=/dev/rfcomm0` |
| ESP32 DevKit | `amr_firmware/` | Arduino IDE, board "ESP32 Dev Module" | `backend:=serial port:=/dev/ttyUSB0` |
| Arduino UNO Q | `amr_firmware/` | Arduino IDE or App Lab | `backend:=serial port:=/dev/ttyUSB0` |
| Raspberry Pi alone | `ros2_ws/src/amr_control/amr_control/backends.py` | No upload, runs inside ROS | `backend:=pi_gpio` |

The sketch picks the right pins from the board you select. Everything you may need to change is in `amr_firmware/config.h`.

| Board | Test status |
|---|---|
| Uno | Compiles, 29% flash and 34% RAM; logic passes the serial link tests |
| ESP32 | Compiles, with and without encoders |
| UNO Q | **Not compiled yet**, its board package isn't installed here |
| Pi | Runs through the same bridge, but the GPIO code needs a real Pi to run |

None of them has run on real hardware yet. Wiring, the HC-05 setup and first power-on steps are in [docs/WIRING.md](../docs/WIRING.md).
