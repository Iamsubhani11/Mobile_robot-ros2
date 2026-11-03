"""Hardware backends for hardware_bridge. See docs/WIRING.md for the serial protocol and wiring."""
import math


def checksum(body):
    c = 0
    for ch in body.encode():
        c ^= ch
    return f'{c:02X}'


def frame(body):
    return f'${body}*{checksum(body)}\n'.encode()


def parse_frame(line):
    """Returns the field list, or None for noise or a bad checksum (boot banners end up here)."""
    line = line.strip()
    if not line.startswith('$') or '*' not in line:
        return None
    body, _, cs = line[1:].rpartition('*')
    if checksum(body) != cs.upper():
        return None
    return body.split(',')


class SerialBackend:
    """Arduino Uno, UNO Q or ESP32 over a serial line. Units on the wire: milli-rad and milli-rad/s."""
    def __init__(self, params, log):
        import serial
        self.log = log
        self.port = serial.serial_for_url(params['port'], params['baud'], timeout=0, write_timeout=0.05)
        self.buf = b''; self.state = None; self.bad = 0; self.status = None; self.last_j = None
        log.info(f"serial backend on {params['port']} at {params['baud']} baud")

    def write(self, wheels, joints):
        try:
            self.port.write(frame('V,' + ','.join(str(int(round(w * 1000))) for w in wheels)))
            if joints is not None and not any(math.isnan(j) for j in joints):
                ints = [int(round(j * 1000)) for j in joints]
                if ints != self.last_j:
                    self.port.write(frame('J,' + ','.join(map(str, ints))))
                    self.last_j = ints
        except Exception as e:
            self.log.warning(f'serial write failed: {e}', throttle_duration_sec=2.0)

    def read(self, dt):
        try:
            self.buf += self.port.read(4096)
        except Exception as e:
            self.log.warning(f'serial read failed: {e}', throttle_duration_sec=2.0)
            return None
        *lines, self.buf = self.buf.split(b'\n')
        new = False
        for raw in lines:
            f = parse_frame(raw.decode(errors='ignore'))
            if f is None:
                self.bad += 1
            elif f[0] == 'S' and len(f) == 16:
                v = [int(x) / 1000.0 for x in f[2:15]]
                self.state = dict(wheel_pos=v[0:4], wheel_vel=v[4:8], joints=v[8:13], flags=int(f[15]))
                new = True
            elif f[0] == 'I':
                self.status = 'firmware ' + ','.join(f[1:])
                self.log.info(self.status)
        if not new:
            return None
        out = dict(self.state)
        if self.status:
            out['status'] = self.status
            self.status = None
        return out

    def close(self):
        try:
            self.port.write(frame('V,0,0,0,0'))
            self.port.close()
        except Exception:
            pass


class MockBackend:
    """First-order model of the robot. mock_wheel_gain != 1 imitates slip or a mismatched motor."""
    def __init__(self, params, log):
        self.lag = params['mock_lag']; self.gain = params['mock_wheel_gain']
        self.v = [0.0] * 4; self.p = [0.0] * 4; self.j = [0.0] * 5; self.target = [0.0] * 5; self.cmd = [0.0] * 4
        log.warning('MOCK backend: no real hardware is connected')

    def write(self, wheels, joints):
        self.cmd = list(wheels)
        if joints is not None:
            self.target = [t if math.isnan(n) else n for t, n in zip(self.target, joints)]

    def read(self, dt):
        a = min(1.0, dt / max(self.lag, 1e-3))
        for i in range(4):
            self.v[i] += a * (self.cmd[i] * self.gain - self.v[i]); self.p[i] += self.v[i] * dt
        for i in range(5):
            step = 1.5 * dt
            self.j[i] += max(-step, min(step, self.target[i] - self.j[i]))
        return dict(wheel_pos=list(self.p), wheel_vel=list(self.v), joints=list(self.j))

    def close(self):
        pass


class PiGpioBackend:
    """Raspberry Pi running ROS itself: lgpio soft-PWM to motor drivers, PCA9685 on I2C for the servos.
    Motor pins are the ones already wired on the AMR-SIH robot. No encoders: wheel feedback equals the command."""
    PINS = [(17, 18), (22, 23), (24, 25), (5, 6)]      # FL FR RL RR, each (IN1, IN2)
    SERVO_CH = [0, 1, 2, 3, 4]
    SERVO_SIGN = [1, 1, 1, 1, 1]
    CENTER_US = [1500, 1500, 1500, 1500, 800]   # pulse at q = 0; the gripper's q = 0 is fully open
    US_PER_RAD = 636.6; MIN_US = 500; MAX_US = 2500
    MAX_RATE = 1.5

    def __init__(self, params, log):
        import lgpio
        from smbus2 import SMBus
        self.lgpio = lgpio; self.max_w = params['max_wheel_speed']; self.freq = params['pwm_frequency']
        self.h = lgpio.gpiochip_open(0)
        for a, b in self.PINS:
            lgpio.gpio_claim_output(self.h, a, 0); lgpio.gpio_claim_output(self.h, b, 0)
        self.bus = SMBus(params['i2c_bus']); self.addr = 0x40
        self.bus.write_byte_data(self.addr, 0x00, 0x10)                   # sleep to set prescale
        self.bus.write_byte_data(self.addr, 0xFE, round(25e6 / (4096 * 50)) - 1)   # 50 Hz
        self.bus.write_byte_data(self.addr, 0x00, 0xA0)                   # wake, auto-increment
        self.v = [0.0] * 4; self.p = [0.0] * 4; self.j = [0.0] * 5; self.target = [0.0] * 5
        log.info('pi_gpio backend ready')

    def write(self, wheels, joints):
        for i, (a, b) in enumerate(self.PINS):
            duty = max(-100.0, min(100.0, wheels[i] / self.max_w * 100.0))
            self.lgpio.tx_pwm(self.h, a, self.freq, max(duty, 0.0))
            self.lgpio.tx_pwm(self.h, b, self.freq, max(-duty, 0.0))
            self.v[i] = duty / 100.0 * self.max_w
        if joints is not None:
            self.target = [t if math.isnan(n) else n for t, n in zip(self.target, joints)]

    def _servo(self, i, ch, rad, sign):
        us = max(self.MIN_US, min(self.MAX_US, self.CENTER_US[i] + sign * rad * self.US_PER_RAD))
        count = int(us * 4096 * 50 / 1e6)
        self.bus.write_i2c_block_data(self.addr, 0x06 + 4 * ch, [0, 0, count & 0xFF, count >> 8])

    def read(self, dt):
        for i in range(5):
            step = self.MAX_RATE * dt
            self.j[i] += max(-step, min(step, self.target[i] - self.j[i]))
            self._servo(i, self.SERVO_CH[i], self.j[i], self.SERVO_SIGN[i])
        for i in range(4):
            self.p[i] += self.v[i] * dt
        return dict(wheel_pos=list(self.p), wheel_vel=list(self.v), joints=list(self.j))

    def close(self):
        for a, b in self.PINS:
            self.lgpio.tx_pwm(self.h, a, self.freq, 0); self.lgpio.tx_pwm(self.h, b, self.freq, 0)
        self.lgpio.gpiochip_close(self.h)
        self.bus.close()


def make_backend(name, params, log):
    table = dict(serial=SerialBackend, mock=MockBackend, pi_gpio=PiGpioBackend)
    if name not in table:
        raise ValueError(f'unknown backend {name!r}, use one of {list(table)}')
    return table[name](params, log)
