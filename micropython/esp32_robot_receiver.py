"""
SpotMicro Robot Controller - ESP32-S3 Receiver
Port of PyBoard robot_receiver.py for ESP32-S3

Changes from PyBoard:
  - pyb.USB_VCP  → sys.stdin/stdout (USB serial via REPL)
  - pyb.I2C      → machine.I2C with explicit pins
  - pyb.delay    → utime.sleep_ms

Wiring ESP32-S3 → PCA9685:
  GPIO 8  → SDA
  GPIO 9  → SCL
  3.3V    → VCC
  GND     → GND

Commands (same as before):
  STAND, SIT,
  LEAN_LEFT, LEAN_CENTER, LEAN_RIGHT,
  WALK_FORWARD, WALK_CENTER, WALK_BACKWARD,
  CRAWL_LEFT, CRAWL_STOP, CRAWL_RIGHT
"""

import utime
import sys
from machine import I2C, Pin
from pca9685 import PCA9685
from profiler import Profiler
from state import StateMachine

# ============================================================
# USB Serial - dùng UART(0) cho ESP32-S3
# UART(0) là cổng USB-Serial mặc định trên ESP32
# ============================================================
from machine import UART

class UsbSerial:
    """Uses UART(0) for USB serial on ESP32-S3 (replaces PyBoard USB_VCP)"""
    def __init__(self):
        # timeout=0 = non-blocking read
        self._uart = UART(0, baudrate=115200, timeout=0)

    def write(self, data):
        if isinstance(data, str):
            data = data.encode()
        self._uart.write(data)

    def any(self):
        return self._uart.any()

    def read(self, n=64):
        return self._uart.read(n)

# ============================================================
# Hardware init
# ============================================================
print("Initializing hardware...")

# I2C - thay đổi GPIO nếu mày dùng pin khác
I2C_SDA = 8
I2C_SCL = 9
i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)

# Scan I2C để verify PCA9685
devices = i2c.scan()
print("I2C devices:", [hex(d) for d in devices])
if 0x40 not in devices:
    print("WARNING: PCA9685 not found! Check wiring.")

pca = PCA9685(i2c, 0x40)
pca.freq(50)
print("PCA9685 ready")

# ============================================================
# Servo class (không đổi)
# ============================================================
class Servo:
    us_min_cmd = 500.0
    us_max_cmd = 2500.0
    deg_min_cmd = 0.0
    deg_max_cmd = 180.0
    us_per_180_degs = us_max_cmd - us_min_cmd
    us_per_deg = us_per_180_degs / 180.0
    freq_hz = 50.0
    period_us = (1.0 / freq_hz) * 1e6
    pca9685 = pca

    def __init__(self, pwm_channel, min_angle_deg, center_angle_deg, max_angle_deg, is_inverted):
        self.pwm_channel = pwm_channel
        self.min_angle_deg = min_angle_deg
        self.center_angle_deg = center_angle_deg
        self.max_angle_deg = max_angle_deg
        self.is_inverted = is_inverted

    @staticmethod
    def get_12_bit_duty_cycle_for_us(position_us):
        if position_us < Servo.us_min_cmd or position_us > Servo.us_max_cmd:
            raise ValueError
        return int((float(position_us) / float(Servo.period_us)) * (2 ** 12))

    @staticmethod
    def get_12_bit_duty_cycle_for_angle(angle_deg):
        if angle_deg < Servo.deg_min_cmd or angle_deg > Servo.deg_max_cmd:
            raise ValueError
        target_us = Servo.us_min_cmd + (float(angle_deg) * Servo.us_per_deg)
        return Servo.get_12_bit_duty_cycle_for_us(target_us)

    def command_deg(self, angle_deg):
        offset = -angle_deg if self.is_inverted else angle_deg
        angle_raw = self.center_angle_deg + offset
        if angle_raw < self.min_angle_deg or angle_raw > self.max_angle_deg:
            raise ValueError
        self.pca9685.duty(self.pwm_channel, Servo.get_12_bit_duty_cycle_for_angle(angle_raw))

# ============================================================
# Servo initialization (calibrated angles - chỉnh tại đây)
# ============================================================
front_right_shoulder = Servo(0,  0, 100, 180, False)
front_right_leg      = Servo(1,  0, 100, 180, False)
front_right_foot     = Servo(2,  0,  30, 180, False)

front_left_shoulder  = Servo(4,  0,  80, 180, True)
front_left_leg       = Servo(5,  0,  80, 180, True)
front_left_foot      = Servo(6,  0, 145, 180, True)

rear_right_shoulder  = Servo(8,  0,  90, 180, False)
rear_right_leg       = Servo(9,  0,  90, 180, False)
rear_right_foot      = Servo(10, 0,  30, 180, False)

rear_left_shoulder   = Servo(12, 0,  80, 180, True)
rear_left_leg        = Servo(13, 0,  85, 180, True)
rear_left_foot       = Servo(14, 0, 145, 180, True)

# Profiler + state machine
profiler = Profiler()
state_machine = StateMachine()

# USB serial
usb = UsbSerial()

# Servo command list (thứ tự phải khớp với profiler.get_position_commands())
servo_commands = [
    front_right_shoulder.command_deg, front_left_shoulder.command_deg,
    front_right_leg.command_deg,      front_left_leg.command_deg,
    front_right_foot.command_deg,     front_left_foot.command_deg,
    rear_right_shoulder.command_deg,  rear_left_shoulder.command_deg,
    rear_right_leg.command_deg,       rear_left_leg.command_deg,
    rear_right_foot.command_deg,      rear_left_foot.command_deg,
]

print("Ready for PC control commands")
usb.write("READY\n")

# ============================================================
# Crawl state
# ============================================================
crawl_mode = 0       # 0=stop, 1=left, 2=right
crawl_rc_channels = None

# ============================================================
# Main control loop
# ============================================================
buffer = ""
last_loop_us = utime.ticks_us()
loop_period_us = 19650  # ~50Hz

while True:
    # ---- Read incoming commands ----
    if usb.any():
        try:
            data = usb.read(64)
            if data:
                buffer += data.decode('utf-8', 'ignore')
        except:
            pass

        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            cmd = line.strip()

            if not cmd:
                continue

            try:
                if cmd == "STAND":
                    rc_channels = [980, 980, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK STAND\n")
                    print("Standing up...")

                elif cmd == "SIT":
                    rc_channels = [980, 980, 980, 980, 980, 980, 1580, 180] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK SIT\n")
                    print("Sitting down...")

                elif cmd == "LEAN_LEFT":
                    rc_channels = [980, 1380, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK LEAN_LEFT\n")

                elif cmd == "LEAN_CENTER":
                    rc_channels = [980, 980, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK LEAN_CENTER\n")

                elif cmd == "LEAN_RIGHT":
                    rc_channels = [980, 580, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK LEAN_RIGHT\n")

                elif cmd == "WALK_FORWARD":
                    rc_channels = [1380, 980, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK WALK_FORWARD\n")

                elif cmd == "WALK_CENTER":
                    rc_channels = [980, 980, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK WALK_CENTER\n")

                elif cmd == "WALK_BACKWARD":
                    rc_channels = [580, 980, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK WALK_BACKWARD\n")

                elif cmd == "CRAWL_LEFT":
                    crawl_mode = 1
                    crawl_rc_channels = [1380, 980, 980, 980, 980, 980, 1580, 1580] + [980] * 8
                    usb.write("OK CRAWL_LEFT\n")
                    print("Crawling left...")

                elif cmd == "CRAWL_STOP":
                    crawl_mode = 0
                    crawl_rc_channels = None
                    rc_channels = [980, 980, 980, 980, 980, 980, 1580, 1000] + [980] * 8
                    state_machine.update(profiler, rc_channels, last_loop_us)
                    usb.write("OK CRAWL_STOP\n")
                    print("Stopping crawl...")

                elif cmd == "CRAWL_RIGHT":
                    crawl_mode = 2
                    crawl_rc_channels = [580, 980, 980, 980, 980, 980, 1580, 1580] + [980] * 8
                    usb.write("OK CRAWL_RIGHT\n")
                    print("Crawling right...")

                else:
                    usb.write("ERROR Unknown command\n")

            except Exception as e:
                usb.write("ERROR {}\n".format(str(e)))

    # ---- Continue crawling ----
    if crawl_mode != 0 and crawl_rc_channels is not None:
        state_machine.update(profiler, crawl_rc_channels, last_loop_us)

    # ---- Update profiler + servos ----
    profiler.tick()
    pos_cmds = profiler.get_position_commands()

    for i, c in enumerate(pos_cmds):
        if c is not None:
            try:
                servo_commands[i](c)
            except:
                pass

    # ---- Timing: 50Hz loop ----
    while utime.ticks_diff(utime.ticks_us(), last_loop_us) < loop_period_us:
        pass
    last_loop_us = utime.ticks_us()
