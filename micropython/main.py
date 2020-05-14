import utime
import struct
from pyb import I2C, RTC, Pin, UART
from pyb_i2c_lcd import I2cLcd
from pca9685 import PCA9685


class SpotServo:
    us_min_cmd = 500.0
    us_max_cmd = 2500.0
    deg_min_cmd = 0.0
    deg_max_cmd = 180.0

    us_per_180_degs = us_max_cmd - us_min_cmd
    us_per_deg = us_per_180_degs / 180.0

    freq_hz = 50.0
    period_us = (1.0 / freq_hz) * 1e6

    pca9685 = None

    def __init__(self, pwm_channel: int, min_angle_deg: float, center_angle_deg: float,
                 max_angle_deg: float, is_inverted: bool):
        # All angles passed in here should be raw angles so:
        #     The max angle is not offset by the mid angle
        #     The max angle does not consider inverted
        self.pwm_channel = pwm_channel
        self.min_angle_deg = min_angle_deg
        self.center_angle_deg = center_angle_deg
        self.max_angle_deg = max_angle_deg
        self.is_inverted = is_inverted

    @staticmethod
    def get_12_bit_duty_cycle_for_us(position_us):
        if position_us < SpotServo.us_min_cmd or position_us > SpotServo.us_max_cmd:
            raise ValueError
        duty_cycle_12_bit = int((float(position_us) / float(SpotServo.period_us)) * (2 ** 12))
        return duty_cycle_12_bit

    @staticmethod
    def get_12_bit_duty_cycle_for_angle(angle_deg):
        if angle_deg < SpotServo.deg_min_cmd or angle_deg > SpotServo.deg_max_cmd:
            raise ValueError
        target_us = SpotServo.us_min_cmd + (float(angle_deg) * SpotServo.us_per_deg)
        return SpotServo.get_12_bit_duty_cycle_for_us(target_us)

    def command_deg(self, angle_deg):
        # zero here will be legs straight down
        angle_deg_with_inversion_relative_to_middle = angle_deg
        if self.is_inverted:
            angle_deg_with_inversion_relative_to_middle = -angle_deg
        angle_deg_raw = self.center_angle_deg + angle_deg_with_inversion_relative_to_middle
        if (angle_deg_raw < self.min_angle_deg) or (angle_deg_raw > self.max_angle_deg):
            raise ValueError
        self.pca9685.duty(self.pwm_channel, SpotServo.get_12_bit_duty_cycle_for_angle(angle_deg_raw))


def setup():
    LCD_ADDR = 0x3F
    PWM_ADDR = 0x40
    # MPU_ADDR = 0x68
    i2c = I2C(1, I2C.MASTER)
    lcd = I2cLcd(i2c, LCD_ADDR, 2, 16)

    pwm_not_enabled_pin = Pin("Y8", Pin.OUT_PP)
    pwm_not_enabled_pin.low()

    SpotServo.pca9685 = PCA9685(i2c, PWM_ADDR)
    SpotServo.pca9685.freq(SpotServo.freq_hz)

    rtc = RTC()
    rtc.datetime((2020, 3, 8, 7, 21, 32, 0, 0))

    sbus_uart = UART(2, 100000, bits=8, parity=0, stop=2, read_buf_len=50)
    debug_uart = UART(3, 115200)

    return rtc, lcd, sbus_uart, debug_uart


def main(rtc, lcd, sbus_uart, debug_uart):
    # front right calibration
    # pca9685.duty(0, SpotServo.get_12_bit_duty_cycle_for_angle(82))  # positive is external rotation
    # pca9685.duty(1, SpotServo.get_12_bit_duty_cycle_for_angle(115))  # positive is forward
    # pca9685.duty(2, SpotServo.get_12_bit_duty_cycle_for_angle(32))  # positive is forward
    # front left calibration
    # pca9685.duty(4, SpotServo.get_12_bit_duty_cycle_for_angle(97))   # positive is internal rotation
    # pca9685.duty(5, SpotServo.get_12_bit_duty_cycle_for_angle(87))  # positive is back
    # pca9685.duty(6, SpotServo.get_12_bit_duty_cycle_for_angle(140))  # positive is back

    # pose: shoulder elbow wrist
    # sphinx: 0 -50 110
    # down_chicken: 40 -90 140

    front_right_shoulder.command_deg(40)
    front_left_shoulder.command_deg(40)

    front_right_leg.command_deg(-60)  # -60 is arm out flat, start here then go to -90
    front_left_leg.command_deg(-60)

    front_right_foot.command_deg(140)
    front_left_foot.command_deg(140)

    min_sweep_deg = 0
    max_sweep_deg = 40
    position_deg = max_sweep_deg

    going_up = False
    next_loop_us = 0
    # put methods on stack to prevent lookups
    ticks_us = utime.ticks_us
    fr_cmd = front_right_shoulder.command_deg
    fl_cmd = front_left_shoulder.command_deg
    loop_rate_us = SpotServo.period_us - 575  # fudge factor here to match loop with PWM
    while True:
        if going_up:
            position_deg += 0.1
            pass
        else:
            position_deg -= 0.1
            pass

        lcd.move_to(0, 0)
        lcd.putstr("Loops:{:10}12-bit:{:9}".format(position_deg, position_deg))

        if position_deg >= max_sweep_deg:
            going_up = False
        elif position_deg <= min_sweep_deg:
            going_up = True
        while ticks_us() < next_loop_us:
            pass
        # fr_cmd(position_deg)
        # fl_cmd(position_deg)
        next_loop_us = ticks_us() + loop_rate_us


if __name__ == '__main__':
    rtc, lcd, sbus_uart, debug_uart = setup()

    sbus_buffer = bytearray(50)
    while True:
        wait = utime.ticks_us() + 0.1e6
        while utime.ticks_us() < wait:
            pass
        sbus_uart.readinto(sbus_buffer)

        start_byte_index = None
        for sbus_index in range(len(sbus_buffer) - 24):
            if sbus_buffer[sbus_index] == 0x0F and sbus_buffer[sbus_index + 24] == 0x00:
                start_byte_index = sbus_index
                break

        if start_byte_index is None:
            debug_uart.write("BAD\r\n")
        else:
            sbus_frame = sbus_buffer[start_byte_index:start_byte_index + 25]
            sbus_as_int = int.from_bytes(sbus_frame, "little")

            channels = []
            for channel_index in range(16):
                shift = (8 + (channel_index * 11))
                channel_value = (sbus_as_int & (0x7FF << shift)) >> shift
                channels.append(channel_value)
            for bit_index in range(4):  # digital channels 17, 18, frame lost flag, failsafe flag
                channels.append(bool((sbus_frame[-2] & (0x1 << bit_index)) >> bit_index))

            debug_uart.write("{}\r\n".format("\t".join([str(x) for x in sbus_frame])))
            debug_uart.write("{}\r\n".format("\t".join([str(c) for c in channels])))

    front_right_shoulder = SpotServo(0, 0, 82, 180, False)
    front_right_leg = SpotServo(1, 0, 115, 180, False)
    front_right_foot = SpotServo(2, 0, 32, 180, False)
    front_left_shoulder = SpotServo(4, 0, 97, 180, True)
    front_left_leg = SpotServo(5, 0, 87, 180, True)
    front_left_foot = SpotServo(6, 0, 140, 180, True)
    main(rtc, lcd, sbus_uart, debug_uart)
