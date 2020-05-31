import utime
from pyb import I2C, RTC, Pin, UART
from pyb_i2c_lcd import I2cLcd

from pca9685 import PCA9685
from profiler import Profiler
from rc import get_rc_command
from state import StateMachine


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

    sbus_uart = UART(2, 100000, bits=8, parity=0, stop=2, read_buf_len=50, timeout=0)
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

    profiler = Profiler(None, None, None, None, None, None, None, None, None, None, None, None)
    state_machine = StateMachine()

    next_loop_us = 0
    # put methods on stack to prevent lookups
    ticks_us = utime.ticks_us
    frs_cmd = front_right_shoulder.command_deg
    fls_cmd = front_left_shoulder.command_deg
    frl_cmd = front_right_leg.command_deg
    fll_cmd = front_left_leg.command_deg
    frf_cmd = front_right_foot.command_deg
    flf_cmd = front_left_foot.command_deg

    rrs_cmd = rear_right_shoulder.command_deg
    rls_cmd = rear_left_shoulder.command_deg
    rrl_cmd = rear_right_leg.command_deg
    rll_cmd = rear_left_leg.command_deg
    rrf_cmd = rear_right_foot.command_deg
    rlf_cmd = rear_left_foot.command_deg
    loop_rate_us = SpotServo.period_us - 360  # fudge factor here to match loop with ~50Hz PWM = 19.64ms
    while True:
        profiler.tick()
        pos_cmds = profiler.get_position_commands()

        lcd.move_to(0, 0)
        lcd.putstr("Hello {}".format(ticks_us()))
        rc_command = get_rc_command(sbus_uart)
        state_machine.update(profiler, rc_command, next_loop_us)

        while ticks_us() < next_loop_us:
            pass  # wait for next loop
        next_loop_us = ticks_us() + loop_rate_us
        if pos_cmds[0] is not None:
            frs_cmd(pos_cmds[0])
        if pos_cmds[1] is not None:
            fls_cmd(pos_cmds[1])
        if pos_cmds[2] is not None:
            frl_cmd(pos_cmds[2])
        if pos_cmds[3] is not None:
            fll_cmd(pos_cmds[3])
        if pos_cmds[4] is not None:
            frf_cmd(pos_cmds[4])
        if pos_cmds[5] is not None:
            flf_cmd(pos_cmds[5])
        if pos_cmds[6] is not None:
            rrs_cmd(pos_cmds[6])
        if pos_cmds[7] is not None:
            rls_cmd(pos_cmds[7])
        if pos_cmds[8] is not None:
            rrl_cmd(pos_cmds[8])
        if pos_cmds[9] is not None:
            rll_cmd(pos_cmds[9])
        if pos_cmds[10] is not None:
            rrf_cmd(pos_cmds[10])
        if pos_cmds[11] is not None:
            rlf_cmd(pos_cmds[11])

        debug_uart.write("\x02{}\x03\r\n".format("\t".join(
            [str(x) for x in pos_cmds + [state_machine.state, len(profiler.position_target_queue), ticks_us()]])))
        # if rc_command is not None:
        #     debug_uart.write("\x02{}\x03\r\n".format("\t".join([str(x) for x in rc_command])))


if __name__ == '__main__':
    rtc, lcd, sbus_uart, debug_uart = setup()

    front_right_shoulder = SpotServo(0, 0, 82, 180, False)
    front_right_leg = SpotServo(1, 0, 115, 180, False)
    front_right_foot = SpotServo(2, 0, 32, 180, False)
    front_left_shoulder = SpotServo(4, 0, 97, 180, True)
    front_left_leg = SpotServo(5, 0, 87, 180, True)
    front_left_foot = SpotServo(6, 0, 140, 180, True)

    rear_right_shoulder = SpotServo(8, 0, 70, 180, False)
    rear_right_leg = SpotServo(9, 0, 90, 180, False)
    rear_right_foot = SpotServo(10, 0, 32, 180, False)
    rear_left_shoulder = SpotServo(12, 0, 100, 180, True)
    rear_left_leg = SpotServo(13, 0, 90, 180, True)
    rear_left_foot = SpotServo(14, 0, 140, 180, True)

    main(rtc, lcd, sbus_uart, debug_uart)
