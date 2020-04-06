from pyb import I2C, delay, millis, RTC, Pin
from pyb_i2c_lcd import I2cLcd
from pca9685 import PCA9685

LCD_ADDR = 0x3F
PWM_ADDR = 0x40
MPU_ADDR = 0x68
i2c = I2C(1, I2C.MASTER)
lcd = I2cLcd(i2c, LCD_ADDR, 2, 16)

pwm_not_enabled_pin = Pin("Y8", Pin.OUT_PP)
pwm_not_enabled_pin.low()

pca9685 = PCA9685(i2c)
rtc = RTC()
rtc.datetime((2020, 3, 8, 7, 21, 32, 0, 0))


class SpotServo:
    us_min_cmd = 500.0
    us_max_cmd = 2500.0
    deg_min_cmd = 0.0
    deg_max_cmd = 180.0

    us_per_180_degs = us_max_cmd - us_min_cmd
    us_per_degree = us_per_180_degs / 180.0

    freq_hz = 50.0
    period_us = (1.0 / freq_hz) * 1e6

    def __init__(self, min_us, center_us, max_us):
        self.min_us = min_us
        self.center_us = center_us
        self.max_us = max_us

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
        target_us = SpotServo.us_min_cmd + (float(angle_deg) * SpotServo.us_per_degree)
        return SpotServo.get_12_bit_duty_cycle_for_us(target_us)


pca9685.freq(SpotServo.freq_hz)
pca9685.duty(0, SpotServo.get_12_bit_duty_cycle_for_angle(90))
pca9685.duty(2, SpotServo.get_12_bit_duty_cycle_for_angle(90))

min_sweep_deg = 85
max_sweep_deg = 95
going_up = True
next_loop_millis = 0

position_deg = 90
while True:
    if going_up:
        position_deg += 1
        pass
    else:
        position_deg -= 1
        pass

    # add 1.8mm to the shoulder that had 3mm taken off of it
    pca9685.duty(1, SpotServo.get_12_bit_duty_cycle_for_angle(position_deg))
    lcd.move_to(0, 0)
    lcd.putstr("Loops:{:10}12-bit:{:9}".format(position_deg, position_deg))

    if position_deg >= max_sweep_deg:
        going_up = False
    elif position_deg <= min_sweep_deg:
        going_up = True
    while millis() < next_loop_millis:
        pass
    next_loop_millis = millis() + 100
