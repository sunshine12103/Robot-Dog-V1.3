from pyb import I2C, delay, millis, RTC
from pyb_i2c_lcd import I2cLcd
from pca9685 import PCA9685

LCD_ADDR = 0x3F
PWM_ADDR = 0x40
i2c = I2C(1, I2C.MASTER)
lcd = I2cLcd(i2c, LCD_ADDR, 2, 16)

pca9685 = PCA9685(i2c)
rtc = RTC()
rtc.datetime((2020, 3, 8, 7, 21, 32, 0, 0))

while True:
    pca9685.freq(50)
    pca9685.duty(0, 2048)
    for i in range(10000):
        lcd.clear()
        lcd.move_to(0, 0)
        lcd.putstr("Loops:{:10}RTC: {:5}:{:2}:{:2}".format(i, *rtc.datetime()[4:7]))
        delay(100)
