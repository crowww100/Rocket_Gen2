import Adafruit_PCA9685
from time import sleep
pwm = Adafruit_PCA9685.PCA9685(address=0x40)
pwm.set_pwm_freq(50)
pwm.set_pwm(9, 0, 200)
sleep(1)
pwm.set_pwm(9, 0, 270)
sleep(1)
pwm.set_pwm(9, 0, 0)

