from machine import Pin, PWM
import time

pwm = PWM(Pin(16), freq=50)

def angle(degrees):
    pwm.duty(int(degrees * 102 / 180 + 26))

angle(90)
time.sleep(0.5)
angle(115)
time.sleep(0.6)
angle(90)
time.sleep(0.5)
pwm.deinit()
print("one servo done")
