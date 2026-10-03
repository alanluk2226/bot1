"""
1. Test a 180-degree servo
2. Move the servo to 90 degrees
"""

from machine import Pin, PWM
import time

# Servo pin
SERVO_PIN = 15

# PWM object
pwm = PWM(Pin(SERVO_PIN), freq=50)


# Map an angle to a PWM duty
def set_angle(angle):
    # Servo angle is 0 to 180 degrees
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180

    # Pulse width is usually 0.5 ms to 2.5 ms, which is a 2.5% to 12.5% duty.
    # Map the angle onto that duty.
    duty = int((angle / 180) * (125 - 25) + 25)
    pwm.duty(duty)


if __name__ == '__main__':

    # Servo to 0 degrees
    set_angle(0)
    time.sleep(1)

    # Servo to 180 degrees
    set_angle(180)
    time.sleep(1)

    # Servo to 90 degrees
    set_angle(90)
    time.sleep(1)
