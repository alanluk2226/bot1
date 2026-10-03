"""
How many servos a USB 2.0 port can power.
"""

from machine import Pin, PWM
import time

# LED
led = Pin(2, Pin.OUT)

# Servo pins
SERVO_PINS = [15, 4, 16, 17, 5, 18, 19, 21]

# PWM object for each servo
servos = {}


def init_servos():
    # Attach every servo
    for pin in SERVO_PINS:
        try:
            pwm = PWM(Pin(pin), freq=50)
            servos[pin] = pwm
        except Exception as e:
            print(f"servo init failed, pin {pin}, error {e}")


def set_angle(angle):
    # Clamp the angle
    angle = max(0, min(180, angle))
    # Duty from the angle
    duty = int((angle / 180) * (125 - 25) + 25)

    # Move every servo together
    for servo in servos.values():
        servo.duty(duty)


def cleanup():
    # Release every PWM channel
    for servo in servos.values():
        servo.deinit()


# Attach the servos
init_servos()

try:
    while True:
        # 70 degrees
        set_angle(70)
        led.on()
        time.sleep(0.5)

        # 110 degrees
        set_angle(110)
        led.off()
        time.sleep(0.5)

        print(time.time())

except KeyboardInterrupt:
    print("stopped")
finally:
    cleanup()
