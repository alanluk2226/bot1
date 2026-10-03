"""A small heading brain on the computer. It only sends forward, left, right, and stop. The legs move on the robot."""

import math
import time

import serial


class FlyBrain:
    """A small central complex: a ring of neurons holds heading, and the angle to the goal chooses a turn or a straight step."""

    def __init__(self):
        self.heading = 0.0
        self.goal = 90.0
        self.turn_step = 35.0

    def bump(self):
        cells = []
        for i in range(8):
            center = i * 45.0
            delta = (self.heading - center + 180.0) % 360.0 - 180.0
            cells.append(round(math.exp(-0.5 * (delta / 40.0) ** 2), 2))
        return cells

    def decide(self):
        error = (self.goal - self.heading + 180.0) % 360.0 - 180.0
        if error > 30:
            command = "right"
            self.heading = (self.heading + self.turn_step) % 360.0
        elif error < -30:
            command = "left"
            self.heading = (self.heading - self.turn_step) % 360.0
        else:
            command = "forward"
        return command, error


def upload_and_run():
    src = open(r"C:\Users\user\Desktop\bot1\quad-mpy\main.py", encoding="utf-8").read()
    ser = serial.Serial()
    ser.port = "COM4"
    ser.baudrate = 115200
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(0.2)
    for _ in range(20):
        ser.write(b"\x03")
        time.sleep(0.04)
    time.sleep(0.2)
    ser.write(b"\x01")
    time.sleep(0.3)
    ser.read(4096)
    payload = "f=open('main.py','w');f.write(" + repr(src) + ");f.close()\r\n"
    ser.write(payload.encode("utf-8"))
    ser.write(b"\x04")
    time.sleep(0.6)
    ser.read(1024)
    ser.write(b"\x02")
    time.sleep(0.2)
    ser.write(b"\x04")
    return ser


def read_until(ser, token, timeout):
    end = time.time() + timeout
    buf = ""
    while time.time() < end:
        buf += ser.read(4096).decode("utf-8", "replace")
        if token in buf:
            return buf
    return buf


def main():
    brain = FlyBrain()
    ser = upload_and_run()
    boot = read_until(ser, "READY", 8)
    print(boot[-400:])
    if "READY" not in boot:
        ser.close()
        raise SystemExit("robot did not become ready")

    goals = [90.0, -80.0]
    brain.goal = goals.pop(0)
    forwards = 0
    for step in range(10):
        command, error = brain.decide()
        cells = brain.bump()
        print(
            "step %d goal %.0f heading %.0f error %.0f -> %s bump %s"
            % (step + 1, brain.goal, brain.heading, error, command, cells)
        )
        ser.write((command + "\n").encode("utf-8"))
        reply = read_until(ser, "ok", 8)
        if "ok" not in reply:
            print("NO_REPLY", reply[-200:])
            break
        if command == "forward":
            forwards += 1
            if forwards >= 2 and goals:
                brain.goal = goals.pop(0)
                forwards = 0
    ser.write(b"stop\n")
    print(read_until(ser, "ok", 6)[-120:])
    ser.close()
    print("BRAIN_DONE")


if __name__ == "__main__":
    main()
