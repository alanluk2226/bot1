"""接收電腦大腦的指令：forward、left、right、back、stop。"""

import sys
import math
import utime
from quad import Quad

HIP = 30
KNEE = 50

LEGS = (
    (0, 1, 2, -1),
    (1, 1, 3, 1),
    (4, -1, 6, 1),
    (5, -1, 7, 1),
)

PHASES = {
    "forward": (0.0, 0.0, 0.5, 0.5),
    "back": (0.5, 0.5, 0.0, 0.0),
    "right": (0.5, 0.0, 0.0, 0.5),
    "left": (0.0, 0.5, 0.5, 0.0),
}


def _smooth(u):
    return 0.5 - 0.5 * math.cos(math.pi * u)


def _leg(phase, hip_sign, knee_sign):
    phase = phase % 1.0
    if phase < 0.18:
        u = _smooth(phase / 0.18)
        knee_u = u
        hip_u = -1.0
    elif phase < 0.42:
        u = _smooth((phase - 0.18) / 0.24)
        knee_u = 1.0
        hip_u = -1.0 + 2.0 * u
    elif phase < 0.56:
        u = _smooth((phase - 0.42) / 0.14)
        knee_u = 1.0 - u
        hip_u = 1.0
    else:
        u = _smooth((phase - 0.56) / 0.44)
        knee_u = 0.0
        hip_u = 1.0 - 2.0 * u
    hip = 90 + hip_sign * HIP * hip_u
    knee = 90 + knee_sign * KNEE * knee_u
    return hip, knee


def go(robot, mode, cycles=1, period=2400):
    phases = PHASES[mode]
    robot.attachServos()
    robot.setRestState(False)
    started = utime.ticks_ms()
    total = period * cycles
    while utime.ticks_diff(utime.ticks_ms(), started) < total:
        elapsed = utime.ticks_diff(utime.ticks_ms(), started)
        ramp = 1.0 if elapsed > 250 else elapsed / 250.0
        base = (elapsed % period) / period
        pose = [90] * 8
        for n in range(4):
            hip_i, hip_s, knee_i, knee_s = LEGS[n]
            hip, knee = _leg(base + phases[n], hip_s, knee_s)
            pose[hip_i] = 90 + (hip - 90) * ramp
            pose[knee_i] = 90 + (knee - 90) * ramp
        for i in range(8):
            robot._servo[i].SetPosition(int(pose[i]))
            robot._servo_position[i] = pose[i]
        utime.sleep_ms(25)


def main():
    robot = Quad()
    robot.init(12, 16, 25, 18, 13, 17, 26, 19)
    robot.setTrims(0, 0, 0, 0, 0, 0, 0, 0)
    robot._moveServos(400, [90] * 8)
    print("READY")
    while True:
        line = sys.stdin.readline()
        if not line:
            utime.sleep_ms(20)
            continue
        cmd = line.strip()
        if cmd in PHASES:
            go(robot, cmd)
            print("ok")
        elif cmd.startswith("m "):
            parts = cmd.split()
            ms = int(parts[1])
            targets = [int(x) for x in parts[2:10]]
            robot._moveServos(ms, targets)
            print("ok")
        elif cmd == "stop":
            robot._moveServos(400, [90] * 8)
            print("ok")
        else:
            print("err")


main()
