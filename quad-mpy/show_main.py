"""Kit motions: jump, dance, and wave. The servos stay powered when the show ends."""

import utime
from quad import Quad


def main():
    robot = Quad()
    robot.init(12, 16, 25, 18, 13, 17, 26, 19)
    robot.setTrims(0, 0, 0, 0, 0, 0, 0, 0)
    robot._moveServos(400, [90] * 8)
    print("JUMP")
    robot.scared()
    robot._moveServos(500, [90] * 8)
    print("DANCE")
    robot.dance(steps=2, t=1600)
    robot._moveServos(500, [90] * 8)
    print("HELLO")
    robot.hello()
    print("SHOW_DONE")
    while True:
        utime.sleep_ms(1000)


main()
