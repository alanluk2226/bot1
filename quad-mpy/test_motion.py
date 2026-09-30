import time
from quad import Quad

robot = Quad()
robot.init(12, 16, 25, 18, 13, 17, 26, 19)
robot.setTrims(0, 0, 0, 0, 0, 0, 0, 0)
time.sleep(0.8)
robot.home()
time.sleep(0.4)
robot.up_down(steps=2, t=1400)
robot.home()
print("motion done")
