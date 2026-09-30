"""果蠅大腦照官方 walk() 的八個姿勢，逐顆下達八個馬達角度。"""

import sys

sys.path.insert(0, r"C:\Users\user\Desktop\bot1")
from fly_brain import FlyBrain, upload_and_run, read_until

# quad.py walk()，a=16 ao=50 b=5 c=-30 co=10。順序同 init：
# 右前髖、左前髖、右前膝、左前膝、右後髖、左後髖、右後膝、左後膝。
A, AO, B, C, CO = 16, 50, 5, -30, 10


def frame(*terms):
    return [int(round(v)) for v in terms]


def official_steps():
    a, ao, b, c, co = A, AO, B, C, CO
    return (
        ("右前落地", frame(
            90 + 2.0 * a - ao, 90 - 4.0 * a + ao,
            90 + c + 5 * b, 90 - c - 4 * b,
            90 + 3.0 * a - co, 90 - 1.0 * a + co,
            90 - c - 4 * b - 10, 90 + c + 6 * b)),
        ("抬起左前腳", frame(
            90 + 2.3 * a - ao, 90 - 2.0 * a + ao,
            90 + c + 5 * b, 90 - c - 0 * b,
            90 + 3.3 * a - co, 90 - 1.3 * a + co,
            90 - c - 4 * b - 10, 90 + c + 6 * b)),
        ("左後落地", frame(
            90 + 3.0 * a - ao, 90 - 1.0 * a + ao,
            90 + c + 4 * b, 90 - c - 6 * b,
            90 + 4.0 * a - co, 90 - 2.0 * a + co,
            90 - c - 4 * b - 10, 90 + c + 5 * b)),
        ("抬起右後腳", frame(
            90 + 3.3 * a - ao, 90 - 1.3 * a + ao,
            90 + c + 4 * b, 90 - c - 6 * b,
            90 + 2.0 * a - co, 90 - 2.3 * a + co,
            90 - c - 0 * b - 10, 90 + c + 5 * b)),
        ("左前落地", frame(
            90 + 4.0 * a - ao, 90 - 2.0 * a + ao,
            90 + c + 4 * b, 90 - c - 5 * b,
            90 + 1.0 * a - co, 90 - 3.0 * a + co,
            90 - c - 6 * b - 10, 90 + c + 4 * b)),
        ("抬起右前腳", frame(
            90 + 2.0 * a - ao, 90 - 2.3 * a + ao,
            90 + c + 0 * b, 90 - c - 5 * b,
            90 + 1.3 * a - co, 90 - 3.3 * a + co,
            90 - c - 6 * b - 10, 90 + c + 4 * b)),
        ("右後落地", frame(
            90 + 1.0 * a - ao, 90 - 3.0 * a + ao,
            90 + c + 6 * b, 90 - c - 4 * b,
            90 + 2.0 * a - co, 90 - 4.0 * a + co,
            90 - c - 5 * b - 10, 90 + c + 4 * b)),
        ("抬起左後腳", frame(
            90 + 1.3 * a - ao, 90 - 3.3 * a + ao,
            90 + c + 6 * b, 90 - c - 4 * b,
            90 + 2.3 * a - co, 90 - 2.0 * a + co,
            90 - c - 5 * b - 10, 90 + c + 0 * b)),
    )


def send(ser, ms, pose):
    line = "m %d %s\n" % (ms, " ".join(str(int(v)) for v in pose))
    ser.write(line.encode("utf-8"))
    reply = read_until(ser, "ok", 8)
    if "ok" not in reply:
        print("NO_REPLY", reply[-120:])
        return False
    return True


def play_forward(ser, t=800):
    for name, pose in official_steps():
        ms = t if "落地" in name else max(120, t // 3)
        print(name, pose)
        if not send(ser, ms, pose):
            return False
    return True


def main():
    brain = FlyBrain()
    brain.heading = 0.0
    brain.goal = 0.0
    ser = upload_and_run()
    boot = read_until(ser, "READY", 8)
    print(boot[-160:])
    if "READY" not in boot:
        ser.close()
        raise SystemExit("not ready")
    for n in range(2):
        command, error = brain.decide()
        print("brain", n + 1, command, "error", round(error))
        if command == "forward":
            if not play_forward(ser):
                break
    send(ser, 500, [90] * 8)
    ser.close()
    print("DONE")


if __name__ == "__main__":
    main()
