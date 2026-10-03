"""Play the eight poses of the kit walk(), one servo angle at a time."""

import sys

sys.path.insert(0, r"C:\Users\user\Desktop\bot1")
from fly_brain import FlyBrain, upload_and_run, read_until

# quad.py walk(), a=16 ao=50 b=5 c=-30 co=10. Servo order matches init:
# front-right hip, front-left hip, front-right knee, front-left knee,
# rear-right hip, rear-left hip, rear-right knee, rear-left knee.
A, AO, B, C, CO = 16, 50, 5, -30, 10


def frame(*terms):
    return [int(round(v)) for v in terms]


def official_steps():
    a, ao, b, c, co = A, AO, B, C, CO
    return (
        ("front-right down", frame(
            90 + 2.0 * a - ao, 90 - 4.0 * a + ao,
            90 + c + 5 * b, 90 - c - 4 * b,
            90 + 3.0 * a - co, 90 - 1.0 * a + co,
            90 - c - 4 * b - 10, 90 + c + 6 * b)),
        ("lift front-left", frame(
            90 + 2.3 * a - ao, 90 - 2.0 * a + ao,
            90 + c + 5 * b, 90 - c - 0 * b,
            90 + 3.3 * a - co, 90 - 1.3 * a + co,
            90 - c - 4 * b - 10, 90 + c + 6 * b)),
        ("rear-left down", frame(
            90 + 3.0 * a - ao, 90 - 1.0 * a + ao,
            90 + c + 4 * b, 90 - c - 6 * b,
            90 + 4.0 * a - co, 90 - 2.0 * a + co,
            90 - c - 4 * b - 10, 90 + c + 5 * b)),
        ("lift rear-right", frame(
            90 + 3.3 * a - ao, 90 - 1.3 * a + ao,
            90 + c + 4 * b, 90 - c - 6 * b,
            90 + 2.0 * a - co, 90 - 2.3 * a + co,
            90 - c - 0 * b - 10, 90 + c + 5 * b)),
        ("front-left down", frame(
            90 + 4.0 * a - ao, 90 - 2.0 * a + ao,
            90 + c + 4 * b, 90 - c - 5 * b,
            90 + 1.0 * a - co, 90 - 3.0 * a + co,
            90 - c - 6 * b - 10, 90 + c + 4 * b)),
        ("lift front-right", frame(
            90 + 2.0 * a - ao, 90 - 2.3 * a + ao,
            90 + c + 0 * b, 90 - c - 5 * b,
            90 + 1.3 * a - co, 90 - 3.3 * a + co,
            90 - c - 6 * b - 10, 90 + c + 4 * b)),
        ("rear-right down", frame(
            90 + 1.0 * a - ao, 90 - 3.0 * a + ao,
            90 + c + 6 * b, 90 - c - 4 * b,
            90 + 2.0 * a - co, 90 - 4.0 * a + co,
            90 - c - 5 * b - 10, 90 + c + 4 * b)),
        ("lift rear-left", frame(
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
        ms = t if name.endswith("down") else max(120, t // 3)
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
