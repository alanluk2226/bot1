"""Move only the foot servos, lifting each dangling foot. Limb servos stay at 90 degrees."""

import sys

sys.path.insert(0, r"C:\Users\user\Desktop\bot1")
from fly_brain import upload_and_run, read_until

# Knee index, lift angle, name. 90 hangs the foot on the floor. Leaving 90 raises the foot.
FEET = (
    (3, 145, "front-left"),
    (2, 35, "front-right"),
    (7, 145, "rear-left"),
    (6, 145, "rear-right"),
)


def send(ser, ms, pose):
    line = "m %d %s\n" % (ms, " ".join(str(int(v)) for v in pose))
    ser.write(line.encode("utf-8"))
    reply = read_until(ser, "ok", 8)
    if "ok" not in reply:
        print("NO_REPLY", reply[-120:])
        return False
    return True


def main():
    ser = upload_and_run()
    boot = read_until(ser, "READY", 8)
    print(boot[-160:])
    if "READY" not in boot:
        ser.close()
        raise SystemExit("not ready")
    pose = [90] * 8
    send(ser, 400, pose)
    for knee, angle, name in FEET:
        print(name, "up")
        pose[knee] = angle
        send(ser, 700, pose)
        send(ser, 900, pose)
        print(name, "down")
        pose[knee] = 90
        send(ser, 700, pose)
    ser.close()
    print("DONE")


if __name__ == "__main__":
    main()
