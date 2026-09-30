"""只動腳部馬達，把下垂的腿往上擺。肢體馬達保持 90 度。"""

import sys

sys.path.insert(0, r"C:\Users\user\Desktop\bot1")
from fly_brain import upload_and_run, read_until

# 膝索引、抬高角度、名稱。90 是垂下貼地，離開 90 腳端才會升高。
FEET = (
    (3, 145, "左前"),
    (2, 35, "右前"),
    (7, 145, "左後"),
    (6, 145, "右後"),
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
