"""Upload the kit jump, dance, and wave, then return to command mode."""

import sys

sys.path.insert(0, r"C:\Users\user\Desktop\bot1")
from fly_brain import upload_and_run, read_until


def upload_source(path):
    import time
    import serial

    src = open(path, encoding="utf-8").read()
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


def main():
    ser = upload_source(r"C:\Users\user\Desktop\bot1\quad-mpy\show_main.py")
    log = read_until(ser, "SHOW_DONE", 25)
    print(log[-500:])
    ser.close()
    if "SHOW_DONE" not in log:
        raise SystemExit("show did not finish")
    ser = upload_and_run()
    boot = read_until(ser, "READY", 8)
    print(boot[-160:])
    ser.close()
    print("RESTORED")


if __name__ == "__main__":
    main()
