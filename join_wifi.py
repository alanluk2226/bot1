"""Upload the body program and wait until Bluetooth is advertising as bot1."""

import socket
import sys
import time

import serial

sys.path.insert(0, r"C:\Users\user\Desktop\bot1")
from fly_brain import read_until

ROOT = r"C:\Users\user\Desktop\bot1\quad-mpy"


def _raw(ser, code, timeout=6):
    data = code.encode("utf-8")
    if not data.endswith(b"\n"):
        data += b"\n"
    for i in range(0, len(data), 48):
        ser.write(data[i:i + 48])
        time.sleep(0.02)
    ser.write(b"\x04")
    end = time.time() + timeout
    buf = b""
    while time.time() < end:
        buf += ser.read(4096)
        if buf.count(b"\x04") >= 2:
            break
        time.sleep(0.02)
    if b"Traceback" in buf:
        raise SystemExit(buf.decode("utf-8", "replace")[-400:])
    return buf


def upload(ser, name, text):
    ser.write(b"\x01")
    time.sleep(0.2)
    ser.read(8192)
    raw = text.encode("utf-8")
    _raw(ser, "f=open(%s,'wb')" % repr(name))
    for i in range(0, len(raw), 128):
        _raw(ser, "f.write(%s)" % repr(raw[i:i + 128]))
    _raw(ser, "f.close()")
    ser.write(b"\x02")
    time.sleep(0.2)
    ser.read(1024)


def main():
    main_src = open(ROOT + r"\main.py", encoding="utf-8").read()
    quad_src = open(ROOT + r"\quad.py", encoding="utf-8").read()
    wifi_src = open(ROOT + r"\wifi_config.py", encoding="utf-8").read()
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
    upload(ser, "wifi_config.py", wifi_src)
    upload(ser, "quad.py", quad_src)
    upload(ser, "main.py", main_src)
    ser.write(b"\x01")
    time.sleep(0.2)
    ser.read(2048)
    _raw(ser, "import bluetooth\ntry:\n    bluetooth.BLE().active(False)\nexcept Exception:\n    pass\n")
    ser.write(b"\x02")
    time.sleep(0.2)
    ser.write(b"\x04")
    boot = read_until(ser, "BLE bot1", 40)
    print(boot[-800:])
    ser.close()
    if "BLE bot1" not in boot:
        raise SystemExit("bluetooth did not start")
    print("BLE bot1")


if __name__ == "__main__":
    main()
