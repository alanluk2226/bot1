"""Upload the Wi-Fi settings and the body program, wait for the robot's IP, then send stop over the network so it stands."""

import socket
import sys
import time

import serial

sys.path.insert(0, r"C:\Users\user\Desktop\bot1")
from fly_brain import read_until

ROOT = r"C:\Users\user\Desktop\bot1\quad-mpy"


def upload(ser, name, text):
    ser.write(b"\x01")
    time.sleep(0.3)
    ser.read(4096)
    payload = "f=open(%s,'w');f.write(%s);f.close()\r\n" % (repr(name), repr(text))
    ser.write(payload.encode("utf-8"))
    ser.write(b"\x04")
    time.sleep(0.8)
    ser.read(2048)
    ser.write(b"\x02")
    time.sleep(0.2)


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
    ser.write(b"\x04")
    boot = read_until(ser, "WIFI ", 25)
    print(boot[-400:])
    ser.close()
    ip = ""
    for line in boot.splitlines():
        if line.startswith("WIFI "):
            ip = line.split()[1]
    if not ip or ip == "FAIL":
        raise SystemExit("wifi did not connect")
    open(r"C:\Users\user\Desktop\bot1\data\robot_ip.txt", "w", encoding="utf-8").write(ip + "\n")
    sock = socket.create_connection((ip, 8765), 5)
    sock.sendall(b"stop\n")
    reply = sock.recv(32)
    sock.close()
    print("IP", ip)
    print("REPLY", reply.decode("utf-8", "replace").strip())


if __name__ == "__main__":
    main()
