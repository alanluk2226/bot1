"""Short commands: py -3.11 go.py forward"""

import socket
import sys

IP_FILE = r"C:\Users\user\Desktop\bot1\data\robot_ip.txt"
PORT = 8765


def once(ip, command):
    sock = socket.create_connection((ip, PORT), 8)
    sock.settimeout(15)
    sock.sendall((command + "\n").encode("utf-8"))
    reply = sock.recv(32).decode("utf-8", "replace").strip()
    sock.close()
    return reply


def hello(ip):
    # Wave the front-left foot (hip 1, knee 3) and the rear-right foot (hip 4, knee 6). The other feet stay down.
    lifted = [90, 90, 90, 160, 90, 90, 150, 90]
    wave_a = [90, 120, 90, 160, 70, 90, 150, 90]
    wave_b = [90, 60, 90, 160, 110, 90, 150, 90]
    home = [90] * 8

    def pose(ms, angles):
        command = "m %d %s" % (ms, " ".join(str(v) for v in angles))
        print(once(ip, command))

    pose(500, lifted)
    for _ in range(3):
        pose(280, wave_a)
        pose(280, wave_b)
    pose(400, home)


def stand(ip):
    print("stand", once(ip, "stop"))


def main():
    command = sys.argv[1] if len(sys.argv) > 1 else "stop"
    try:
        ip = open(IP_FILE, encoding="utf-8").read().strip()
    except OSError:
        raise SystemExit("missing data/robot_ip.txt")
    if command == "hello":
        hello(ip)
        stand(ip)
        return
    times = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    for n in range(times):
        print(n + 1, once(ip, command))
    if command != "stop":
        stand(ip)


if __name__ == "__main__":
    main()
