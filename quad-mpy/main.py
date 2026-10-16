"""Command server for the body. The phone sends forward, back, left, right, stop, and auto over Bluetooth."""

import sys
import math
import utime
import network
import select
import socket
from machine import Pin
from quad import Quad

try:
    import bluetooth
except ImportError:
    bluetooth = None

try:
    import wifi_config
except ImportError:
    wifi_config = None

# GPIO 2 is sampled at boot, so the blue LED stays off until the program is running.
led = Pin(2, Pin.OUT)
led.value(0)

HIP = 22
KNEE = 38
# Feet rest slightly behind center so the rear feet do not tuck under the body and tip it forward.
BIAS = 12

LEGS = (
    (0, 1, 2, -1),
    (1, -1, 3, -1),
    (4, -1, 6, 1),
    (5, -1, 7, 1),
)

# Front-right, front-left, rear-right, rear-left. One foot lifts at a time: front-left, rear-right, front-right, rear-left.
PHASES = (0.5, 0.0, 0.25, 0.75)
HIP_DIR = {
    "forward": (1, 1, 1, 1),
    "back": (-1, -1, -1, -1),
    "right": (-1, 1, -1, 1),
    "left": (1, -1, 1, -1),
}


def _smooth(u):
    return 0.5 - 0.5 * math.cos(math.pi * u)


def _leg(phase, hip_sign, knee_sign, hip_dir):
    phase = phase % 1.0
    if phase < 0.08:
        u = _smooth(phase / 0.08)
        knee_u = u
        hip_u = -1.0
    elif phase < 0.18:
        u = _smooth((phase - 0.08) / 0.10)
        knee_u = 1.0
        hip_u = -1.0 + 2.0 * u
    elif phase < 0.25:
        u = _smooth((phase - 0.18) / 0.07)
        knee_u = 1.0 - u
        hip_u = 1.0
    else:
        u = _smooth((phase - 0.25) / 0.75)
        knee_u = 0.0
        hip_u = 1.0 - 2.0 * u
    hip = 90 + hip_sign * hip_dir * (HIP * hip_u - BIAS)
    knee = 90 + knee_sign * KNEE * knee_u
    return hip, knee


def go(robot, mode, cycles=1, period=1600):
    phases = PHASES
    hip_dirs = HIP_DIR[mode]
    robot.attachServos()
    robot.setRestState(False)
    started = utime.ticks_ms()
    total = period * cycles
    while utime.ticks_diff(utime.ticks_ms(), started) < total:
        elapsed = utime.ticks_diff(utime.ticks_ms(), started)
        ramp = 1.0 if elapsed > 350 else elapsed / 350.0
        base = (elapsed % period) / period
        pose = [90] * 8
        for n in range(4):
            hip_i, hip_s, knee_i, knee_s = LEGS[n]
            hip, knee = _leg(base + phases[n], hip_s, knee_s, hip_dirs[n])
            pose[hip_i] = 90 + (hip - 90) * ramp
            pose[knee_i] = 90 + (knee - 90) * ramp
        for i in range(8):
            robot._servo[i].SetPosition(int(pose[i]))
            robot._servo_position[i] = pose[i]
        utime.sleep_ms(25)


HIP = 28
KNEE = 36
FLY_LEGS = (
    (0, 1, 2, -1, "right"),
    (1, -1, 3, -1, "left"),
    (4, -1, 6, 1, "right"),
    (5, -1, 7, 1, "left"),
)
FLY_PHASE = (0.5, 0.0, 0.0, 0.5)
fly = {"on": False, "heading": 0.0, "goal": 0.0, "straight": 0, "frame": 0, "error": 0.0}


def _fly_leg(phase, hip_sign, knee_sign, hip_dir):
    phase = phase % 1.0
    if phase < 0.18:
        u = _smooth(phase / 0.18)
        knee_u = u
        hip_u = -1.0
    elif phase < 0.42:
        u = _smooth((phase - 0.18) / 0.24)
        knee_u = 1.0
        hip_u = -1.0 + 2.0 * u
    elif phase < 0.56:
        u = _smooth((phase - 0.42) / 0.14)
        knee_u = 1.0 - u
        hip_u = 1.0
    else:
        u = _smooth((phase - 0.56) / 0.44)
        knee_u = 0.0
        hip_u = 1.0 - 2.0 * u
    hip = 90 + hip_sign * hip_dir * HIP * hip_u
    knee = 90 + knee_sign * KNEE * knee_u
    return hip, knee


def _fly_dirs(error):
    mix = min(0.4, abs(error) / 120.0)
    weaken = "right" if error > 0 else "left"
    dirs = []
    for item in FLY_LEGS:
        dirs.append(1.0 - mix if item[4] == weaken else 1.0)
    return dirs


def _fly_pose(phase, hip_dirs):
    pose = [90] * 8
    nudge = hip_dirs[0] - hip_dirs[1]
    for n, leg in enumerate(FLY_LEGS):
        hip_i, hip_s, knee_i, knee_s, side = leg
        shift = -0.05 * nudge if side == "right" else 0.05 * nudge
        hip, knee = _fly_leg(phase + FLY_PHASE[n] + shift, hip_s, knee_s, hip_dirs[n])
        pose[hip_i] = max(0, min(180, int(round(hip))))
        pose[knee_i] = max(0, min(180, int(round(knee))))
    return pose


def _fly_arm(direction):
    fly["on"] = True
    fly["dir"] = direction
    fly["heading"] = 0.0
    fly["goal"] = 0.0
    fly["straight"] = 0
    fly["error"] = 0.0


def _fly_stop(robot):
    fly["on"] = False
    robot._moveServos(400, [90] * 8)


def _pace_ms(error):
    # The usual walk is 420 ms. A larger compass error slows that same gait into a jog, down to 740 ms.
    err = abs(error)
    if err < 8:
        return 420
    mix = min(1.0, (err - 8.0) / 32.0)
    return int(420 + mix * 320)


def _curve_step(robot, error, period=None):
    # Same waveform as straight walking. A turn only shortens the inside hips. It does not switch to a separate turn gait.
    if period is None:
        period = _pace_ms(error)
    x_amp = 15
    z_amp = 15
    mix = 0.0 if abs(error) < 8 else min(0.35, abs(error) / 140.0)
    right = 1.0 - mix if error > 0 else 1.0
    left = 1.0 - mix if error < 0 else 1.0
    amplitude = [
        x_amp * right, x_amp * left, z_amp, z_amp,
        x_amp * right, x_amp * left, z_amp, z_amp,
    ]
    offset = [4, -4, -15, 15, -16, 16, 15, -15]
    phase = [180, 180, 90, 90, 0, 0, 90, 90]
    robot._execute(amplitude, offset, [period] * 8, phase, 1)


def _fly_frame(robot):
    direction = fly.get("dir")
    if direction == "back":
        robot.forward(steps=1, t=420)
        return
    if direction != "auto":
        robot.backward(steps=1, t=420)
        return
    error = (fly["goal"] - fly["heading"] + 180.0) % 360.0 - 180.0
    fly["error"] = error
    if error > 8:
        fly["heading"] = (fly["heading"] + 7.0) % 360.0
    elif error < -8:
        fly["heading"] = (fly["heading"] - 7.0) % 360.0
    _curve_step(robot, error)
    if abs(error) < 8:
        fly["straight"] += 1
    else:
        fly["straight"] = 0
    if fly["straight"] >= 4:
        fly["goal"] = (fly["heading"] + 40.0) % 360.0
        fly["straight"] = 0
        print("turn")


def handle(robot, cmd):
    if cmd == "walk":
        _fly_arm("forward")
        return "ok"
    if cmd == "auto":
        _fly_arm("auto")
        return "ok"
    if cmd == "reverse":
        _fly_arm("back")
        return "ok"
    if cmd == "stop":
        _fly_stop(robot)
        return "ok"
    if fly["on"]:
        fly["on"] = False
    if cmd == "forward":
        robot.backward(steps=1, t=420)
        return "ok"
    if cmd == "back":
        robot.forward(steps=1, t=420)
        return "ok"
    if cmd == "left":
        robot.turn_L(steps=1, t=420)
        return "ok"
    if cmd == "right":
        robot.turn_R(steps=1, t=420)
        return "ok"
    if cmd.startswith("bias "):
        parts = cmd.split()
        period = int(parts[2]) if len(parts) > 2 else None
        _curve_step(robot, float(parts[1]), period)
        return "ok"
    if cmd.startswith("m "):
        parts = cmd.split()
        ms = int(parts[1])
        targets = [int(x) for x in parts[2:10]]
        robot._moveServos(ms, targets)
        return "ok"
    if cmd == "hello":
        _hello(robot)
        return "ok"
    return "err"


def _hello(robot):
    lifted = [90, 90, 90, 160, 90, 90, 150, 90]
    wave_a = [90, 120, 90, 160, 70, 90, 150, 90]
    wave_b = [90, 60, 90, 160, 110, 90, 150, 90]
    robot._moveServos(500, lifted)
    for _ in range(3):
        robot._moveServos(280, wave_a)
        robot._moveServos(280, wave_b)
    robot._moveServos(400, [90] * 8)


def _networks():
    nets = []
    ssid = getattr(wifi_config, "SSID", "")
    if ssid:
        nets.append((ssid, getattr(wifi_config, "PASSWORD", "")))
    hot = getattr(wifi_config, "HOTSPOT_SSID", "")
    if hot:
        nets.append((hot, getattr(wifi_config, "HOTSPOT_PASSWORD", "")))
    return nets


def join_wifi():
    if wifi_config is None:
        print("WIFI_SKIP")
        return None
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    try:
        network.hostname("bot1")
    except Exception:
        try:
            wlan.config(dhcp_hostname="bot1")
        except Exception:
            pass
    if wlan.isconnected():
        ip = wlan.ifconfig()[0]
        print("WIFI", ip)
        return ip
    for ssid, password in _networks():
        print("TRY", ssid)
        wlan.connect(ssid, password)
        started = utime.ticks_ms()
        while not wlan.isconnected():
            if utime.ticks_diff(utime.ticks_ms(), started) > 12000:
                break
            utime.sleep_ms(200)
        if wlan.isconnected():
            ip = wlan.ifconfig()[0]
            print("WIFI", ip)
            return ip
        try:
            wlan.disconnect()
        except Exception:
            pass
    print("WIFI_FAIL")
    return None


PHONE_PAGE = """<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>bot1</title>
<style>body{font-family:sans-serif;background:#07080d;color:#e8edf5;margin:0;padding:16px}button{font:16px sans-serif;padding:12px;border-radius:8px;border:1px solid #2a3142;background:#12151e;color:#e8edf5;width:100%;box-sizing:border-box;min-height:52px;margin-top:8px}.g{display:grid;grid-template-columns:1fr 1fr;gap:8px}#s{color:#8b95a8}</style></head>
<body><h1>bot1</h1><p>Forward, Back, Left, and Right each take one step. Auto keeps walking until Stop.</p>
<button onclick="go('auto')">Auto</button>
<div class="g"><button onclick="go('left')">Left</button><button onclick="go('right')">Right</button></div>
<button onclick="go('forward')">Forward</button>
<button onclick="go('back')">Back</button>
<button onclick="go('stop')">Stop</button>
<p id="s"></p>
<script>function go(c){var s=document.getElementById("s");s.textContent="sending";fetch("/cmd?c="+c).then(function(r){return r.text()}).then(function(t){s.textContent=t!=="ok"?"no reply":(c==="auto"?"walking":(c==="stop"?"standing":"ok"))}).catch(function(){s.textContent="no reply"})}</script>
</body></html>"""

ALLOWED = ("forward", "back", "left", "right", "stop", "hello", "walk", "reverse", "auto")

# Nordic UART. The iPhone writes a command; the robot notifies the reply.
_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E") if bluetooth else None
_UART_TX = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E") if bluetooth else None
_UART_RX = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E") if bluetooth else None
_BLE_PAD = {
    b"1": "forward",
    b"2": "back",
    b"3": "left",
    b"4": "right",
    b"5": "auto",
    b"6": "stop",
}
_ble = None
_ble_tx = None
_ble_rx_handle = None
_ble_conn = None
_ble_buf = bytearray(80)
_ble_n = 0
_ble_flag = False
_ble_adv_at = 0


def _adv_field(kind, value):
    return bytes((len(value) + 1, kind)) + value


def _ble_command(raw):
    # Adafruit Bluefruit control pad: '!' 'B' <button 1-8> <'1' pressed or '0' released> checksum.
    if len(raw) >= 4 and raw[0:2] == b"!B" and raw[3:4] == b"1":
        return _BLE_PAD.get(raw[2:3])
    text = raw.decode("utf-8", "replace").strip().lower().split("\n", 1)[0].strip()
    if text in ALLOWED:
        return text
    return None


def _ble_irq(event, data):
    global _ble_conn, _ble_n, _ble_flag
    if event == 1:
        _ble_conn = data[0]
        return
    if event == 2:
        _ble_conn = None
        _ble_advertise()
        return
    if event == 3:
        _conn, attr = data
        if attr != _ble_rx_handle:
            return
        value = _ble.gatts_read(_ble_rx_handle)
        n = len(value)
        if n > len(_ble_buf):
            n = len(_ble_buf)
        _ble_buf[:n] = value[:n]
        _ble_n = n
        _ble_flag = True


def _ble_advertise():
    # The name has to sit in the main advertisement. iOS shows "Unknown" when it is only in the scan response.
    global _ble_adv_at
    if _ble is None:
        return
    payload = _adv_field(0x01, b"\x06") + _adv_field(0x09, b"bot1")
    try:
        uid = bytes(_UART_UUID)
        if len(uid) == 16 and len(payload) + len(uid) + 2 <= 31:
            payload += _adv_field(0x07, uid)
    except Exception:
        pass
    _ble.gap_advertise(100000, payload)
    _ble_adv_at = utime.ticks_ms()


def start_ble():
    global _ble, _ble_tx, _ble_rx_handle
    if bluetooth is None:
        print("BLE_SKIP")
        return
    try:
        ble = bluetooth.BLE()
        try:
            ble.active(True)
        except OSError:
            ble.active(False)
            utime.sleep_ms(300)
            ble.active(True)
        ble.config(gap_name="bot1")
        ble.irq(_ble_irq)
        ((_ble_tx, _ble_rx_handle),) = ble.gatts_register_services((
            (
                _UART_UUID,
                (
                    (_UART_TX, bluetooth.FLAG_NOTIFY),
                    (_UART_RX, bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE),
                ),
            ),
        ))
        _ble = ble
        _ble_advertise()
        print("BLE bot1")
        try:
            from machine import RTC
            RTC().memory(b"")
        except Exception:
            pass
    except Exception as exc:
        print("BLE_ERR", exc)
        # A soft reboot leaves the controller stuck. One hard reset clears it.
        try:
            from machine import RTC, reset
            rtc = RTC()
            if rtc.memory() != b"ble-only":
                rtc.memory(b"ble-only")
                reset()
        except Exception:
            pass


def _ble_keepalive():
    if _ble is None or _ble_conn is not None:
        return
    if utime.ticks_diff(utime.ticks_ms(), _ble_adv_at) > 5000:
        _ble_advertise()


def _ble_poll(robot):
    global _ble_flag
    _ble_keepalive()
    if not _ble_flag:
        return
    _ble_flag = False
    cmd = _ble_command(bytes(_ble_buf[:_ble_n]))
    if not cmd:
        return
    try:
        reply = handle(robot, cmd)
    except Exception:
        reply = "err"
    if _ble is not None and _ble_conn is not None:
        try:
            _ble.gatts_notify(_ble_conn, _ble_tx, (reply + "\n").encode())
        except OSError:
            pass


def _unquote(text):
    text = text.replace("+", " ")
    out = ""
    i = 0
    while i < len(text):
        if text[i] == "%" and i + 2 < len(text):
            try:
                out += chr(int(text[i + 1:i + 3], 16))
                i += 3
                continue
            except ValueError:
                pass
        out += text[i]
        i += 1
    return out


def _query_cmd(target):
    if "?" not in target:
        return ""
    for pair in target.split("?", 1)[1].split("&"):
        if pair.startswith("c="):
            return _unquote(pair[2:]).strip()
    return ""


def _http_send(client, code, body, content_type):
    payload = body.encode("utf-8")
    head = (
        "HTTP/1.1 %s\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Access-Control-Allow-Methods: GET, OPTIONS\r\n"
        "Access-Control-Allow-Private-Network: true\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n"
    ) % (code, content_type, len(payload))
    client.send(head.encode("utf-8") + payload)


def _tcp_client(robot, client):
    client.settimeout(3)
    data = b""
    try:
        while b"\n" not in data and len(data) < 160:
            chunk = client.recv(128)
            if not chunk:
                break
            data += chunk
    except OSError:
        data = b""
    reply = "err"
    if data:
        line = data.decode("utf-8", "replace").strip().split("\n")[0]
        try:
            client.settimeout(None)
            reply = handle(robot, line)
        except Exception:
            reply = "err"
    try:
        client.send((reply + "\n").encode("utf-8"))
    except OSError:
        pass
    client.close()


def _http_client(robot, client):
    client.settimeout(3)
    data = b""
    try:
        while b"\r\n\r\n" not in data and len(data) < 800:
            chunk = client.recv(256)
            if not chunk:
                break
            data += chunk
    except OSError:
        data = b""
    try:
        line = data.decode("utf-8", "replace").split("\r\n", 1)[0]
        parts = line.split(" ")
        method = parts[0] if parts else ""
        target = parts[1] if len(parts) > 1 else "/"
        path = target.split("?", 1)[0]
        if method == "OPTIONS":
            _http_send(client, "204 No Content", "", "text/plain")
        elif path == "/cmd":
            cmd = _query_cmd(target)
            if cmd not in ALLOWED:
                _http_send(client, "200 OK", "err", "text/plain")
            else:
                client.settimeout(None)
                try:
                    reply = handle(robot, cmd)
                except Exception:
                    reply = "err"
                _http_send(client, "200 OK", reply, "text/plain")
        elif path == "/":
            _http_send(client, "200 OK", PHONE_PAGE, "text/html")
        else:
            _http_send(client, "404 Not Found", "err", "text/plain")
    except Exception:
        pass
    client.close()


def _listen(port):
    last = None
    for _ in range(10):
        server = socket.socket()
        try:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("0.0.0.0", port))
            server.listen(2)
            return server
        except OSError as exc:
            last = exc
            server.close()
            utime.sleep_ms(400)
    print("BIND_ERR", port, last)
    raise last


def _led():
    if fly["on"] and fly.get("dir") == "auto":
        led.value(0 if led.value() else 1)
    else:
        led.value(1)


def serve(robot, ip, tcp, http):
    print("LISTEN", ip, wifi_config.PORT)
    print("HTTP", ip, getattr(wifi_config, "HTTP_PORT", 80))
    while True:
        _ble_poll(robot)
        ready, _, _ = select.select([tcp, http], [], [], 0 if fly["on"] else 0.2)
        for sock in ready:
            try:
                client, _addr = sock.accept()
            except OSError:
                continue
            if sock is http:
                _http_client(robot, client)
            else:
                _tcp_client(robot, client)
        _led()
        if fly["on"]:
            try:
                _fly_frame(robot)
            except Exception as exc:
                fly["on"] = False
                print("WALK_ERR", exc)


def main():
    # Bluetooth only. Wi-Fi and Bluetooth together run out of memory on this board.
    start_ble()
    robot = Quad()
    robot.init(12, 16, 25, 18, 13, 17, 26, 19)
    robot.setTrims(0, 0, 0, 0, 0, 0, 0, 0)
    robot._moveServos(400, [90] * 8)
    print("READY")
    if _ble is not None:
        led.value(1)
    while True:
        _ble_poll(robot)
        utime.sleep_ms(40)


main()
