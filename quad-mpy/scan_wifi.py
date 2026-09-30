import serial
import time

ser = serial.Serial()
ser.port = "COM4"
ser.baudrate = 115200
ser.timeout = 0.3
ser.dtr = False
ser.rts = False
ser.open()
time.sleep(0.2)
for _ in range(20):
    ser.write(b"\x03")
    time.sleep(0.05)
time.sleep(0.3)
ser.write(b"\x01")
time.sleep(0.3)
ser.read(4096)
code = """
import network
w = network.WLAN(network.STA_IF)
w.active(True)
nets = w.scan()
for n in nets:
    ssid = n[0].decode()
    rssi = n[3]
    print(ssid, rssi)
print('SCAN_DONE', len(nets))
"""
ser.write(code.encode("utf-8").replace(b"\n", b"\r\n"))
ser.write(b"\x04")
end = time.time() + 12
buf = b""
while time.time() < end:
    buf += ser.read(4096)
    if b"SCAN_DONE" in buf:
        break
ser.close()
print(buf.decode("utf-8", "replace"))
