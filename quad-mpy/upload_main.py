import serial
import time

src = open(r"C:\Users\user\Desktop\bot1\quad-mpy\main.py", encoding="utf-8").read()
ser = serial.Serial()
ser.port = "COM4"
ser.baudrate = 115200
ser.timeout = 0.3
ser.dtr = False
ser.rts = False
ser.open()
time.sleep(0.2)
for _ in range(25):
    ser.write(b"\x03")
    time.sleep(0.05)
time.sleep(0.3)
ser.write(b"\x01")
time.sleep(0.3)
ser.read(4096)
payload = "f=open('main.py','w');f.write(" + repr(src) + ");f.close()\r\n"
ser.write(payload.encode("utf-8"))
ser.write(b"\x04")
time.sleep(0.8)
write_out = ser.read(4096).decode("utf-8", "replace")
print("WRITE", write_out[-300:])
ser.write(b"\x02")
time.sleep(0.2)
ser.write(b"\x04")
end = time.time() + 40
buf = b""
while time.time() < end:
    buf += ser.read(4096)
ser.close()
text = buf.decode("utf-8", "replace")
print(text[-800:])
print("HOLD" if "holding" in text else "NOHOLD")
print("ERR" if "Traceback" in text else "NOERR")
