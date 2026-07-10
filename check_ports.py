import serial
import time
for port in ['/dev/ttyS4', '/dev/ttyS5']:
    try:
        s = serial.Serial(port, 115200, timeout=1)
        print(f"Opened {port}")
        s.write(b'\n')
        res = s.read(100)
        print(f"Read {len(res)} bytes from {port}: {res}")
        s.close()
    except Exception as e:
        print(f"Error on {port}: {e}")
