import serial
import time

try:
    print("Resetting board...")
    s = serial.Serial('/dev/ttyACM0', 1200)
    s.close()
    time.sleep(2)
except Exception as e:
    print("Reset error:", e)

try:
    s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    print("Monitoring...")
    start = time.time()
    while time.time() - start < 10:
        line = s.readline()
        if line:
            print(line.decode('utf-8', errors='replace'), end='')
except Exception as e:
    print("Monitor error:", e)
