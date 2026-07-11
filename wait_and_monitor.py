import serial
import time
import sys

print("Waiting for port...")
for _ in range(20):
    try:
        s = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        print("Connected!")
        break
    except:
        time.sleep(0.5)
else:
    print("Failed to connect.")
    sys.exit(1)

start = time.time()
while time.time() - start < 5:
    try:
        line = s.readline()
        if line:
            print(line.decode('utf-8', errors='replace'), end='')
    except Exception as e:
        pass
