import serial
import time
port = '/dev/ttyACM1'
print(f"Resetting {port}...")
try:
    ser = serial.Serial(port, 1200)
    ser.dtr = False
    time.sleep(0.5)
    ser.dtr = True
    ser.close()
    print("Done. It should reboot now.")
except Exception as e:
    print("Error:", e)
