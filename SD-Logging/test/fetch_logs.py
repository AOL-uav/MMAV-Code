import serial
import time
import os

# Configuration
PORT = '/dev/ttyACM0'
BAUD = 115200
OUTPUT_DIR = '/home/kaikeller/Purdue/AOL/MAV/SD-Logging/outputs'

def fetch_log():
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=2)
        time.sleep(2) # Wait for connection
        
        print(f"Connecting to {PORT}...")
        
        # Send 'r' to read the log
        ser.write(b'r\n')
        print("Sent 'r' (Read Log) command. Downloading data...")
        
        # Read the filename first
        filename = "downloaded_log.csv"
        
        # Open file to write
        with open(os.path.join(OUTPUT_DIR, filename), 'w') as f:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore')
                if not line or "# End of log." in line:
                    break
                if line.startswith("#"):
                    print(line.strip())
                    continue
                f.write(line)
        
        ser.close()
        print(f"Done! File saved to {OUTPUT_DIR}/{filename}")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    fetch_log()
