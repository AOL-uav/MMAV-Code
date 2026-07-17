import socket
import struct
import time
import os
import atexit
import subprocess
import threading
import sys

previous_network = None
last_arduino_addr = None

def setup_network():
    global previous_network
    print("Starting dedicated telemetry hotspot (kxkT14s)...")
    
    # Record whatever Wi-Fi network is currently active
    try:
        output = subprocess.check_output("nmcli -t -f NAME,TYPE c show --active", shell=True, text=True)
        for line in output.split('\n'):
            if '802-11-wireless' in line:
                name = line.split(':')[0]
                if name != 'Hotspot':
                    previous_network = name
                    break
    except Exception:
        pass

    os.system("nmcli connection up Hotspot 2>/dev/null || nmcli dev wifi hotspot ssid kxkT14s password 'Mika12345.' >/dev/null")
    print("Hotspot active! (Internet is temporarily paused)")

def restore_network():
    if previous_network:
        print(f"\nRestoring previous internet connection ({previous_network})...")
        os.system(f"nmcli connection up '{previous_network}' >/dev/null 2>&1")
        print("Internet restored.")
    else:
        print("\nNo previous Wi-Fi connection detected. Turning off Hotspot...")
        os.system("nmcli connection down Hotspot >/dev/null 2>&1")

atexit.register(restore_network)
setup_network()

UDP_IP = "0.0.0.0" # Listen on all interfaces
UDP_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

def input_thread():
    global last_arduino_addr
    print("\n--- COMMAND INTERFACE ACTIVE ---")
    print("Commands: 'arm', 'disarm', or 'tune p i d' (e.g. 'tune 1.5 0.2 0.0')")
    print("Type your command and press Enter (telemetry will keep scrolling above)\n")
    
    while True:
        try:
            line = sys.stdin.readline().strip().lower()
            if not line:
                continue
                
            if not last_arduino_addr:
                print(">> ERROR: Cannot send command. Waiting for first telemetry packet from Arduino to get its IP address!")
                continue

            cmd_id = 0
            vals = [0.0, 0.0, 0.0]

            if line == "arm":
                cmd_id = 1
            elif line == "disarm":
                cmd_id = 2
            elif line.startswith("tune"):
                parts = line.split()
                if len(parts) == 4:
                    cmd_id = 3
                    try:
                        vals[0] = float(parts[1])
                        vals[1] = float(parts[2])
                        vals[2] = float(parts[3])
                    except ValueError:
                        print(">> ERROR: Invalid numbers for tune command")
                        continue
                else:
                    print(">> ERROR: format is 'tune p i d'")
                    continue
            else:
                print(f">> Unknown command: {line}")
                continue

            # Pack struct UplinkCommand: < I B 3f (uint32 magic, uint8 id, float[3] values)
            data = struct.pack("< I B 3f", 0xA1B2C3D4, cmd_id, vals[0], vals[1], vals[2])
            sock.sendto(data, last_arduino_addr)
            print(f">> SENT: ID {cmd_id}, Vals {vals}")

        except Exception as e:
            print(f"Input thread error: {e}")
            break

# Start input thread
t = threading.Thread(target=input_thread, daemon=True)
t.start()

print(f"\nListening for telemetry UDP packets on port {UDP_PORT}...")

# C-struct parsing identical to receiver script
struct_format = "< B 3x 6I 36f 4H ? B B x 3f ? 3x I 2d 4f"
expected_size = struct.calcsize(struct_format)

last_print = 0
start_time = time.time()
print_warn = True

while True:
    try:
        data, addr = sock.recvfrom(1024)
        last_arduino_addr = addr # Save IP address so input_thread can reply!

        actual_format = struct_format
        if len(data) == expected_size:
            pass
        elif len(data) == expected_size + 4:
            actual_format = "< B 3x 6I 36f 4H ? B B x 3f ? 3x I 4x 2d 4f"
            if print_warn:
                print("(Note: 8-byte double alignment active)")
                print_warn = False
        else:
            continue

        unpacked = struct.unpack(actual_format, data)
        elapsed = time.time() - start_time
        
        # Throttle printing to 1Hz so the user can type in the terminal easier
        if elapsed - last_print >= 1.0:
            kind = unpacked[0]
            roll_rad = unpacked[9]
            pitch_rad = unpacked[10]
            yaw_rad = unpacked[11]
            pwmL = unpacked[43]
            pwmR = unpacked[44]
            fix = unpacked[47]
            sats = unpacked[48]

            roll_deg = roll_rad * 180.0 / 3.1415926535
            pitch_deg = pitch_rad * 180.0 / 3.1415926535
            yaw_deg = yaw_rad * 180.0 / 3.1415926535

            gps_str = f"FIX ({sats} sats)" if fix else f"NO-FIX ({sats} sats)"
            print(f"[{elapsed:4.1f}s] Roll: {roll_deg:6.1f} | Pitch: {pitch_deg:6.1f} | Yaw: {yaw_deg:6.1f} | PWM (L/R): {pwmL}/{pwmR} | GPS: {gps_str}")
            last_print = elapsed

    except BlockingIOError:
        time.sleep(0.01)
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"Error unpacking data: {e}")
        time.sleep(1)
