import socket
import struct
import time

import os
import atexit
import subprocess

previous_network = None

def setup_network():
    global previous_network
    print("🚀 Starting dedicated telemetry hotspot (kxkT14s)...")
    
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
    print("📶 Hotspot active! (Internet is temporarily paused)")

def restore_network():
    if previous_network:
        print(f"\n🌐 Restoring previous internet connection ({previous_network})...")
        os.system(f"nmcli connection up '{previous_network}' >/dev/null 2>&1")
        print("✅ Internet restored.")
    else:
        print("\n🌐 No previous Wi-Fi connection detected. Turning off Hotspot...")
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

print(f"\n🎧 Listening for telemetry UDP packets on port {UDP_PORT}...")


# Format string for unpacking the C-struct (236 or 240 bytes)
# <  = Little Endian
# B  = uint8_t (kind)
# 3x = 3 bytes padding
# 6I = 6x uint32_t (ms, dt, exec, esekf, missed, dropped)
# 36f = 36x float (accel, gyro, att, vel, pos, biases, cmds, etc.)
# 4H = 4x uint16_t (PWM values)
# ?  = bool (gpsFix)
# B  = uint8_t (sats)
# B  = uint8_t (fixType)
# x  = 1 byte padding
# 3f = 3x float (hAcc, vAcc, sAcc)
# ?  = bool (originSet)
# 3x = 3 bytes padding
# I  = uint32_t (iTowMs)
# 2d = 2x double (lat, lon)  -- note: some compilers might add 4 bytes padding before this, adjust if unpacking fails
# 4f = 4x float (alt, velNed)
struct_format = "< B 3x 6I 36f 4H ? B B x 3f ? 3x I 2d 4f"
expected_size = struct.calcsize(struct_format)

last_print = time.time()

while True:
    try:
        data, addr = sock.recvfrom(1024)
        
        if len(data) == expected_size:
            unpacked = struct.unpack(struct_format, data)
            
            # Unpack a few key values to display
            # unpacked[0] is kind
            # unpacked[1] is ms
            timestamp = unpacked[1] / 1000.0
            roll_deg = unpacked[13]
            pitch_deg = unpacked[14]
            yaw_deg = unpacked[15]
            left_pwm = unpacked[43]
            right_pwm = unpacked[44]
            gps_fix = unpacked[47]
            gps_sats = unpacked[48]
            
            # Only print at 2Hz to avoid flooding the console
            if time.time() - last_print > 0.5:
                print(f"[{timestamp:.1f}s] Roll: {roll_deg:6.1f} | Pitch: {pitch_deg:6.1f} | "
                      f"Yaw: {yaw_deg:6.1f} | PWM (L/R): {left_pwm}/{right_pwm} | "
                      f"GPS: {'FIX' if gps_fix else 'NO-FIX'} ({gps_sats} sats)")
                last_print = time.time()
                
        elif len(data) == expected_size + 4:
            # If there's 4 extra bytes of padding before the doubles
            struct_format_padded = "< B 3x 6I 36f 4H ? B B x 3f ? 3x I 4x 2d 4f"
            unpacked = struct.unpack(struct_format_padded, data)
            timestamp = unpacked[1] / 1000.0
            print(f"[{timestamp:.1f}s] Received data (Note: 8-byte double alignment active)")
            last_print = time.time()
            expected_size += 4
            struct_format = struct_format_padded
        else:
            print(f"Received {len(data)} bytes from {addr}, expected {expected_size} bytes. Struct format may need tweaking!")
            
    except BlockingIOError:
        time.sleep(0.01)
    except KeyboardInterrupt:
        print("\\nStopped listening.")
        break
