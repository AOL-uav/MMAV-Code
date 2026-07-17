import socket
import struct
import time
import os
import atexit
import subprocess
import curses

previous_network = None
last_arduino_addr = None
sock = None

def setup_network():
    global previous_network
    print("Starting dedicated telemetry hotspot (kxkT14s)...")
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
    time.sleep(1.5) # Let user read this before launching UI

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

# Setup UDP
UDP_IP = "0.0.0.0"
UDP_PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

def main(stdscr):
    global last_arduino_addr
    
    curses.curs_set(1)
    stdscr.nodelay(True)
    max_y, max_x = stdscr.getmaxyx()
    
    # Split screen: Log window on top, Command window on bottom
    log_win = curses.newwin(max_y - 2, max_x, 0, 0)
    log_win.scrollok(True)
    
    cmd_win = curses.newwin(2, max_x, max_y - 2, 0)
    
    input_str = ""
    
    struct_format = "< B 3x 6I 36f 4H ? B B x 3f ? 3x I 2d 4f"
    expected_size = struct.calcsize(struct_format)
    
    start_time = time.time()
    last_print = 0
    print_warn = True
    
    log_win.addstr("=== TWO-WAY TELEMETRY STATION ACTIVE ===\n")
    log_win.addstr(f"Listening on port {UDP_PORT}...\n\n")
    log_win.refresh()
    
    while True:
        # --- 1. HANDLE USER INPUT ---
        try:
            c = stdscr.getch()
            if c != curses.ERR:
                if c in (curses.KEY_BACKSPACE, 127, 8):
                    input_str = input_str[:-1]
                elif c in (curses.KEY_ENTER, 10, 13):
                    line = input_str.strip().lower()
                    input_str = ""
                    
                    if line == "quit" or line == "exit":
                        break
                        
                    if not last_arduino_addr and line != "":
                        log_win.addstr(">> ERROR: Waiting for first telemetry packet to get Arduino IP!\n")
                    elif line:
                        cmd_id = 0
                        vals = [0.0, 0.0, 0.0]
                        
                        if line == "arm": cmd_id = 1
                        elif line == "disarm": cmd_id = 2
                        elif line == "release": cmd_id = 4
                        elif line == "deploy": cmd_id = 5
                        elif line.startswith("tune"):
                            parts = line.split()

                            if len(parts) == 4:
                                try:
                                    vals = [float(parts[1]), float(parts[2]), float(parts[3])]
                                    cmd_id = 3
                                except ValueError:
                                    log_win.addstr(">> ERROR: Invalid tune numbers\n")
                            else:
                                log_win.addstr(">> ERROR: format is 'tune p i d'\n")
                        else:
                            log_win.addstr(f">> Unknown command: '{line}'\n")
                        
                        if cmd_id != 0:
                            data = struct.pack("< I B 3f", 0xA1B2C3D4, cmd_id, vals[0], vals[1], vals[2])
                            sock.sendto(data, last_arduino_addr)
                            log_win.addstr(f">> SENT: ID {cmd_id}, Vals {vals}\n")
                    
                    log_win.refresh()
                elif 32 <= c <= 126:
                    input_str += chr(c)
        except KeyboardInterrupt:
            break
            
        # Draw command bar
        cmd_win.erase()
        cmd_win.hline(0, 0, '-', max_x)
        cmd_win.addstr(1, 0, f"Command (type 'exit' to quit) > {input_str}")
        cmd_win.refresh()
        
        # --- 2. HANDLE INCOMING TELEMETRY ---
        try:
            data, addr = sock.recvfrom(1024)
            last_arduino_addr = addr
            
            actual_format = struct_format
            if len(data) == expected_size:
                pass
            elif len(data) == expected_size + 4:
                actual_format = "< B 3x 6I 36f 4H ? B B x 3f ? 3x I 4x 2d 4f"
                if print_warn:
                    log_win.addstr("(Note: 8-byte double alignment active)\n")
                    print_warn = False
            else:
                log_win.addstr(f"Discarding packet of length {len(data)}\n")
                log_win.refresh()
                continue

            unpacked = struct.unpack(actual_format, data)

            elapsed = time.time() - start_time
            
            # Print at 2Hz (or 4Hz)
            if elapsed - last_print >= 0.5:
                roll = unpacked[9] * 180.0 / 3.14159
                pitch = unpacked[10] * 180.0 / 3.14159
                yaw = unpacked[11] * 180.0 / 3.14159
                sats = unpacked[48]
                log_win.addstr(f"[{elapsed:5.1f}s] R:{roll:5.1f} | P:{pitch:5.1f} | Y:{yaw:5.1f} | L/R PWM: {unpacked[43]}/{unpacked[44]} | GPS: {sats} sats\n")
                log_win.refresh()
                last_print = elapsed
                
        except BlockingIOError:
            time.sleep(0.01)

# Start the interactive UI
try:
    curses.wrapper(main)
except KeyboardInterrupt:
    pass
