#!/usr/bin/env python3
"""Start the kxkT14s hotspot, show live telemetry, and test the UDP uplink."""

import argparse
import atexit
import curses
import socket
import struct
import subprocess
import time


HOTSPOT_NAME = "Hotspot"
HOTSPOT_SSID = "kxkT14s"
HOTSPOT_PASSWORD = "Mika12345."
UDP_PORT = 5000
UPLINK_MAGIC = 0xA1B2C3D4

# Native ControlRecord layouts used by the RP2040 build (with or without
# padding before the two doubles).
RECORD_STRUCTS = (
    struct.Struct("<B3x6I36f4H?BBx3f?3xI2d4f"),
    struct.Struct("<B3x6I36f4H?BBx3f?3xI4x2d4f"),
)
UPLINK_STRUCT = struct.Struct("<IB3f")
COMMAND_IDS = {"arm": 1, "disarm": 2, "release": 4, "deploy": 5, "fold": 6, "unfold": 7}
COMMAND_HELP = "Commands: fold | unfold | arm | disarm | release | deploy | tune P I D | exit"


def active_wifi_name():
    try:
        result = subprocess.run(
            ["nmcli", "-t", "-f", "NAME,TYPE", "connection", "show", "--active"],
            check=True, capture_output=True, text=True,
        )
        for line in result.stdout.splitlines():
            name, _, kind = line.partition(":")
            if kind == "802-11-wireless" and name != HOTSPOT_NAME:
                return name
    except (OSError, subprocess.CalledProcessError):
        pass
    return None


def start_hotspot():
    print(f"Starting hotspot: {HOTSPOT_SSID}")
    existing = subprocess.run(
        ["nmcli", "connection", "up", HOTSPOT_NAME], capture_output=True, text=True,
    )
    if existing.returncode != 0:
        subprocess.run(
            ["nmcli", "device", "wifi", "hotspot", "ssid", HOTSPOT_SSID,
             "password", HOTSPOT_PASSWORD],
            check=True,
        )
    print("Hotspot ready.")


def parse_command(line):
    words = line.lower().split()
    if len(words) == 1 and words[0] in COMMAND_IDS:
        return COMMAND_IDS[words[0]], (0.0, 0.0, 0.0)
    if len(words) == 4 and words[0] == "tune":
        try:
            return 3, (float(words[1]), float(words[2]), float(words[3]))
        except ValueError:
            pass
    return None


def dashboard(stdscr, sock):
    """Curses dashboard with live telemetry and wing fold/unfold control."""
    try:
        curses.curs_set(1)
    except curses.error:
        pass
    stdscr.nodelay(True)
    typed = ""
    status = "Waiting for telemetry..."
    board_address = None
    latest = None
    last_packet_time = 0.0

    while True:
        try:
            while True:
                data, address = sock.recvfrom(1024)
                decoder = next((item for item in RECORD_STRUCTS if item.size == len(data)), None)
                if decoder is not None:
                    latest = decoder.unpack(data)
                    board_address = address
                    last_packet_time = time.monotonic()
        except BlockingIOError:
            pass

        key = stdscr.getch()
        if key in (curses.KEY_BACKSPACE, 127, 8):
            typed = typed[:-1]
        elif key in (10, 13, curses.KEY_ENTER):
            line = typed.strip()
            typed = ""
            if line.lower() in {"exit", "quit"}:
                return
            command = parse_command(line)
            if not line:
                pass
            elif command is None:
                status = f"Invalid command. {COMMAND_HELP}"
            elif board_address is None:
                status = "No board packet yet; command was not sent."
            else:
                command_id, values = command
                sock.sendto(
                    UPLINK_STRUCT.pack(UPLINK_MAGIC, command_id, *values), board_address
                )
                if command_id in (COMMAND_IDS["fold"], COMMAND_IDS["unfold"]):
                    status = f"Sent wing command: {line}."
                else:
                    status = f"Sent telemetry-only uplink: {line}."
        elif 32 <= key <= 126:
            typed += chr(key)

        rows, cols = stdscr.getmaxyx()
        stdscr.erase()
        lines = [
            "0719 kxkT14s LIVE TELEMETRY",
            "fold/unfold move the wings. All other uplinks are telemetry-only.",
            COMMAND_HELP,
            "",
        ]
        if latest is None:
            lines.append("No valid telemetry packet received yet.")
        else:
            gps_state = "FIX" if latest[47] else "NO-FIX"
            vertical_up_mps = -latest[60]  # NED down-positive -> display up-positive.
            packet_age = time.monotonic() - last_packet_time
            lines.extend([
                f"Board: {board_address[0]}:{board_address[1]}    packet age: {packet_age:.2f} s",
                f"Time: {latest[1] / 1000.0:.2f} s",
                f"Attitude: roll {latest[13]:+.2f}  pitch {latest[14]:+.2f}  yaw {latest[15]:+.2f} deg",
                f"PWM: left {latest[43]}  right {latest[44]}  sweep {latest[45]}  tail {latest[46]}",
                f"GPS: {gps_state}   type {latest[49]}   satellites {latest[48]}   iTOW {latest[54]} ms",
                f"Position: {latest[55]:.7f}, {latest[56]:.7f}   altitude {latest[57]:.1f} m",
                f"Velocity: N {latest[58]:+.2f}  E {latest[59]:+.2f}  vertical(up) {vertical_up_mps:+.2f} m/s",
                f"GPS accuracy: h {latest[50]:.1f} m  v {latest[51]:.1f} m  speed {latest[52]:.2f} m/s",
            ])
        lines.extend(["", status, "", f"Command > {typed}"])
        for row, line in enumerate(lines[:rows]):
            stdscr.addnstr(row, 0, line, max(0, cols - 1))
        stdscr.refresh()
        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-hotspot", action="store_true", help="do not manage the hotspot")
    args = parser.parse_args()

    previous_wifi = None
    if not args.no_hotspot:
        previous_wifi = active_wifi_name()
        start_hotspot()

        def restore_wifi():
            if previous_wifi:
                print(f"Restoring Wi-Fi: {previous_wifi}")
                subprocess.run(["nmcli", "connection", "up", previous_wifi], check=False)
            else:
                subprocess.run(["nmcli", "connection", "down", HOTSPOT_NAME], check=False)

        atexit.register(restore_wifi)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setblocking(False)
    try:
        curses.wrapper(dashboard, sock)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
