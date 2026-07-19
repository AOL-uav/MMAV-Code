#!/usr/bin/env python3
"""Interactive UDP telemetry viewer and test-uplink console for 0719."""

import argparse
import curses
import socket
import time
from typing import Optional, Tuple

from kxkT14s_telemetry_common import (
    HotspotLifecycle,
    UPLINK_MAGIC,
    UPLINK_STRUCT,
    decode_record,
    open_telemetry_socket,
)


def parse_command(line: str) -> Optional[Tuple[int, tuple[float, float, float]]]:
    command = line.strip().lower().split()
    if not command:
        return None
    command_ids = {"arm": 1, "disarm": 2, "release": 4, "deploy": 5}
    if len(command) == 1 and command[0] in command_ids:
        return command_ids[command[0]], (0.0, 0.0, 0.0)
    if len(command) == 4 and command[0] == "tune":
        try:
            return 3, (float(command[1]), float(command[2]), float(command[3]))
        except ValueError:
            return None
    return None


def station(stdscr: curses.window, sock: socket.socket) -> None:
    curses.curs_set(1)
    stdscr.nodelay(True)
    input_line = ""
    message = "Waiting for telemetry..."
    board_address = None
    latest = None
    last_packet_at = 0.0

    while True:
        try:
            while True:
                data, address = sock.recvfrom(1024)
                decoded = decode_record(data)
                if decoded is not None:
                    latest = decoded
                    board_address = address
                    last_packet_at = time.monotonic()
        except BlockingIOError:
            pass

        key = stdscr.getch()
        if key in (curses.KEY_BACKSPACE, 127, 8):
            input_line = input_line[:-1]
        elif key in (10, 13, curses.KEY_ENTER):
            line = input_line.strip()
            input_line = ""
            if line.lower() in {"exit", "quit"}:
                return
            parsed = parse_command(line)
            if parsed is None:
                message = "Commands: arm, disarm, release, deploy, tune P I D, exit"
            elif board_address is None:
                message = "No board address yet; wait for a telemetry packet."
            else:
                command_id, values = parsed
                payload = UPLINK_STRUCT.pack(UPLINK_MAGIC, command_id, *values)
                sock.sendto(payload, board_address)
                message = f"Sent uplink id={command_id}, values={values}"
        elif 32 <= key <= 126:
            input_line += chr(key)

        height, width = stdscr.getmaxyx()
        stdscr.erase()
        lines = [
            "0719 kxkT14s LIVE TELEMETRY",
            "UDP full ControlRecord stream + SD logging; wireless uplinks are logged by the test sketch.",
            "",
        ]
        if latest is None:
            lines.append("No valid ControlRecord received yet.")
        else:
            age = time.monotonic() - last_packet_at
            fix = "FIX" if latest.gps_fix else "NO FIX"
            lines.extend([
                f"Packet age: {age:.2f}s    Board: {board_address[0]}:{board_address[1]}",
                f"Time: {latest.timestamp_s:.2f} s",
                f"Attitude: roll {latest.roll_deg:+.2f} deg   pitch {latest.pitch_deg:+.2f} deg   yaw {latest.yaw_deg:+.2f} deg",
                f"PWM: L {latest.left_pwm_us}  R {latest.right_pwm_us}  sweep {latest.morph_pwm_us}  tail {latest.tail_pwm_us}",
                f"GPS: {fix}   type {latest.gps_fix_type}   satellites {latest.gps_satellites}   iTOW {latest.gps_itow_ms} ms",
                f"Position: {latest.latitude_deg:.7f}, {latest.longitude_deg:.7f}   altitude {latest.altitude_m:.1f} m",
                f"Velocity: north {latest.velocity_north_mps:+.2f}  east {latest.velocity_east_mps:+.2f}  vertical(up) {latest.vertical_velocity_up_mps:+.2f} m/s",
                f"GPS accuracy: h {latest.gps_hacc_m:.1f} m   v {latest.gps_vacc_m:.1f} m   speed {latest.gps_sacc_mps:.2f} m/s",
            ])
        lines.extend(["", message, "", f"Command > {input_line}"])
        for row, line in enumerate(lines[:height]):
            stdscr.addnstr(row, 0, line, max(0, width - 1))
        stdscr.refresh()
        time.sleep(0.02)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-hotspot", action="store_true", help="do not manage NetworkManager")
    args = parser.parse_args()

    hotspot = None
    if not args.no_hotspot:
        hotspot = HotspotLifecycle()
        hotspot.start()
        hotspot.register_cleanup()

    sock = open_telemetry_socket()
    try:
        curses.wrapper(station, sock)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
