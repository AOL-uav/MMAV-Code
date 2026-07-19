#!/usr/bin/env python3
"""Start the kxkT14s hotspot and print live Nano RP2040 telemetry."""

import argparse
import atexit
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


def send_test_command(sock, board_address, command):
    command_ids = {"arm": 1, "disarm": 2, "tune": 3, "release": 4, "deploy": 5}
    command_id = command_ids[command]
    sock.sendto(UPLINK_STRUCT.pack(UPLINK_MAGIC, command_id, 0.0, 0.0, 0.0), board_address)
    print(f"Sent test uplink: {command}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-hotspot", action="store_true", help="do not manage the hotspot")
    parser.add_argument(
        "--send", choices=("arm", "disarm", "tune", "release", "deploy"),
        help="send one test uplink after the first telemetry packet",
    )
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
    print("Listening for telemetry on UDP port 5000. Press Ctrl+C to stop.")

    last_print = 0.0
    test_sent = False
    try:
        while True:
            try:
                data, board_address = sock.recvfrom(1024)
            except BlockingIOError:
                time.sleep(0.01)
                continue

            decoder = next((item for item in RECORD_STRUCTS if item.size == len(data)), None)
            if decoder is None:
                continue
            value = decoder.unpack(data)

            if args.send and not test_sent:
                send_test_command(sock, board_address, args.send)
                test_sent = True

            now = time.monotonic()
            if now - last_print < 0.5:
                continue

            # Roll/pitch/yaw are already degrees. GPS vertical speed arrives
            # in NED, so negate down-positive velocity for an up-positive HUD.
            gps_state = "FIX" if value[47] else "NO-FIX"
            vertical_up_mps = -value[60]
            print(
                f"[{value[1] / 1000.0:7.2f}s] "
                f"R/P/Y {value[13]:6.1f}/{value[14]:6.1f}/{value[15]:6.1f} deg | "
                f"PWM L/R {value[43]}/{value[44]} | "
                f"GPS {gps_state} type={value[49]} sats={value[48]} | "
                f"Vv(up) {vertical_up_mps:+.2f} m/s"
            )
            last_print = now
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
