"""Shared UDP decoding and hotspot lifecycle helpers for the 0719 test."""

from __future__ import annotations

import atexit
import socket
import struct
import subprocess
from dataclasses import dataclass
from typing import Optional


HOTSPOT_NAME = "Hotspot"
HOTSPOT_SSID = "kxkT14s"
HOTSPOT_PASSWORD = "Mika12345."
UDP_PORT = 5000
UPLINK_MAGIC = 0xA1B2C3D4
UPLINK_STRUCT = struct.Struct("<IB3f")

# ControlRecord is deliberately sent as the board's native struct.  RP2040
# builds use one of these two layouts depending on double alignment.
_RECORD_FORMAT = "<B3x6I36f4H?BBx3f?3xI2d4f"
_RECORD_FORMAT_PADDED = "<B3x6I36f4H?BBx3f?3xI4x2d4f"
RECORD_STRUCTS = (struct.Struct(_RECORD_FORMAT), struct.Struct(_RECORD_FORMAT_PADDED))


@dataclass(frozen=True)
class Telemetry:
    timestamp_s: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    left_pwm_us: int
    right_pwm_us: int
    morph_pwm_us: int
    tail_pwm_us: int
    gps_fix: bool
    gps_satellites: int
    gps_fix_type: int
    gps_hacc_m: float
    gps_vacc_m: float
    gps_sacc_mps: float
    gps_itow_ms: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    velocity_north_mps: float
    velocity_east_mps: float
    vertical_velocity_up_mps: float


def decode_record(data: bytes) -> Optional[Telemetry]:
    """Return a decoded ControlRecord, or None for an unrelated UDP packet."""
    decoder = next((item for item in RECORD_STRUCTS if len(data) == item.size), None)
    if decoder is None:
        return None

    values = decoder.unpack(data)
    # Field positions are kept here, rather than scattered through each UI.
    # The last GPS velocity is NED down-positive; the live display is up-positive.
    return Telemetry(
        timestamp_s=values[1] / 1000.0,
        roll_deg=values[13],
        pitch_deg=values[14],
        yaw_deg=values[15],
        left_pwm_us=values[43],
        right_pwm_us=values[44],
        morph_pwm_us=values[45],
        tail_pwm_us=values[46],
        gps_fix=values[47],
        gps_satellites=values[48],
        gps_fix_type=values[49],
        gps_hacc_m=values[50],
        gps_vacc_m=values[51],
        gps_sacc_mps=values[52],
        gps_itow_ms=values[54],
        latitude_deg=values[55],
        longitude_deg=values[56],
        altitude_m=values[57],
        velocity_north_mps=values[58],
        velocity_east_mps=values[59],
        vertical_velocity_up_mps=-values[60],
    )


def open_telemetry_socket() -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.setblocking(False)
    return sock


class HotspotLifecycle:
    """Bring up the dedicated hotspot and restore the prior Wi-Fi on exit."""

    def __init__(self) -> None:
        self.previous_network: Optional[str] = None

    def start(self) -> None:
        try:
            active = subprocess.run(
                ["nmcli", "-t", "-f", "NAME,TYPE", "connection", "show", "--active"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout
            for line in active.splitlines():
                name, _, connection_type = line.partition(":")
                if connection_type == "802-11-wireless" and name != HOTSPOT_NAME:
                    self.previous_network = name
                    break
        except (OSError, subprocess.CalledProcessError):
            pass

        print(f"Starting telemetry hotspot ({HOTSPOT_SSID})...")
        result = subprocess.run(
            ["nmcli", "connection", "up", HOTSPOT_NAME],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            subprocess.run(
                ["nmcli", "device", "wifi", "hotspot", "ssid", HOTSPOT_SSID,
                 "password", HOTSPOT_PASSWORD],
                check=True,
            )
        print("Hotspot active; internet Wi-Fi is temporarily paused.")

    def restore(self) -> None:
        if self.previous_network:
            print(f"Restoring Wi-Fi connection: {self.previous_network}")
            subprocess.run(["nmcli", "connection", "up", self.previous_network], check=False)
        else:
            subprocess.run(["nmcli", "connection", "down", HOTSPOT_NAME], check=False)

    def register_cleanup(self) -> None:
        atexit.register(self.restore)
