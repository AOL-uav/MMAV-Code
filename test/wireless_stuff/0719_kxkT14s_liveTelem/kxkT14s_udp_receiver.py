#!/usr/bin/env python3
"""One-way live telemetry receiver for the 0719 fixed sweep test."""

import argparse
import time

from kxkT14s_telemetry_common import HotspotLifecycle, decode_record, open_telemetry_socket


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-hotspot", action="store_true", help="do not manage NetworkManager")
    args = parser.parse_args()

    if not args.no_hotspot:
        hotspot = HotspotLifecycle()
        hotspot.start()
        hotspot.register_cleanup()

    sock = open_telemetry_socket()
    print("Listening for UDP telemetry on port 5000 (Ctrl+C to stop).")
    last_print = 0.0
    try:
        while True:
            try:
                data, _ = sock.recvfrom(1024)
            except BlockingIOError:
                time.sleep(0.01)
                continue

            record = decode_record(data)
            if record is None:
                continue
            now = time.monotonic()
            if now - last_print < 0.5:
                continue
            fix = "FIX" if record.gps_fix else "NO-FIX"
            print(
                f"[{record.timestamp_s:7.2f}s] "
                f"R/P/Y {record.roll_deg:6.1f}/{record.pitch_deg:6.1f}/{record.yaw_deg:6.1f} deg | "
                f"PWM L/R {record.left_pwm_us}/{record.right_pwm_us} | "
                f"GPS {fix} type={record.gps_fix_type} sats={record.gps_satellites} | "
                f"Vv(up) {record.vertical_velocity_up_mps:+.2f} m/s"
            )
            last_print = now
    except KeyboardInterrupt:
        print("\nStopped telemetry receiver.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
