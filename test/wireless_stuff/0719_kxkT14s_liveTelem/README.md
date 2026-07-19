# 0719 kxkT14s Live Telemetry

This is the `sweepFilterRot` fixed-position, rotational-sweep data logger with the working `kxkT14s` Wi-Fi/UDP telemetry setup merged in.

It preserves the sweep sketch's SD logging and sends every queued `ControlRecord` over UDP broadcast on port `5000`; wireless telemetry therefore continues if the SD card is absent or fails. The GPS fields added to the record and live displays are:

- GPS fix validity and fix type
- satellite count
- horizontal, vertical, and speed accuracy
- latitude, longitude, altitude, and GPS time-of-week
- north/east velocity and vertical velocity shown as **up-positive** m/s

Despite the older directory's wording, this is not Arduino firmware OTA updating: it is Wi-Fi/UDP live telemetry and a test uplink channel. The incoming `arm`, `disarm`, `release`, `deploy`, and `tune` packets are validated and printed by the sketch; they intentionally do not actuate hardware in this fixed-position test.

## Files

- `0719_kxkT14s_liveTelem.ino` — the Nano RP2040 Connect sketch. It uses `WiFiNINA`, connects to hotspot SSID `kxkT14s`, retains the existing 20 Hz SD data logger, and broadcasts the same full record used for CSV logging.
- `kxkT14s_udp_receiver.py` — minimal, read-only HUD.
- `kxkT14s_twoway_station.py` — curses HUD and protocol-compatible test uplink console.
- `kxkT14s_telemetry_common.py` — shared binary decoder and safe NetworkManager hotspot lifecycle.

## Tomorrow's run

1. Flash `0719_kxkT14s_liveTelem.ino` to the Nano RP2040 Connect.
2. Power the board and wait for serial output confirming Wi-Fi connection (SD logging still works if it cannot join the hotspot).
3. From this directory, run either:

   ```bash
   python3 kxkT14s_udp_receiver.py
   # or
   python3 kxkT14s_twoway_station.py
   ```

   The scripts enable the `kxkT14s` hotspot, then restore the previous Wi-Fi connection when they exit. Pass `--no-hotspot` if the hotspot is already running.

4. If Fedora blocks inbound broadcast traffic, configure this once:

   ```bash
   sudo firewall-cmd --permanent --zone=nm-shared --add-port=5000/udp
   sudo firewall-cmd --reload
   ```

The wireless password remains the same one already used by the referenced telemetry project. Do not run the test uplink commands as a flight-control interface; this sketch only reports them for link verification.
