# 0719 kxkT14s Live Telemetry

`0719_kxkT14s_liveTelem.ino` is the fixed-position `sweepFilterRot` logger with Wi-Fi UDP telemetry added.

It logs to SD and broadcasts the same `ControlRecord` on UDP port `5000`. The live display includes attitude, PWM, GPS fix type, satellite count, position, and up-positive vertical velocity.

## Run

1. Flash `0719_kxkT14s_liveTelem.ino` to the Nano RP2040 Connect.
2. From this directory, start either viewer:

   ```bash
   python3 kxkT14s_udp_receiver.py
   python3 kxkT14s_twoway_station.py
   ```

   Each script starts the `kxkT14s` hotspot and restores the previous Wi-Fi connection when it exits. Use `--no-hotspot` if it is already running.

3. If packets are blocked, run once:

   ```bash
   sudo firewall-cmd --permanent --zone=nm-shared --add-port=5000/udp
   sudo firewall-cmd --reload
   ```

The test-station uplink commands are received and printed by the sketch, but do not move hardware.
