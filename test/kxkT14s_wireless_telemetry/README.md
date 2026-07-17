# kxkT14s Wireless Telemetry

This directory contains code customized specifically for the `kxkT14s` laptop to establish a zero-latency, direct wireless umbilical cord with the drone for live telemetry viewing.

## Files

- `kxkT14s_wireless_telemetry.ino`: The main flight control and logging sketch.
  - Initializes the Wi-Fi module to connect specifically to the laptop's `kxkT14s` hotspot.
  - Broadcasts the binary `ControlRecord` struct over UDP (Port 5000) to the local network regardless of SD card status.
- `kxkT14s_udp_receiver.py`: A fully automated Python-based ground station script.
  - Automatically disconnects the laptop from `eduroam` and brings up the `kxkT14s` hotspot on startup.
  - Listens on UDP port 5000 for incoming telemetry packets and prints a live HUD (Roll, Pitch, Yaw, PWM, GPS).
  - Automatically restores the `eduroam` internet connection when closed (Ctrl+C).

## Usage

1. Flash `kxkT14s_wireless_telemetry.ino` to the Nano RP2040 Connect.
2. Ensure the board is powered.
3. On the laptop, run the automated ground station script: 
   ```bash
   python3 kxkT14s_udp_receiver.py
   ```
   *(The script handles activating the hotspot, listening for data, and reconnecting to the internet when you quit).*

## Firewall Setup (One-Time)
If telemetry is not arriving, Fedora's `firewalld` may be blocking the incoming packets on the `nm-shared` hotspot zone. Run this once to permanently punch a hole for the telemetry:
```bash
sudo firewall-cmd --permanent --zone=nm-shared --add-port=5000/udp
sudo firewall-cmd --reload
```
