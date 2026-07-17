# Wireless Telemetry

This directory contains code for logging and wirelessly transmitting MAV flight telemetry using the Arduino Nano RP2040 Connect.

## Files

- `wireless_stuff.ino`: The main flight control and logging sketch.
  - Initializes the NINA-W102 Wi-Fi module to connect to a local hotspot.
  - Maintains the 50Hz SD card logging redundancy.
  - Broadcasts the binary `ControlRecord` struct over UDP (Port 5000) to the local network.
- `udp_receiver.py`: A Python-based ground station receiver script.
  - Listens on UDP port 5000 for incoming telemetry packets.
  - Unpacks the binary C-struct and prints parsed values (Roll, Pitch, Yaw, PWM, GPS status) to the console at 2Hz.

## Usage

1. Flash `wireless_stuff.ino` to the Nano RP2040 Connect.
2. Ensure the board is powered and the specified Wi-Fi hotspot (`Kai's A55`) is active.
3. Connect the ground station computer to the same hotspot.
4. Run the Python receiver script: `python udp_receiver.py`
