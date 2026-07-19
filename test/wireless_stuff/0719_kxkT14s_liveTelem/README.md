# 0719 Live Telemetry

Flash `0719_kxkT14s_liveTelem.ino` to the Nano RP2040 Connect, then run:

```bash
python3 kxkT14s_live_telem.py
```

The Python script starts the `kxkT14s` hotspot, shows live telemetry, and restores the previous Wi-Fi connection when you exit.

The Python script is configured for my Fedora-based T14s ground station and will need NetworkManager/hotspot updates for other machines.

The dashboard shows attitude, PWM, GPS fix/type, satellites, position, and up-positive vertical velocity.

The command field accepts `arm`, `disarm`, `release`, `deploy`, and `tune P I D` for UDP link testing only. They are printed by the sketch and do not move hardware.

If UDP port 5000 is blocked:

```bash
sudo firewall-cmd --permanent --zone=nm-shared --add-port=5000/udp
sudo firewall-cmd --reload
```
