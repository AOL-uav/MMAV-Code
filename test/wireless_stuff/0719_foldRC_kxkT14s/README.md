# 0719 Fold-RC kxkT14s Telemetry

Flash `0719_foldRC_kxkT14s.ino` to the Nano RP2040 Connect, then run:

```bash
python3 kxkT14s_fold_rc.py
```

The Python script starts the `kxkT14s` hotspot, shows live telemetry, and restores the previous Wi-Fi connection when you exit.

The Python script is configured for my Fedora T14s ground station and will need NetworkManager/hotspot updates for other machines.

The dashboard shows attitude, PWM, GPS fix/type, satellites, position, and up-positive vertical velocity.

The command field accepts `fold` and `unfold` for live wing control. These replace the former momentary A2 (fold) and A1 (unfold) shorts, retaining the proven sequence and PWM positions: `fold` moves AoA to 1125/1050 µs, then sweeps D6 from 2475 to 500 µs; `unfold` sweeps D6 back to 2475 µs, then restores flat AoA at 1575/1500 µs. The board starts unfolded and flat.

`arm`, `disarm`, `release`, `deploy`, and `tune P I D` remain UDP link tests only: they are printed by the sketch and do not move hardware.

If UDP port 5000 is blocked:

```bash
sudo firewall-cmd --permanent --zone=nm-shared --add-port=5000/udp
sudo firewall-cmd --reload
```
