# 0719 Fold-RC kxkT14s Telemetry

Flash `0719_foldRC_kxkT14s.ino` to the Nano RP2040 Connect, then run:

```bash
python3 kxkT14s_fold_rc.py
```

The Python script starts the `kxkT14s` hotspot, shows live telemetry, and restores the previous Wi-Fi connection when you exit.

The Python script is configured for my Fedora T14s ground station and will need NetworkManager/hotspot updates for other machines.

The dashboard shows attitude, PWM, GPS fix/type, satellites, position, and up-positive vertical velocity.

At boot, the serial monitor should print `SD logging initialized before radio startup.` before Wi-Fi connects. This deliberately uses the same isolated D10/SPI `SD.begin` retry and CSV-file creation sequence as `test/rotational_mode/sweepFilterRot` before initializing GPS or Wi-Fi.

The command field accepts `fold` and `unfold` for live wing control. These replace the former momentary A2 (fold) and A1 (unfold), retaining the proven sequence and PWM positions: `fold` moves AoA to 1125/1050 µs, then sweeps D6 from 2475 to 500 µs; `unfold` sweeps D6 back to 2475 µs, restores flat AoA at 1575/1500 µs, then holds the 100 µs (about 10-degree) flight sweep-back at 2375 µs. The board starts in that unfolded flight position.

`arm`, `disarm`, `release`, `deploy`, and `tune P I D` remain UDP link tests only: they are printed by the sketch and do not move hardware.

If UDP port 5000 is blocked:

```bash
sudo firewall-cmd --permanent --zone=nm-shared --add-port=5000/udp
sudo firewall-cmd --reload
```
