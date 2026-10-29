# Wing Sweep Servo Characterization

Wing fold/unfold test. Nano RP2040 Connect (`arduino:mbed_nano:nanorp2040connect`).
Firmware: `wing_sweep_test.ino` in this folder (continuous-servo version).
Positional-servo version: `../wing_sweep_positional/wing_sweep_positional.ino`.

## Trigger
- Momentary **button on A2** (board silkscreen slot "10" - board pin numbers do NOT
  match the Nano D-pins; found empirically with a pin-finder). INPUT_PULLUP, press =
  short to GND. 40 ms debounce + 3 s cooldown.
- NOTE: RST-button toggle via RP2040 watchdog SCRATCH was tested and FAILED on the
  mbed core (SCRATCH doesn't survive a reset) - dropped in favor of the A2 button.

## Sweep servo (D5) - CONTINUOUS rotation (current bench version)
- 1500 us = stop. pulse < 1500 = FOLD (CW), pulse > 1500 = UNFOLD (CCW).
- **DEMO values (2026-06-26), tuned by eye, wings OFF:**
  - speed = offset 200 us  ->  fold 1300 us, unfold 1700 us
  - fold ~800 ms ~= 90 deg, unfold ~1270 ms ~= 90 deg (unfold direction runs slower)
- Measured: fold ~45 deg in 400 ms, unfold ~25 deg in 350 ms (at offset 200).
- Earlier offset-500 calibration (servo_control tool, 2026-06-24): fold 127.3 deg/s,
  unfold 141.85 deg/s @ pulses 1000/2000. Superseded by the slower demo speed above.

### LIMITATION (why a positional servo is recommended)
The continuous sweep is OPEN-LOOP: travel = speed x time, no position feedback.
Actual angle **drifts with load** (fold ~45 vs unfold ~25 for similar times = net
drift per cycle), and flight loads vary. **Not reliable for flight.** Use a
**positional (metal-gear) servo** that holds an absolute commanded angle - see the
positional firmware variant. Demo timings need re-tuning on any change.

## AoA servos (D4 left, D3 right) - POSITIONAL
- Driven to fixed pulses per state (no degree math). Attach range 500-2500 us (the
  old 1000-2000 us range CAPPED travel, so they only reached ~30 deg).
- FLAT (unfolded): left 1575 us, right 1500 us (measured neutral / 0 incidence).
- FOLDED: left 600 us, right 2400 us (surfaces ~90 deg to clear; TUNE toward 500/2500
  for more, back off if a servo strains). Left servo is reversed.
- Quirk: AoA must reach folded BEFORE the sweep folds so the surfaces clear. Order:
  FOLD = AoA folded -> settle -> sweep in. UNFOLD = sweep out -> AoA flat.

## Tools
- Live cal tool: `C:\Users\Kai\servo_control\` (servo_control.py / servo_jog.py).
- The continuous-servo deg/s calibration (above) only applies to a continuous servo;
  a positional servo needs no speed cal, just its two endpoint pulses.
