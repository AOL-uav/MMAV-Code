# 7182_GlideMission (wind-guard variant)

Development copy of `718_GlideMission` — 718 stays frozen as the working backup.  
Gliding aircraft flight controller for the Nano RP2040 Connect.  
Based on `708_HomeBench` (HGLRC M100 MINI GPS, 15-state ESEKF, RTOS logger).

Added in 7182:
- **Wind guard (anti-circling)**: if a saturated navigation turn runs for 5 s
  while the closing speed on the target stays negative, the wind is stronger
  than the airspeed and no heading can make progress. The guard drops to
  wings-level (minimum sink) instead of circling downwind, and resumes
  navigation automatically once closing speed exceeds 0.5 m/s.
  Constants: `NAV_STUCK_TURN_MS`, `NAV_RESUME_CLOSE_MPS`.

---

## Hardware

| Component | Connection |
|-----------|-----------|
| Left wing servo  | D4 |
| Right wing servo | D3 |
| Morph sweep servo | D5 |
| Tail servo | D6 |
| GPS (HGLRC M100 MINI, u-blox M10) | Serial1 — TX→D1, RX→D0 |
| SD card (SPI) | CS→D10 |

All servo PWM range: 500–2500 µs.

---

## Mission overview

```
M0  CARRIED       Wings folded, being carried up by drone.
                  Free-fall detector watching accelerometer.
        ↓ |accel| < 0.3 g for 10 consecutive frames (125 ms)
M1  FREE_FALL     Released. Still folded. Building airspeed.
        ↓ Primary:  speed ≥ V_OPEN_MIN_MPS  AND  AGL ≥ H_OPEN_MIN_M
          Failsafe: AGL ≤ H_FORCE_OPEN_M  (regardless of speed)
          Both require esekf.v[2] < 0 (descending) to avoid
          triggering during the ascent through the same altitude.
M2  DEPLOYING     Morph sweeps from fold to fully open in T2_OPEN_S (0.8 s).
                  Left/right wings stay at fold AoA until morph clears.
        ↓ modeElapsedS ≥ T2_OPEN_S
M3  GLIDE         Wings fully open. Cascade PID holds level flight.
                  (Set ENABLE_APPROACH_PHASES 1 and fill target coords
                   to activate M4/M5.)
        ↓ (if ENABLE_APPROACH_PHASES) inside APPROACH_RADIUS_M for
          APPROACH_CONFIRM_CYCLES consecutive frames
M4  APPROACH      PID continues. 0.5 s hold, then transitions to heli.
        ↓ modeElapsedS ≥ 0.5 s
M5  HELI_DESCENT  Wings in autorotation config. Morph adjusts descent rate.
```

---

## Key parameters to set before each flight

Edit the **"Edit each flight"** section at the top of the `.ino` file.

### Servo positions
Measure with `sweep_aero_logger` and convert: `deg = (us − 500) × 180 / 2000`.

```cpp
WING_FLAT_LEFT_DEG   // flat (glide) left wing position
WING_FLAT_RIGHT_DEG  // flat (glide) right wing position
WING_FOLD_LEFT_DEG   // fold position = flat − 30° (collision prevention)
WING_FOLD_RIGHT_DEG  // fold position = flat + 30°
```

> **Verify collision clearance on the bench** before first flight: manually move wings to fold positions, then sweep morph from 0° to 180° and confirm no contact.

### Deployment thresholds (tune from drop tests)

| Constant | Default | Notes |
|----------|---------|-------|
| `V_OPEN_MIN_MPS` | 8.0 m/s | Minimum speed for wing deployment. Raise if glider stalls on opening. |
| `H_OPEN_MIN_M` | 40 m | AGL floor for normal deployment. |
| `H_FORCE_OPEN_M` | 25 m | Emergency deployment floor (speed not required). |
| `T2_OPEN_S` | 0.8 s | Morph sweep duration. |

### PID gains (M3)

Start with defaults and tune from flight logs:

| Constant | Default | Role |
|----------|---------|------|
| `KP_OUTER_ROLL/PITCH` | 1.2 / 1.0 | Angle error → rate command (deg/s per deg) |
| `KP_INNER_ROLL/PITCH` | 0.08 | Rate error → servo command |
| `KI_INNER_ROLL/PITCH` | 0.01 | Steady-state trim correction (max 4° via integrator limit) |
| `INTEGRATOR_LIMIT_DEG` | 400 | Integral state clamp; Ki × limit = 4° max trim authority |
| `PID_NEUTRAL_DEG` | 90° | Servo neutral for PID mixing (both wings) |
| `CMD_LIMIT_DEG` | 30° | Max deflection from 90° neutral; range = [60°, 120°] |

### GPS navigation (M3/M4, `ENABLE_GPS_NAVIGATION 1`)

Bank-to-turn guidance: roll target = `NAV_KP × (bearing − ground track)`, fed into the cascade PID.
Ground track comes from ESEKF velocity (GPS-fused), not yaw — yaw drifts without a magnetometer.
Default target is 0/0 = the GPS origin (launch point).

| Constant | Default | Role |
|----------|---------|------|
| `NAV_KP_ROLL_PER_HDG_DEG` | 0.4 | Roll per degree of heading error. **Negate if aircraft turns away from target.** |
| `NAV_MAX_BANK_DEG` | 15° | Bank limit for navigation turns |
| `NAV_MIN_GROUND_SPEED_MPS` | 3.0 | Below this, hold wings level (track angle unreliable) |
| `NAV_ARRIVAL_RADIUS_M` | 20 m | Inside this, stop steering (bearing changes too fast) |

Navigation falls back to wings-level whenever: no GPS origin, no fix, ground speed too low, or within arrival radius.

### M4/M5 (approach and heli descent)

```cpp
#define ENABLE_APPROACH_PHASES 0  // set 1 when ready
APPROACH_TARGET_N_M   // target North offset from GPS origin (metres)
APPROACH_TARGET_E_M   // target East offset
APPROACH_RADIUS_M     // 15 m — distance to trigger M4
```

---

## Critical annotated values

### Free-fall threshold — `FREE_FALL_THRESHOLD_MPS2 = 0.3 g`
In free fall, gravity is cancelled and the accelerometer reads ≈ 0 m/s².  
`0.3 g (2.94 m/s²)` sits above typical drone vibration (< 0.15 g) and below any real supported attitude.  
**Raise** this value if the carrier drone vibrates heavily and causes false triggers.

### Confirmation window — `FREE_FALL_CONFIRM_FRAMES = 10`
10 frames at 80 Hz = **125 ms** of continuous free-fall signal.  
Long enough to reject a single motor pulse; short enough to lose < 8 cm of altitude.

### Altitude gate direction — `esekf.v[2] < −0.5 m/s`
Both deployment triggers require a **downward** vertical velocity.  
This prevents accidental deployment while ascending through the threshold altitude on the way up.

### ESEKF rate gate — `ESEKF_LEVEL_RATE_GATE_RADPS = 0.25 rad/s`
Accelerometer-based attitude correction is suppressed when the body is rotating faster than 14°/s.  
During rapid manoeuvres, centripetal acceleration would corrupt the roll/pitch estimate.  
Validated result: roll RMSE improved from 14.9° → 3.0° during morphing transitions.

### Adaptive GPS velocity noise — `GPS_VELOCITY_NOISE_MPS = 0.25 m/s` (floor)
The M10 receiver reports `sAcc` (speed accuracy) in each NAV-PVT packet (byte offset 68).  
The ESEKF uses `max(sAcc, 0.25)` as the velocity measurement noise.  
In open sky `sAcc ≈ 0.05 m/s`; under obstruction it rises automatically, loosening fusion.

---

## Log columns

| Column | Description |
|--------|-------------|
| `mode_id / mode_name` | Flight phase (0=carried … 5=heli_descent) |
| `accel_norm_mps2` | Raw accelerometer magnitude; use to verify free-fall detection in post-processing |
| `free_fall_count` | Value of the free-fall counter at each sample |
| `rate_tgt_roll/pitch` | Outer PID loop output (desired angular rate, deg/s) |
| `u_roll_deg / u_pitch_deg` | Inner PID loop servo command |
| `gps_sacc_mps` | M10 speed accuracy estimate (adaptive velocity noise source) |
| `approach_count` | M3→M4 confirmation counter |

---

## Pre-flight checklist

- [ ] Servo positions measured and set in constants (fold + flat + heli)
- [ ] Fold collision clearance verified on bench (morph sweep with wings at fold AoA)
- [ ] `ENABLE_APPROACH_PHASES 0` unless target coordinates are filled
- [ ] GPS has 3D fix and `gps_origin_set` confirmed before arming drone
- [ ] Deployment thresholds reviewed against planned release altitude
- [ ] SD card inserted; confirm `[Core1] Logging to:` appears on Serial
- [ ] Serial monitor shows `Ready. Waiting for release` before launch
