# 719_GlideMission (wind-guard variant)

Development version (was 7182) — `718_GlideMission` stays frozen as the working backup.  
Gliding aircraft flight controller for the Nano RP2040 Connect.  
Based on `708_HomeBench` (HGLRC M100 MINI GPS, 15-state ESEKF, RTOS logger).

**Two ways to flash** (identical code):
- Split sketch: open `719_GlideMission.ino` here — the IDE builds all six `.ino` files together.
- Single file: open `719_Combined/719_Combined.ino` — everything concatenated into one file.
  Regenerate after editing the split files (order: main, esekf, gps, imu, logger, mission).

Added in 719:
- **Wind state machine** (`nav_wind_state` in the log: 0/1/2):
  - **NORMAL → PENETRATION**: if a saturated navigation turn runs for 5 s while
    the closing speed on the target stays negative, the wind beats the normal
    glide airspeed. The morph pulls back to `PENETRATION_MORPH_DEG` (partial
    sweep): less wing area + aft-shifted neutral point raise the trim airspeed
    so the aircraft can out-fly the wind. Steering continues throughout.
  - **PENETRATION → NORMAL**: closing speed > 2 m/s (sweep reopens).
  - **PENETRATION → LEVEL_HOLD**: still losing ground after 8 s — even the
    raised airspeed cannot beat the wind. Wings level, morph reopened,
    minimum sink. Also entered directly when AGL < 25 m (no altitude budget
    for the higher sink rate of penetration).
  - **LEVEL_HOLD → NORMAL**: closing speed > 0.5 m/s.

  > **Bench-verify before enabling in flight**: `PENETRATION_MORPH_DEG = 140°`
  > is a servo-space placeholder. Map the morph linkage (servo angle → actual
  > wing sweep) and confirm the wing is stable at that partial-sweep setting.
  > Set `ENABLE_PENETRATION_MODE 0` to fall back to the plain wind guard.

- **M4 altitude floor** (`H_FORCE_APPROACH_M = 15 m`): forces the M3→M4→M5
  heli transition when descending below 15 m AGL even if the target zone was
  never reached — landing in heli configuration beats gliding into the ground.
- **NIS logging** (`gps_nis_pos`, `gps_nis_vel`): per-epoch filter consistency
  metric (chi-square, 3 dof). Healthy filter: mean ≈ 3, 95% of samples < 7.81.
- **Servo slew limiter** (`SERVO_SLEW_LIMIT_DPS = 300°/s`): mode transitions
  become ramps, and simultaneous stall-current spikes through the TPS61088
  5 V boost rail are staggered (brownout prevention).
- **Rate-target clamp** (`RATE_TARGET_LIMIT_DPS = 60°/s`): outer loop cannot
  request rates the inner loop and servos cannot track.
- **Log window 600 → 1200 s**: ground wait + drone carry no longer risk
  closing the log before release.

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
M3  GLIDE         Wings open. Cascade PID + GPS bank-to-turn guidance
                  steer toward the target. Wind state machine may engage
                  penetration sweep (morph partial fold) in strong headwind.
                  (Set ENABLE_APPROACH_PHASES 1 and fill target coords
                   to activate M4/M5.)
        ↓ (if ENABLE_APPROACH_PHASES) inside APPROACH_RADIUS_M for
          APPROACH_CONFIRM_CYCLES consecutive frames,
          OR descending below H_FORCE_APPROACH_M (15 m) — forced
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
| `RATE_TARGET_LIMIT_DPS` | 60°/s | Clamp on outer-loop rate command |
| `SERVO_SLEW_LIMIT_DPS` | 300°/s | Servo command ramp limit (must stay > M2 sweep rate 225°/s) |

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
H_FORCE_APPROACH_M    // 15 m — AGL floor that forces M4 regardless of distance
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
| `nav_roll_tgt_deg` | Roll target commanded by GPS guidance (0 when not navigating) |
| `nav_wind_state` | 0=normal, 1=penetration sweep, 2=level hold |
| `gps_nis_pos / gps_nis_vel` | Filter consistency (chi², 3 dof); healthy mean ≈ 3, 95% < 7.81 |
| `left_deg` vs `left_pwm` | Command vs slew-limited output; the difference shows limiter activity |

### Log file naming

Automatic from the GPS date: `MM_DD_xx.CSV` (e.g. `07_18_00.CSV`).  
The logger waits up to 60 s for a valid GPS date before opening the file;
fallback name is `NOFIX_xx.CSV`. No manual tag to edit between flight days.
After GPS origin lock the receiver state is saved to battery-backed RAM
(UBX-UPD-SOS) so the next power-on hot-starts in seconds.

---

## Pre-flight checklist

- [ ] Servo positions measured and set in constants (fold + flat + heli)
- [ ] Fold collision clearance verified on bench (morph sweep with wings at fold AoA)
- [ ] **Control sign check**: tilt the aircraft by hand and confirm the wing
      servos deflect to oppose the tilt (IMU axes right ≠ servo signs right)
- [ ] **Nav sign check**: confirmed turn direction on first flight, or accept
      that `NAV_KP_ROLL_PER_HDG_DEG` may need negating
- [ ] **Penetration bench check**: morph at 140° gives intended sweep and the
      wing is stable there (else set `ENABLE_PENETRATION_MODE 0`)
- [ ] `ENABLE_APPROACH_PHASES 0` unless target coordinates are filled
- [ ] GPS has 3D fix and `gps_origin_set` confirmed before arming drone
- [ ] Deployment thresholds reviewed against planned release altitude
- [ ] SD card inserted; confirm `[Core1] GPS date:` then `[Core1] Logging to:` on Serial
- [ ] Serial monitor shows `Ready. Waiting for release` before launch
