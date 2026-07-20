// ========================= Servo output =========================

static int pwmFromDeg(float deg) {
  deg = clampfLocal(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  const float frac = (deg - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
  return (int)lroundf(SERVO_MIN_US + frac*(SERVO_MAX_US - SERVO_MIN_US));
}

static void writeAllServos() {
  // Slew limiter: each channel moves toward its commanded angle at no more
  // than SERVO_SLEW_LIMIT_DPS. First call passes through unlimited so the
  // boot-time fold positioning is immediate. dt is capped at 50 ms so a
  // stalled loop cannot produce a giant step on resume.
  static bool     slewInit = false;
  static float    slewDeg[4];
  static uint32_t slewLastUs = 0;

  float tgt[4] = {leftServoDeg, rightServoDeg, morphServoDeg, tailServoDeg};
  const uint32_t nowUs = micros();
  if (!slewInit) {
    for (int i = 0; i < 4; i++) slewDeg[i] = tgt[i];
    slewInit = true;
  } else {
    float dtS = (nowUs - slewLastUs) * 1.0e-6f;
    if (dtS > 0.05f) dtS = 0.05f;
    const float maxStep = SERVO_SLEW_LIMIT_DPS * dtS;
    for (int i = 0; i < 4; i++) {
      const float d = tgt[i] - slewDeg[i];
      if      (d >  maxStep) tgt[i] = slewDeg[i] + maxStep;
      else if (d < -maxStep) tgt[i] = slewDeg[i] - maxStep;
      slewDeg[i] = tgt[i];
    }
  }
  slewLastUs = nowUs;

  leftPwmUs  = (uint16_t)pwmFromDeg(tgt[0]);
  rightPwmUs = (uint16_t)pwmFromDeg(tgt[1]);
  morphPwmUs = (uint16_t)pwmFromDeg(tgt[2]);
  tailPwmUs  = (uint16_t)pwmFromDeg(tgt[3]);
  servoLeft.writeMicroseconds(leftPwmUs);
  servoRight.writeMicroseconds(rightPwmUs);
  servoMorph.writeMicroseconds(morphPwmUs);
  servoTail.writeMicroseconds(tailPwmUs);
}

// ========================= GPS navigation (bank-to-turn) =========================
//
// Third (outermost) guidance layer on top of the cascade PID:
//   heading error = bearing(to target) - ground track(from ESEKF velocity)
//   roll target   = clamp(NAV_KP · heading error, ±NAV_MAX_BANK_DEG)
//
// Ground track comes from ESEKF horizontal velocity (GPS-fused), NOT from
// yaw — yaw is unobservable without a magnetometer and drifts freely.
// Falls back to wings-level (ROLL_TARGET_DEG) whenever navigation
// cannot be trusted: no GPS origin, no fix, too slow, or already close.
//
// Wind-response state machine (navWindState lives in the main .ino because
// logger.ino needs it and precedes this file in the build):
//
//   NORMAL ──(saturated turn + losing ground for NAV_STUCK_TURN_MS)──►
//     PENETRATION (partial sweep raises airspeed, steering continues)
//       ──(closing > PENETRATION_EXIT_CLOSE_MPS)──► NORMAL
//       ──(still losing after PENETRATION_GIVEUP_MS)──► LEVEL_HOLD
//     LEVEL_HOLD (wings level, morph reopened, minimum sink)
//       ──(closing > NAV_RESUME_CLOSE_MPS)──► NORMAL
//
// If ENABLE_PENETRATION_MODE is 0, or AGL is below PENETRATION_MIN_AGL_M,
// the stuck turn goes straight to LEVEL_HOLD (original wind-guard behaviour).
static bool     navSatTurnLive    = false; // saturated no-progress turn in progress
static uint32_t navSatTurnStartMs = 0;
static uint32_t navWindStateMs    = 0;     // entry time of the current wind state

// Penetration is only meaningful while actively steering. If navigation
// becomes unavailable (GPS lost, arrived, too slow) while the sweep is
// engaged, reopen the wings — otherwise the state machine's exit checks
// are skipped and the aircraft would stay swept (high sink) indefinitely.
static void navAbortPenetrationIfActive() {
  if (navWindState == NAV_WIND_PENETRATION) {
    navWindState = NAV_WIND_NORMAL;
    safeSerialPrintln(F("[NAV] Navigation unavailable - penetration sweep reopened."));
  }
}

static float navRollTargetCompute() {
#if ENABLE_GPS_NAVIGATION
  if (!gpsOriginSet || !gps.fix) {
    navAbortPenetrationIfActive();
    return ROLL_TARGET_DEG;
  }

  const float dn = APPROACH_TARGET_N_M - esekf.p[0];
  const float de = APPROACH_TARGET_E_M - esekf.p[1];
  const float dist = sqrtf(dn*dn + de*de);
  if (dist < NAV_ARRIVAL_RADIUS_M) {
    navAbortPenetrationIfActive();
    return ROLL_TARGET_DEG;
  }

  const float groundSpeed = sqrtf(esekf.v[0]*esekf.v[0] + esekf.v[1]*esekf.v[1]);
  if (groundSpeed < NAV_MIN_GROUND_SPEED_MPS) {
    // Track angle untrustworthy at low ground speed, so no steering this
    // frame. Do NOT abort penetration here: passing through zero ground
    // speed is the EXPECTED transient while penetration builds airspeed
    // (blown-back → stopped → forward). Keep the sweep and let the give-up
    // timer decide; without this the sweep would reopen mid-transient and
    // the aircraft would oscillate between penetrating and drifting.
    if (navWindState == NAV_WIND_PENETRATION &&
        (uint32_t)(millis() - navWindStateMs) >= PENETRATION_GIVEUP_MS) {
      navWindState   = NAV_WIND_LEVEL_HOLD;
      navWindStateMs = millis();
      safeSerialPrintln(F("[NAV] Penetration failed - holding wings level."));
    }
    return ROLL_TARGET_DEG;
  }

  // Closing speed: projection of ground velocity onto the target direction.
  // Positive = approaching the target, negative = being pushed away.
  const float vClose = (esekf.v[0]*dn + esekf.v[1]*de) / dist;

  // Level hold: turning was declared futile even with penetration. Stay
  // wings-level (minimum sink) until we are genuinely closing again.
  if (navWindState == NAV_WIND_LEVEL_HOLD) {
    if (vClose > NAV_RESUME_CLOSE_MPS) {
      navWindState = NAV_WIND_NORMAL;
      safeSerialPrintln(F("[NAV] Closing on target again - navigation resumed."));
    } else {
      return ROLL_TARGET_DEG;
    }
  }

  // Penetration: sweep is engaged (morph handled in MODE_GLIDE), airspeed
  // is building. Keep steering — the extra speed is what restores turning
  // authority over the ground track.
  if (navWindState == NAV_WIND_PENETRATION) {
    if (vClose > PENETRATION_EXIT_CLOSE_MPS) {
      navWindState = NAV_WIND_NORMAL;
      safeSerialPrintln(F("[NAV] Penetration succeeded - back to normal glide."));
    } else if ((uint32_t)(millis() - navWindStateMs) >= PENETRATION_GIVEUP_MS) {
      navWindState   = NAV_WIND_LEVEL_HOLD;
      navWindStateMs = millis();
      safeSerialPrintln(F("[NAV] Penetration failed - holding wings level."));
      return ROLL_TARGET_DEG;
    }
    // fall through: continue steering while penetrating
  }

  const float bearingRad = atan2f(de, dn);                 // desired course (N=0, E=+90°)
  const float trackRad   = atan2f(esekf.v[1], esekf.v[0]); // current ground track
  const float hdgErrDeg  = wrapPi(bearingRad - trackRad) * RAD_TO_DEG_F;

  // Positive heading error (target right of track) → positive roll → right bank.
  // If the aircraft turns away from the target, negate NAV_KP_ROLL_PER_HDG_DEG.
  const float cmd = clampfLocal(NAV_KP_ROLL_PER_HDG_DEG * hdgErrDeg,
                                -NAV_MAX_BANK_DEG, NAV_MAX_BANK_DEG);

  // Stuck-turn detection (only from NORMAL): saturated bank AND still
  // losing ground. At 15° bank / ~10 m/s the heading changes ~15°/s, so 5 s
  // of saturated turning sweeps ~75° of heading. If closing speed never went
  // positive in that span, the wind beats our normal glide airspeed.
  const bool saturated = (fabsf(cmd) >= NAV_MAX_BANK_DEG - 0.1f);
  if (navWindState == NAV_WIND_NORMAL && saturated && vClose < 0.0f) {
    if (!navSatTurnLive) {
      navSatTurnLive    = true;
      navSatTurnStartMs = millis();
    } else if ((uint32_t)(millis() - navSatTurnStartMs) >= NAV_STUCK_TURN_MS) {
      navSatTurnLive = false;
#if ENABLE_PENETRATION_MODE
      if (esekf.p[2] > PENETRATION_MIN_AGL_M) {
        navWindState   = NAV_WIND_PENETRATION;
        navWindStateMs = millis();
        safeSerialPrintln(F("[NAV] Wind beats glide speed - engaging penetration sweep."));
        // keep steering this frame; morph target changes in MODE_GLIDE
      } else {
        navWindState   = NAV_WIND_LEVEL_HOLD;
        navWindStateMs = millis();
        safeSerialPrintln(F("[NAV] Wind too strong, too low for penetration - level hold."));
        return ROLL_TARGET_DEG;
      }
#else
      navWindState   = NAV_WIND_LEVEL_HOLD;
      navWindStateMs = millis();
      safeSerialPrintln(F("[NAV] Turn not closing (wind > airspeed?) - holding wings level."));
      return ROLL_TARGET_DEG;
#endif
    }
  } else {
    navSatTurnLive = false;
  }

  return cmd;
#else
  return ROLL_TARGET_DEG;
#endif
}

// Wrapper: stores the commanded roll in navRollCmdDeg for the CSV log.
static float navRollTargetDeg() {
  navRollCmdDeg = navRollTargetCompute();
  return navRollCmdDeg;
}

// ========================= Cascade PID (M3) =========================
//
// Outer loop: angle error (deg) → desired angular rate (deg/s).
//   pidRateTargetRoll/Pitch = -Kp_outer · angleError
//
// Inner loop: rate error (deg/s) → servo deflection (deg).
//   u = Kp_inner · rateError + Ki_inner · integrator
//
// Mixing (wing servos act as elevons):
//   left  = pitch + roll   (symmetric pitch, differential roll)
//   right = pitch - roll
//
// rollTargetDeg comes from navRollTargetDeg() (GPS guidance) or is
// ROLL_TARGET_DEG (wings level) when navigation is unavailable.

static void clearPidState() {
  pidRateIntRoll     = 0.0f;
  pidRateIntPitch    = 0.0f;
  pidRateTargetRoll  = 0.0f;
  pidRateTargetPitch = 0.0f;
  uRollDeg  = 0.0f;
  uPitchDeg = 0.0f;
}

static void updateCascadePid(float dt, float rollTargetDeg) {
  // Outer loop
  const float rollErr  = deadband(rollRad *RAD_TO_DEG_F - rollTargetDeg,    ANGLE_DEADBAND_DEG);
  const float pitchErr = deadband(pitchRad*RAD_TO_DEG_F - PITCH_TARGET_DEG, ANGLE_DEADBAND_DEG);
  pidRateTargetRoll  = clampfLocal(-KP_OUTER_ROLL  * rollErr,
                                   -RATE_TARGET_LIMIT_DPS, RATE_TARGET_LIMIT_DPS);
  pidRateTargetPitch = clampfLocal(-KP_OUTER_PITCH * pitchErr,
                                   -RATE_TARGET_LIMIT_DPS, RATE_TARGET_LIMIT_DPS);

  // Inner loop — subtract the ESEKF-estimated gyro bias before use.
  // Raw imu.gyro still contains bias; esekf.bg tracks it online.
  const float measRollDps  = deadband((imu.gyro[0] - esekf.bg[0])*RAD_TO_DEG_F, RATE_DEADBAND_DPS);
  const float measPitchDps = deadband((imu.gyro[1] - esekf.bg[1])*RAD_TO_DEG_F, RATE_DEADBAND_DPS);
  const float eRoll  = pidRateTargetRoll  - measRollDps;
  const float ePitch = pidRateTargetPitch - measPitchDps;

  pidRateIntRoll  = clampfLocal(pidRateIntRoll  + eRoll *dt,
                                -INTEGRATOR_LIMIT_DEG, INTEGRATOR_LIMIT_DEG);
  pidRateIntPitch = clampfLocal(pidRateIntPitch + ePitch*dt,
                                -INTEGRATOR_LIMIT_DEG, INTEGRATOR_LIMIT_DEG);

  uRollDeg  = clampfLocal(KP_INNER_ROLL *eRoll  + KI_INNER_ROLL *pidRateIntRoll,
                           -CMD_LIMIT_DEG, CMD_LIMIT_DEG);
  uPitchDeg = clampfLocal(KP_INNER_PITCH*ePitch + KI_INNER_PITCH*pidRateIntPitch,
                           -CMD_LIMIT_DEG, CMD_LIMIT_DEG);

  // Elevon mixing: both servos are centred at PID_NEUTRAL_DEG (90°).
  // WING_FLAT_* positions are used only for fold/deploy geometry, not here.
  leftServoDeg  = clampfLocal(PID_NEUTRAL_DEG + uPitchDeg + uRollDeg,
                               SERVO_MIN_DEG, SERVO_MAX_DEG);
  rightServoDeg = clampfLocal(PID_NEUTRAL_DEG + uPitchDeg - uRollDeg,
                               SERVO_MIN_DEG, SERVO_MAX_DEG);
}

// ========================= Mission state machine =========================

static void setFlightMode(FlightMode next) {
  if (next == flightMode) return;
  flightMode   = next;
  modeStartUs  = micros();
  modeElapsedS = 0.0f;

  if (next == MODE_DEPLOYING) {
    // Remember morph position at transition time so ramp starts from the right angle.
    morphDeployStart = morphServoDeg;
  }
  if (next == MODE_GLIDE) {
    clearPidState();
  }
  if (next == MODE_APPROACH) {
    // Close to target now — penetration/hold no longer applies; make sure
    // the wings reopen fully for the approach and heli transition.
    navWindState = NAV_WIND_NORMAL;
  }
  if (next == MODE_HELI_DESCENT) {
    // Seed the descent-rate filter with the CURRENT descent rate, not zero.
    // Seeding with 0 made morphAdj start at -20° (clamped to MORPH_MIN) and
    // the servo step 180° → 70° at entry; seeding with the real value makes
    // the morph transition continuous.
    heliVdFilt = -esekf.v[2];
  }

  serialMutex.lock();
  Serial.print(F("[M] -> "));
  Serial.println(modeName(next));
  serialMutex.unlock();
}

static void updateMission(float dt) {
  modeElapsedS = (micros() - modeStartUs) * 1.0e-6f;

  const float speed = sqrtf(esekf.v[0]*esekf.v[0] +
                            esekf.v[1]*esekf.v[1] +
                            esekf.v[2]*esekf.v[2]);
  // AGL altitude (NEU: up = positive), relative to GPS origin.
  const float agl   = esekf.p[2];
  // Downward velocity (positive = falling).
  const float vDown = -esekf.v[2];

  switch (flightMode) {

    // ----------------------------------------------------------------
    case MODE_CARRIED: {
      // M0: Folded, carried by drone. Hold fold position.
      // Free-fall detector watches for |accel| < 0.3 g for 125 ms.
      leftServoDeg  = WING_FOLD_LEFT_DEG;
      rightServoDeg = WING_FOLD_RIGHT_DEG;
      morphServoDeg = MORPH_FOLD_DEG;
      tailServoDeg  = TAIL_HOLD_DEG;

      // Free-fall detection: in free fall gravity is cancelled;
      // accelerometer reads ≈0 m/s². See FREE_FALL_THRESHOLD/CONFIRM notes.
      const float aN = sqrtf(imu.accel[0]*imu.accel[0] +
                             imu.accel[1]*imu.accel[1] +
                             imu.accel[2]*imu.accel[2]);
      if (aN < FREE_FALL_THRESHOLD_MPS2) {
        if (freeFallCount < FREE_FALL_CONFIRM_FRAMES) freeFallCount++;
      } else {
        freeFallCount = 0;
      }
      if (freeFallCount >= FREE_FALL_CONFIRM_FRAMES) {
        freeFallCount = 0;
        setFlightMode(MODE_FREE_FALL);
      }
      break;
    }

    // ----------------------------------------------------------------
    case MODE_FREE_FALL: {
      // M1: Released, building speed, still folded (no drag = faster spin-up).
      // Deployment is triggered by speed (primary) or altitude floor (failsafe).
      // Both require v[2] < 0 (descending) to prevent accidental trigger
      // while ascending through the same altitude on the way up.
      leftServoDeg  = WING_FOLD_LEFT_DEG;
      rightServoDeg = WING_FOLD_RIGHT_DEG;
      morphServoDeg = MORPH_FOLD_DEG;
      tailServoDeg  = TAIL_HOLD_DEG;

      const bool descending = (esekf.v[2] < -0.5f);

      // Primary: enough speed AND safe altitude remaining for deployment.
      const bool normalOpen = descending &&
                              (speed >= V_OPEN_MIN_MPS) &&
                              (agl   >= H_OPEN_MIN_M);
      // Failsafe: too close to ground — open regardless of speed.
      // See H_FORCE_OPEN_M annotation in constants.
      const bool forceOpen  = descending && (agl <= H_FORCE_OPEN_M);

      if (normalOpen || forceOpen) {
        if (forceOpen && !normalOpen) {
          safeSerialPrintln(F("[M1] FORCE OPEN: altitude floor reached."));
        }
        setFlightMode(MODE_DEPLOYING);
      }
      break;
    }

    // ----------------------------------------------------------------
    case MODE_DEPLOYING: {
      // M2: Morph sweeps from fold angle to fully open in T2_OPEN_S seconds.
      // Left/right wings stay at fold AoA until morph clears to prevent
      // mechanical contact (±30° bias from flat).
      leftServoDeg  = WING_FOLD_LEFT_DEG;
      rightServoDeg = WING_FOLD_RIGHT_DEG;
      tailServoDeg  = TAIL_HOLD_DEG;

      const float progress  = clampfLocal(modeElapsedS / T2_OPEN_S, 0.f, 1.f);
      morphServoDeg = morphDeployStart + progress*(MORPH_OPEN_DEG - morphDeployStart);

      if (modeElapsedS >= T2_OPEN_S) {
        morphServoDeg = MORPH_OPEN_DEG;
        // Hand over at PID_NEUTRAL_DEG so M3's first PID frame (which centres
        // on the same neutral) causes no servo step.
        leftServoDeg  = PID_NEUTRAL_DEG;
        rightServoDeg = PID_NEUTRAL_DEG;
        setFlightMode(MODE_GLIDE);
      }
      break;
    }

    // ----------------------------------------------------------------
    case MODE_GLIDE: {
      // M3: Wings open. Cascade PID holds attitude; roll target comes from
      // GPS bank-to-turn guidance. Morph follows the wind state machine:
      // partial sweep while penetrating a headwind, fully open otherwise.
      // (navRollTargetDeg() updates navWindState, so morph is set after.)
      tailServoDeg  = TAIL_HOLD_DEG;
      updateCascadePid(dt, navRollTargetDeg());
      morphServoDeg = (navWindState == NAV_WIND_PENETRATION)
                        ? PENETRATION_MORPH_DEG
                        : MORPH_OPEN_DEG;

#if ENABLE_APPROACH_PHASES
      if (!gpsOriginSet) break;

      // Altitude floor: below H_FORCE_APPROACH_M there is not enough height
      // to keep gliding toward the target — force the heli transition NOW,
      // regardless of distance. Mirrors H_FORCE_OPEN_M in M1, including the
      // descending gate so ESEKF glitches upward cannot trigger it.
      const bool descendingM3 = (esekf.v[2] < -0.5f);
      if (descendingM3 && agl <= H_FORCE_APPROACH_M) {
        safeSerialPrintln(F("[M3] FORCE APPROACH: altitude floor reached."));
        approachCount = 0;
        setFlightMode(MODE_APPROACH);
        break;
      }

      const float dn = esekf.p[0] - APPROACH_TARGET_N_M;
      const float de = esekf.p[1] - APPROACH_TARGET_E_M;
      if (sqrtf(dn*dn + de*de) <= APPROACH_RADIUS_M) {
        if (approachCount < APPROACH_CONFIRM_CYCLES) approachCount++;
      } else {
        approachCount = 0;
      }
      if (approachCount >= APPROACH_CONFIRM_CYCLES) {
        approachCount = 0;
        setFlightMode(MODE_APPROACH);
      }
#endif
      break;
    }

    // ----------------------------------------------------------------
    case MODE_APPROACH: {
      // M4: Inside approach zone. PID continues; hold 0.5 s then
      // transition to helicopter configuration.
      morphServoDeg = MORPH_OPEN_DEG;
      tailServoDeg  = TAIL_HOLD_DEG;
      updateCascadePid(dt, navRollTargetDeg());

      if (modeElapsedS >= 0.5f) {
        setFlightMode(MODE_HELI_DESCENT);
      }
      break;
    }

    // ----------------------------------------------------------------
    case MODE_HELI_DESCENT: {
      // M5: Wings in autorotation configuration.
      // Morph angle modulates descent rate:
      //   faster descent → increase morph (more blade area → more drag/lift).
      //   slower descent → decrease morph.
      // heliVdFilt is a low-pass of measured downward velocity to filter GPS noise.
      leftServoDeg  = HELI_LEFT_DEG;
      rightServoDeg = HELI_RIGHT_DEG;
      tailServoDeg  = TAIL_HOLD_DEG;

      heliVdFilt += HELI_VD_FILTER_ALPHA * (vDown - heliVdFilt);

      // Dead zone: [NORMAL, FAST] = [2.5, 3.5] m/s → no adjustment.
      // Outside that band, adjustment is proportional to deviation from the
      // respective boundary — continuous at both edges (morphAdj = 0 at each).
      float morphAdj = 0.0f;
      if (heliVdFilt > HELI_FAST_DESCENT_MPS) {
        // Descending too fast: open morph further to add rotor area, slow down.
        morphAdj = HELI_GAIN_DEG_PER_MPS * (heliVdFilt - HELI_FAST_DESCENT_MPS);
      } else if (heliVdFilt < HELI_NORMAL_DESCENT_MPS) {
        // Descending too slowly: close morph to reduce area, speed up.
        morphAdj = HELI_GAIN_DEG_PER_MPS * (heliVdFilt - HELI_NORMAL_DESCENT_MPS);
      }
      morphServoDeg = clampfLocal(
          HELI_MORPH_BASE_DEG + morphAdj,
          HELI_MORPH_MIN_DEG,
          HELI_MORPH_MAX_DEG);
      break;
    }

    default: break;
  }

  writeAllServos();
}
