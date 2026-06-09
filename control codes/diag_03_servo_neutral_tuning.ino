/*
  diag_03_servo_v2.ino
  Layer 3 of the MMAV bench-test suite: servo path validation (acceptance).

  VALIDATES the full command path of 64_gliding_pid_only.ino:
    raw cmd (incidence deg about neutral)
      -> CMD_LPF (command smoothing, dynamics only)
      -> per-side map: servo = NEUTRAL[side] + REVSIGN[side]*GAIN[side]*incidence
      -> rateLimit (240 deg/s slew)
      -> pwmFromDeg (deg -> us)
      -> servo.writeMicroseconds()
    NEUTRAL_LEFT/RIGHT and GAIN_LEFT/RIGHT are per-side sub-trim calibration.

  MEASUREMENT REALITY
    The onboard IMU is on the FUSELAGE and cannot see the wing surfaces.
    Static modes (l/r/b/s/c/m) DRIVE clean held commands and print exactly what
    they command (dev, scaled deg, PWM); YOU read the surface angle vs fuselage
    with a protractor and pair commanded-vs-measured. Each point holds until you
    press a key.
    p (step response) logs the FIRMWARE command path at 100 Hz -> t10/t50/t90 and
    rate-limit saturation. That is firmware lag only; physical servo/linkage lag
    needs high-speed video (film the surface during p and read frames).

    Note: only CMD_LPF sits in the actuation path. The gyro RATE_LPF is in the
    estimator / D-term path, not the command->servo path, so it is not in p.

  Config IDENTICAL to 64. Numbers via Print.print (no snprintf %f).

  SERIAL MENU (115200 baud):
    n = all neutral                 q = quick smoke (neutral, +-10 L/R/both)
    l = LEFT-only sweep             r = RIGHT-only sweep
    b = BOTH symmetric sweep        s = sign check (+-roll, +-pitch)
    c = command-clamp check         p = firmware step-response log
    m = travel-limit ramp           h = help
*/

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// ============== config: IDENTICAL to 64_gliding_pid_only.ino ==============
static const uint32_t SERIAL_BAUD    = 115200;
static const uint32_t SERIAL_WAIT_MS = 1500;
static const float    DT_NOMINAL     = 0.01f;

static const int   SERVO_LEFT_PIN     = 4;
static const int   SERVO_RIGHT_PIN    = 3;
static const bool  SERVO_LEFT_REVERSE = true;
static const bool  SERVO_RIGHT_REVERSE= false;
static const float SERVO_LEFT_GAIN    = 1.00f;
static const float SERVO_RIGHT_GAIN   = 1.00f;
static const float SERVO_LEFT_NEUTRAL_DEG  = 103.5f;  // measured (jog): raw servo deg for LEFT  wing 0 incidence
static const float SERVO_RIGHT_NEUTRAL_DEG =  90.0f;  // measured (jog): raw servo deg for RIGHT wing 0 incidence
static const int   SERVO_MIN_US  = 1000;
static const int   SERVO_MAX_US  = 2000;
static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 180.0f;
static const float AOA_NEUTRAL_DEG          = 90.0f;
static const float AOA_RATE_LIMIT_DEG_PER_S = 240.0f;
static const float AOA_CMD_LIMIT_DEG        = 60.0f;
static const float CMD_LPF_ALPHA            = 0.55f;   // command smoothing (actuation path)

// ============== test-only config ==============
static const float    SETPOINTS[]  = { -60,-45,-30,-15,-10,-5, 0, 5, 10, 15, 30, 45, 60 };
static const uint8_t  N_SET        = sizeof(SETPOINTS) / sizeof(SETPOINTS[0]);
static const uint32_t SETTLE_MS    = 800;
static const float    SIGN_DEG     = 15.0f;
static const float    QUICK_DEG    = 10.0f;
static const uint32_t QUICK_HOLD_MS= 1000;
static const float    CLAMP_REQS[] = { 80, -80, 100, -100 };
static const uint8_t  N_CLAMP      = sizeof(CLAMP_REQS) / sizeof(CLAMP_REQS[0]);
static const float    STEP_DEG     = 15.0f;   // step amplitude for p/P
static const uint32_t STEP_AT_MS   = 100;     // baseline before the step
static const uint32_t STEP_LOG_MS  = 900;     // total log window
static const uint16_t STEP_MAX_ROWS= 140;     // RAM rows (>= STEP_LOG_MS/10)
static const uint32_t STEP_PERIOD_US = 10000; // 100 Hz pacing for the step log
static const float    LIMIT_STEP   = 2.0f;
static const float    LIMIT_MAX_DEG= 70.0f;   // defensive ceiling; raise only if linkage allows
static const uint32_t LIMIT_HOLD_MS= 300;

// ============== globals ==============
static Servo servoLeft, servoRight;
static float leftServoDeg  = SERVO_LEFT_NEUTRAL_DEG;   // rate-limiter state (final servo deg)
static float rightServoDeg = SERVO_RIGHT_NEUTRAL_DEG;
static float leftCmdFiltDeg = 0.0f, rightCmdFiltDeg = 0.0f;  // CMD_LPF state (deviation)

// step-response RAM log: no Serial during capture (clean 100 Hz), dumped afterwards
struct StepRow {
  uint32_t t_us;
  float    raw_l, raw_r, filt_l, filt_r, servo_l, servo_r;
  uint16_t pwm_l, pwm_r;
};
static StepRow stepLog[STEP_MAX_ROWS];

// ============== flight writeServos path (verbatim from 64) ==============
static float clampfLocal(float x, float lo, float hi) { if (x < lo) return lo; if (x > hi) return hi; return x; }
static float rateLimit(float target, float current, float maxStep) { return current + clampfLocal(target - current, -maxStep, maxStep); }
static int   pwmFromDeg(float deg) {
  const float d = clampfLocal(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  const float s = (d - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
  return (int)lroundf(SERVO_MIN_US + s * (SERVO_MAX_US - SERVO_MIN_US));
}
// per-side calibration:  servo = NEUTRAL[side] + REVSIGN[side] * GAIN[side] * incidence
static int   revSign(bool reverse) { return reverse ? -1 : +1; }
static float sideServoOut(float incidence, float neutral, float gain, bool reverse) {
  return clampfLocal(neutral + revSign(reverse) * gain * incidence, SERVO_MIN_DEG, SERVO_MAX_DEG);
}
static void  writeServos(float leftDeg, float rightDeg, float dt) {
  const float incL = leftDeg  - AOA_NEUTRAL_DEG;   // callers pass AOA_NEUTRAL_DEG + incidence
  const float incR = rightDeg - AOA_NEUTRAL_DEG;
  const float step = AOA_RATE_LIMIT_DEG_PER_S * dt;
  const float l = sideServoOut(incL, SERVO_LEFT_NEUTRAL_DEG,  SERVO_LEFT_GAIN,  SERVO_LEFT_REVERSE);
  const float r = sideServoOut(incR, SERVO_RIGHT_NEUTRAL_DEG, SERVO_RIGHT_GAIN, SERVO_RIGHT_REVERSE);
  leftServoDeg  = rateLimit(l, leftServoDeg, step);
  rightServoDeg = rateLimit(r, rightServoDeg, step);
  servoLeft.writeMicroseconds((uint16_t)pwmFromDeg(leftServoDeg));
  servoRight.writeMicroseconds((uint16_t)pwmFromDeg(rightServoDeg));
}

// ============== helpers ==============
static void flushSerial() { while (Serial.available() > 0) Serial.read(); }
static void waitKey()     { flushSerial(); while (Serial.available() <= 0) { /* hold */ } Serial.read(); }

static void commandHold(float devL, float devR, uint32_t settleMs) {
  const float tgtL = AOA_NEUTRAL_DEG + devL, tgtR = AOA_NEUTRAL_DEG + devR;
  const uint32_t t0 = millis();
  while (millis() - t0 < settleMs) { writeServos(tgtL, tgtR, DT_NOMINAL); delay(10); }
}

// Full traceability INCLUDING the gain stage (matches writeServos).
static void printTarget(const char *tag, float devL, float devR) {
  const float outL = sideServoOut(devL, SERVO_LEFT_NEUTRAL_DEG,  SERVO_LEFT_GAIN,  SERVO_LEFT_REVERSE);
  const float outR = sideServoOut(devR, SERVO_RIGHT_NEUTRAL_DEG, SERVO_RIGHT_GAIN, SERVO_RIGHT_REVERSE);
  const int pwmL = pwmFromDeg(outL);
  const int pwmR = pwmFromDeg(outR);
  Serial.print(F("#   ")); Serial.print(tag);
  Serial.print(F(" | L inc=")); Serial.print(devL, 1); Serial.print(F(" servo ")); Serial.print(outL, 1); Serial.print(F(" pwm ")); Serial.print(pwmL);
  Serial.print(F(" | R inc=")); Serial.print(devR, 1); Serial.print(F(" servo ")); Serial.print(outR, 1); Serial.print(F(" pwm ")); Serial.println(pwmR);
}

// ============== static sweeps ==============
static void runSweep(bool isLeft) {
  const char *side = isLeft ? "LEFT" : "RIGHT";
  Serial.print(F("# ")); Serial.print(side); Serial.println(F("-only static sweep (other side neutral)."));
  Serial.println(F("# Read BOTH surface angles vs fuselage at each point, record, press any key."));
  for (uint8_t k = 0; k < N_SET; k++) {
    const float dev = SETPOINTS[k];
    commandHold(isLeft ? dev : 0.0f, isLeft ? 0.0f : dev, SETTLE_MS);
    Serial.print(F("# point ")); Serial.print(k + 1); Serial.print(F("/")); Serial.print(N_SET); Serial.println(F(":"));
    printTarget(side, isLeft ? dev : 0.0f, isLeft ? 0.0f : dev);
    Serial.println(F("#   -> measure & press any key"));
    waitKey();
  }
  commandHold(0, 0, SETTLE_MS);
  Serial.println(F("# sweep done, neutral."));
}

static void runBoth() {
  Serial.println(F("# BOTH symmetric static sweep (pitch-axis geometry)."));
  Serial.println(F("# Read BOTH surface angles, record, press any key."));
  for (uint8_t k = 0; k < N_SET; k++) {
    const float dev = SETPOINTS[k];
    commandHold(dev, dev, SETTLE_MS);
    Serial.print(F("# point ")); Serial.print(k + 1); Serial.print(F("/")); Serial.print(N_SET); Serial.println(F(":"));
    printTarget("BOTH", dev, dev);
    Serial.println(F("#   -> measure & press any key"));
    waitKey();
  }
  commandHold(0, 0, SETTLE_MS);
  Serial.println(F("# both-sweep done, neutral."));
}

// ============== quick smoke ==============
static void runQuick() {
  Serial.println(F("# QUICK smoke: neutral, +-10 L/R/both (auto holds). Watch each surface moves correctly."));
  const float seqL[] = { 0,  QUICK_DEG, -QUICK_DEG, 0, 0,          0,          0, QUICK_DEG, -QUICK_DEG, 0 };
  const float seqR[] = { 0,  0,          0,         0, QUICK_DEG, -QUICK_DEG,  0, QUICK_DEG, -QUICK_DEG, 0 };
  const char *tag[]  = { "neutral","L+10","L-10","neutral","R+10","R-10","neutral","both+10","both-10","neutral" };
  for (uint8_t k = 0; k < 10; k++) {
    commandHold(seqL[k], seqR[k], QUICK_HOLD_MS);
    printTarget(tag[k], seqL[k], seqR[k]);
  }
  Serial.println(F("# quick smoke done, neutral."));
}

// ============== sign / direction check (flight mixing) ==============
static void signCase(const char *name, float uRoll, float uPitch, const __FlashStringHelper *expect) {
  const float lc = clampfLocal(uPitch + uRoll, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);
  const float rc = clampfLocal(uPitch - uRoll, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);
  commandHold(lc, rc, SETTLE_MS);
  Serial.print(F("# ")); Serial.println(name);
  printTarget(name, lc, rc);
  Serial.print(F("#   EXPECT: ")); Serial.println(expect);
  Serial.println(F("#   Confirm physical direction vs your convention; press any key."));
  waitKey();
}
static void runSign() {
  Serial.println(F("# SIGN CHECK. mixing: leftCmd=uPitch+uRoll, rightCmd=uPitch-uRoll."));
  signCase("+ROLL",  +SIGN_DEG, 0.0f, F("L & R incidence OPPOSITE (differential = roll)"));
  signCase("-ROLL",  -SIGN_DEG, 0.0f, F("L & R OPPOSITE, mirror of +ROLL"));
  signCase("+PITCH", 0.0f, +SIGN_DEG, F("L & R SAME direction (symmetric = pitch)"));
  signCase("-PITCH", 0.0f, -SIGN_DEG, F("L & R SAME, mirror of +PITCH"));
  commandHold(0, 0, SETTLE_MS);
  Serial.println(F("# sign check done, neutral."));
}

// ============== command-clamp check ==============
static void runClamp() {
  Serial.print(F("# CLAMP CHECK: requests beyond +-")); Serial.print(AOA_CMD_LIMIT_DEG, 0);
  Serial.println(F(" deg must clamp. Confirm surface does NOT exceed the limit position; press key."));
  for (uint8_t k = 0; k < N_CLAMP; k++) {
    const float req = CLAMP_REQS[k];
    const float cl  = clampfLocal(req, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);
    commandHold(cl, cl, SETTLE_MS);
    Serial.print(F("# request both dev=")); Serial.print(req, 0);
    Serial.print(F(" -> clamped ")); Serial.println(cl, 1);
    printTarget("CLAMP", cl, cl);
    waitKey();
  }
  // post-mixing clamp example
  {
    const float uRoll = 80.0f, uPitch = 0.0f;
    const float lc = clampfLocal(uPitch + uRoll, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);
    const float rc = clampfLocal(uPitch - uRoll, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);
    commandHold(lc, rc, SETTLE_MS);
    Serial.print(F("# mixing uRoll=+80 -> leftCmd ")); Serial.print(lc, 1);
    Serial.print(F(", rightCmd ")); Serial.println(rc, 1);
    printTarget("MIX-CLAMP", lc, rc);
    waitKey();
  }
  commandHold(0, 0, SETTLE_MS);
  Serial.println(F("# clamp check done, neutral."));
}

// ============== firmware step response (p = symmetric, P = differential) ==============
static void runStep(bool differential) {
  // clean neutral start
  leftServoDeg = rightServoDeg = AOA_NEUTRAL_DEG;
  leftCmdFiltDeg = rightCmdFiltDeg = 0.0f;
  writeServos(AOA_NEUTRAL_DEG, AOA_NEUTRAL_DEG, DT_NOMINAL);
  delay(300);

  // capture into RAM with NO Serial in the loop, MEASURED dt, 100 Hz busy-wait pacing
  uint16_t n = 0;
  const uint32_t t0 = micros();
  uint32_t lastUs = t0;
  while ((uint32_t)(micros() - t0) < STEP_LOG_MS * 1000UL && n < STEP_MAX_ROWS) {
    const uint32_t loopStart = micros();
    const float dt = (loopStart - lastUs) * 1.0e-6f; lastUs = loopStart;
    const bool stepOn = (uint32_t)(loopStart - t0) >= STEP_AT_MS * 1000UL;
    const float rawL = stepOn ? STEP_DEG : 0.0f;
    const float rawR = differential ? -rawL : rawL;
    leftCmdFiltDeg  += CMD_LPF_ALPHA * (rawL - leftCmdFiltDeg);
    rightCmdFiltDeg += CMD_LPF_ALPHA * (rawR - rightCmdFiltDeg);
    writeServos(AOA_NEUTRAL_DEG + leftCmdFiltDeg, AOA_NEUTRAL_DEG + rightCmdFiltDeg, dt);   // measured dt
    StepRow &row = stepLog[n];
    row.t_us = loopStart - t0;
    row.raw_l = rawL; row.raw_r = rawR;
    row.filt_l = leftCmdFiltDeg; row.filt_r = rightCmdFiltDeg;
    row.servo_l = leftServoDeg; row.servo_r = rightServoDeg;
    row.pwm_l = (uint16_t)pwmFromDeg(leftServoDeg);   // leftServoDeg is already final servo deg
    row.pwm_r = (uint16_t)pwmFromDeg(rightServoDeg);
    n++;
    while ((uint32_t)(micros() - loopStart) < STEP_PERIOD_US) { /* pace 100 Hz, no Serial */ }
  }
  commandHold(0, 0, SETTLE_MS);

  // dump afterwards
  Serial.print(F("# STEP ")); Serial.print(differential ? F("DIFFERENTIAL (L+,R-)") : F("SYMMETRIC (L+,R+)"));
  Serial.print(F(": raw -> CMD_LPF a=")); Serial.print(CMD_LPF_ALPHA, 2);
  Serial.print(F(" -> rate ")); Serial.print(AOA_RATE_LIMIT_DEG_PER_S, 0);
  Serial.println(F(" deg/s. measured dt, RAM-logged. +15 deg at t=100ms."));
  Serial.println(F("k,t_ms,raw_l,raw_r,filt_l,filt_r,servo_l_deg,servo_r_deg,pwm_l,pwm_r"));
  for (uint16_t i = 0; i < n; i++) {
    StepRow &row = stepLog[i];
    Serial.print(i); Serial.print(',');
    Serial.print(row.t_us / 1000.0, 2); Serial.print(',');
    Serial.print(row.raw_l, 2); Serial.print(','); Serial.print(row.raw_r, 2); Serial.print(',');
    Serial.print(row.filt_l, 3); Serial.print(','); Serial.print(row.filt_r, 3); Serial.print(',');
    Serial.print(row.servo_l, 3); Serial.print(','); Serial.print(row.servo_r, 3); Serial.print(',');
    Serial.print(row.pwm_l); Serial.print(','); Serial.println(row.pwm_r);
  }
  const double dur = n ? stepLog[n - 1].t_us / 1.0e6 : 0.0;
  Serial.print(F("# ")); Serial.print(n); Serial.print(F(" rows, ~"));
  Serial.print(dur > 0 ? n / dur : 0.0, 1);
  Serial.println(F(" Hz. Extract t10/t50/t90 of servo_deg vs step at t=100ms (firmware lag; film surface for physical)."));
}

// ============== travel-limit ramp ==============
static void rampLimit(bool isLeft, bool positive) {
  const char *side = isLeft ? "LEFT" : "RIGHT";
  const char *dir  = positive ? "+" : "-";
  Serial.print(F("# LIMIT ramp ")); Serial.print(side); Serial.print(F(" ")); Serial.print(dir);
  Serial.println(F(": ramps slowly. Press any key the MOMENT the surface stops moving."));
  flushSerial();
  const float stepDeg = positive ? LIMIT_STEP : -LIMIT_STEP;
  float dev = 0.0f; bool stopped = false;
  while (fabsf(dev) <= LIMIT_MAX_DEG) {
    commandHold(isLeft ? dev : 0.0f, isLeft ? 0.0f : dev, LIMIT_HOLD_MS);
    Serial.print(F("#   ")); Serial.print(side); Serial.print(F(" dev=")); Serial.println(dev, 1);
    if (Serial.available() > 0) { Serial.read(); stopped = true; break; }
    dev += stepDeg;
  }
  Serial.print(F("# -> ")); Serial.print(side); Serial.print(F(" ")); Serial.print(dir);
  Serial.print(stopped ? F(" stop at commanded dev ") : F(" ramp ceiling at dev "));
  Serial.print(dev, 1); Serial.println(F(" deg. Measure actual angle, then press any key."));
  waitKey();
  commandHold(0, 0, SETTLE_MS);
}
static void runLimit() {
  Serial.println(F("# TRAVEL-LIMIT ramp: L+, L-, R+, R-."));
  Serial.println(F("# CAUTION: goal is the SAFE mechanical range, not max throw. Check horn/linkage first."));
  Serial.println(F("# Press a key at the FIRST sign of resistance / sound change. Never force to the ceiling."));
  rampLimit(true, true); rampLimit(true, false); rampLimit(false, true); rampLimit(false, false);
  Serial.println(F("# limit sweep done, neutral."));
}

// ============== NEUTRAL finder (jog raw servo angle until wing = 0 deg) ==============
// Writes a RAW servo angle directly (no gain, no reverse, no rate limit) to the
// selected side, so the displayed angle that levels the wing IS that side's NEUTRAL
// in the per-side formula:  servo = NEUTRAL[side] + REVSIGN[side]*GAIN[side]*incidence.
// This mode only FINDS the value; you then hardcode it as a constant. It does not
// persist by itself, and it does NOT change the normal n/l/r/b paths.
static void jogPrint(char side, float angL, float angR) {
  Serial.print(F("# side=")); Serial.print(side);
  Serial.print(F("  L=")); Serial.print(angL, 1);
  Serial.print(F("  R=")); Serial.println(angR, 1);
}
static void runJog() {
  static float angL = AOA_NEUTRAL_DEG, angR = AOA_NEUTRAL_DEG;  // raw servo deg
  char side = 'L';
  servoLeft.writeMicroseconds((uint16_t)pwmFromDeg(angL));
  servoRight.writeMicroseconds((uint16_t)pwmFromDeg(angR));
  Serial.println(F("# --- JOG: find per-side NEUTRAL (level each wing to 0 deg) ---"));
  Serial.println(F("#   l/r select side | +/- =0.5deg | ]/[ =5deg | p print | x exit"));
  Serial.println(F("#   raw angle (no gain/reverse). Protractor on chord vs fuselage. 0deg = NEUTRAL."));
  jogPrint(side, angL, angR);
  flushSerial();
  for (;;) {
    if (Serial.available() <= 0) { delay(5); continue; }
    char c = (char)Serial.read();
    bool show = true;
    float *a = (side == 'L') ? &angL : &angR;
    switch (c) {
      case 'l': side = 'L'; break;
      case 'r': side = 'R'; break;
      case '+': case '=': *a += 0.5f; break;
      case '-': case '_': *a -= 0.5f; break;
      case ']': *a += 5.0f; break;
      case '[': *a -= 5.0f; break;
      case 'p': break;
      case 'x': case 'q':
        Serial.println(F("# JOG done. Hardcode these (per-side formula):"));
        Serial.print(F("#   SERVO_LEFT_NEUTRAL_DEG  = ")); Serial.println(angL, 1);
        Serial.print(F("#   SERVO_RIGHT_NEUTRAL_DEG = ")); Serial.println(angR, 1);
        Serial.println(F("# Put these in SERVO_LEFT/RIGHT_NEUTRAL_DEG and re-flash to apply."));
        return;
      case '\n': case '\r': show = false; break;
      default: show = false; Serial.println(F("# keys: l r + - ] [ p x")); break;
    }
    angL = clampfLocal(angL, SERVO_MIN_DEG, SERVO_MAX_DEG);
    angR = clampfLocal(angR, SERVO_MIN_DEG, SERVO_MAX_DEG);
    servoLeft.writeMicroseconds((uint16_t)pwmFromDeg(angL));
    servoRight.writeMicroseconds((uint16_t)pwmFromDeg(angR));
    if (show) jogPrint(side, angL, angR);
  }
}

static void printMenu() {
  Serial.println(F("# --- diag_03 servo path validation (protractor, airframe fixed) ---"));
  Serial.print(F("# neutralL=")); Serial.print(SERVO_LEFT_NEUTRAL_DEG, 1);
  Serial.print(F(" neutralR=")); Serial.print(SERVO_RIGHT_NEUTRAL_DEG, 1);
  Serial.print(F(" cmdlim=+-")); Serial.print(AOA_CMD_LIMIT_DEG, 0);
  Serial.print(F(" L_rev=")); Serial.print(SERVO_LEFT_REVERSE);
  Serial.print(F(" R_rev=")); Serial.print(SERVO_RIGHT_REVERSE);
  Serial.print(F(" gainL=")); Serial.print(SERVO_LEFT_GAIN, 2);
  Serial.print(F(" gainR=")); Serial.println(SERVO_RIGHT_GAIN, 2);
  Serial.println(F("#   n neutral     q quick smoke"));
  Serial.println(F("#   l L sweep     r R sweep     b both sweep"));
  Serial.println(F("#   s sign(+-roll,+-pitch)      c clamp check"));
  Serial.println(F("#   p step symmetric  P step differential"));
  Serial.println(F("#   m limit ramp      j jog NEUTRAL   h help"));
  Serial.println(F("# Fix airframe; surfaces WILL move. Measure vs fuselage with protractor."));
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < SERIAL_WAIT_MS) delay(10);
  servoLeft.attach(SERVO_LEFT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoRight.attach(SERVO_RIGHT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  writeServos(AOA_NEUTRAL_DEG, AOA_NEUTRAL_DEG, DT_NOMINAL);
  printMenu();
}

void loop() {
  if (Serial.available() <= 0) return;
  char c = (char)Serial.read();
  switch (c) {
    case 'n': commandHold(0, 0, SETTLE_MS); Serial.println(F("# neutral.")); break;
    case 'q': runQuick();      break;
    case 'l': runSweep(true);  break;
    case 'r': runSweep(false); break;
    case 'b': runBoth();       break;
    case 's': runSign();       break;
    case 'c': runClamp();      break;
    case 'p': runStep(false);  break;
    case 'P': runStep(true);   break;
    case 'm': runLimit();      break;
    case 'j': runJog();        break;
    case 'h': printMenu();     break;
    case '\n': case '\r': break;
    default: Serial.println(F("# unknown command (h for help)")); break;
  }
}
