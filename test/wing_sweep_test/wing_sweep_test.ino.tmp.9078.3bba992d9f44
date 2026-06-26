#include <Servo.h>

/*
  Wing sweep mechanism test - binary fold / unfold toggled by a momentary button.

  Board: Arduino Nano RP2040 Connect
  FQBN:  arduino:mbed_nano:nanorp2040connect

  WHAT IT DOES
    Each button press toggles the wings between FOLDED and UNFOLDED.
    The sweep is driven by a CONTINUOUS-rotation servo (no angle feedback), so a
    fold/unfold is a timed spin (DEMO values below). This is open-loop and drifts
    with load - see the LIMITATION note at the sweep constants; a positional servo
    is recommended for flight. The AoA servos ARE positional (fixed flat/folded us).

    Hardware quirk handled here: the AoA surfaces must sit at 90deg of incidence
    while the wing is folded (so they clear the fold path), and return to flat
    (0deg incidence) when unfolded. Ordering is chosen so the AoA servos are ALWAYS
    at 90deg during the actual sweep motion:
      FOLD   : AoA -> 90deg, then sweep in.
      UNFOLD : sweep out (AoA still 90deg), then AoA -> 0deg.

  BUTTON / STATE
    Momentary button on A2 (board slot "10"), INPUT_PULLUP, pressed = LOW
    (short to GND). State (folded/unfolded) lives in RAM and is read live, so no
    reset-survival trickery is needed. Debounced, with a cooldown lockout so a
    bouncy or double tap can't fire a second toggle mid-move.

    (History: an earlier version toggled on the RST button via the RP2040 watchdog
    SCRATCH registers. Tested 2026-06-25 - SCRATCH does NOT survive a reset on this
    board/mbed core, so that approach was dropped in favor of this GPIO button.)

  BOOT BEHAVIOR
    Assumes UNFOLDED on boot: sets AoA flat and does NOT spin the sweep servo (no
    position feedback, so we must not guess-move it). Set the wings to UNFOLDED by
    hand once to match this assumption.
*/

// ----------------------------- Pin map -----------------------------
static const int SWEEP_PIN     = 5;   // D5  - continuous-rotation sweep servo
static const int AOA_LEFT_PIN  = 4;   // D4  - left  wing AoA servo (matches flight code)
static const int AOA_RIGHT_PIN = 3;   // D3  - right wing AoA servo (matches flight code)
static const int BUTTON_PIN    = A2;  // board slot "10" - fold/unfold button (to GND)

// ----------------------------- Button -----------------------------
static const uint32_t DEBOUNCE_MS = 40;    // contact debounce
static const uint32_t COOLDOWN_MS = 3000;  // ignore presses for this long after a toggle

// ------------------------- Sweep servo (continuous) -------------------------
// DEMO VALUES (2026-06-26), tuned by eye on the bench with wings OFF:
//   speed = offset 200us from neutral (1300 fold / 1700 unfold)
//   fold ~800ms ~= 90deg, unfold ~1270ms ~= 90deg (unfold direction runs slower)
//
// LIMITATION - this is a CONTINUOUS-rotation servo, so motion is OPEN-LOOP:
// travel = speed x time with NO position feedback. The actual angle drifts with
// load (measured fold ~45deg vs unfold ~25deg for similar times -> net drift each
// cycle). NOT reliable for flight, where loads vary. RECOMMENDATION: swap to a
// POSITIONAL (metal-gear) servo that holds an absolute commanded angle regardless
// of load. The timings below are demo-only and will need re-tuning on any change.
static const int SWEEP_NEUTRAL_US = 1500;  // stop point; nudge if it creeps while stopped
static const int SWEEP_FOLD_US    = 1300;  // CW,  below neutral = fold in   (slower: offset 200, was 1000)
static const int SWEEP_UNFOLD_US  = 1700;  // CCW, above neutral = unfold out (slower: offset 200, was 2000)
static const int SWEEP_FOLD_MS    = 800;   // DEMO ~90deg fold   (measured 400ms -> ~45deg, so ~2x)
static const int SWEEP_UNFOLD_MS  = 1270;  // DEMO ~90deg unfold (measured 350ms -> ~25deg; slower dir)
// (For a guarded blind first bench cycle, temporarily drop these ~30% so the sweep
//  undershoots 90deg and can't hit a hard stop, then restore to the values above.)
// If the wing sweeps the WRONG way, swap SWEEP_FOLD_US/SWEEP_UNFOLD_US AND the
// matching MS values (the speeds are direction-specific).

// ------------------------- AoA servos -------------------------
// Fold/unfold drives each AoA servo to a fixed FLAT or FOLDED pulse (no degree
// math). Attach range widened to 500-2500us so they can reach the folded extreme -
// the old 1000-2000us range CAPPED travel, which is why they only reached ~30deg.
static const int AOA_MIN_US = 500;   // attach min - full mechanical travel
static const int AOA_MAX_US = 2500;  // attach max

static const int AOA_LEFT_FLAT_US  = 1575;  // measured flat / 0 incidence (left)
static const int AOA_RIGHT_FLAT_US = 1500;  // measured flat / 0 incidence (right)

// FOLDED: surfaces rotated to ~90deg so they clear each other. Left servo reversed
// (folded = LOWER pulse), right normal (folded = HIGHER pulse). TUNE toward the
// extremes (500 / 2500) until they hit 90deg / clear; back off if a servo strains.
static const int AOA_LEFT_FOLDED_US  = 600;
static const int AOA_RIGHT_FOLDED_US = 2400;

static const int AOA_SETTLE_MS = 600;  // let AoA reach folded before sweeping

// ----------------------------- State -----------------------------
static bool folded = false;          // start UNFOLDED
static int  btnStable = HIGH;        // released = HIGH (pullup)
static int  btnLastRead = HIGH;
static uint32_t btnLastChange = 0;
static uint32_t lastToggleMs = 0;
static uint32_t pressCount = 0;

Servo sweepServo;
Servo aoaLeft;
Servo aoaRight;

// ----------------------------- helpers -----------------------------
static void setAoa(int leftUs, int rightUs) {
  aoaLeft.writeMicroseconds(leftUs);
  aoaRight.writeMicroseconds(rightUs);
  if (Serial) {
    Serial.print(F("# AoA -> left ")); Serial.print(leftUs);
    Serial.print(F("us, right ")); Serial.print(rightUs); Serial.println(F("us"));
  }
}
static void aoaFlat()   { setAoa(AOA_LEFT_FLAT_US,   AOA_RIGHT_FLAT_US); }
static void aoaFolded() { setAoa(AOA_LEFT_FOLDED_US, AOA_RIGHT_FOLDED_US); }

// Timed sweep spin for a 90deg travel at the calibrated per-direction rate, then stop.
static void sweep90(int spinUs, int spinMs) {
  sweepServo.writeMicroseconds(spinUs);
  delay(spinMs);
  sweepServo.writeMicroseconds(SWEEP_NEUTRAL_US);  // stop
}

static void doFold() {
  if (Serial) Serial.println(F("# FOLD: AoA -> 90deg, then sweep in"));
  digitalWrite(LED_BUILTIN, HIGH);
  aoaFolded();                                // clear the fold path first
  delay(AOA_SETTLE_MS);
  sweep90(SWEEP_FOLD_US, SWEEP_FOLD_MS);       // AoA stays at 90deg through the sweep
  aoaFolded();                                 // re-assert AoA (D4 shares a PWM slice with sweep D5)
  digitalWrite(LED_BUILTIN, LOW);
}

static void doUnfold() {
  if (Serial) Serial.println(F("# UNFOLD: sweep out, then AoA -> flat"));
  digitalWrite(LED_BUILTIN, HIGH);
  sweep90(SWEEP_UNFOLD_US, SWEEP_UNFOLD_MS);   // AoA still folded during the sweep
  aoaFlat();                                   // drop to flat only once swept out
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 1500) delay(10);

  // Attach AoA servos (positional).
  aoaLeft.attach(AOA_LEFT_PIN, AOA_MIN_US, AOA_MAX_US);
  aoaRight.attach(AOA_RIGHT_PIN, AOA_MIN_US, AOA_MAX_US);

  // Attach sweep servo with the re-attach "kick" - first attach after reset can
  // fail silently on this Servo lib for the continuous servo.
  sweepServo.attach(SWEEP_PIN, 500, 2500);
  delay(50);
  sweepServo.detach();
  delay(50);
  sweepServo.attach(SWEEP_PIN, 500, 2500);
  sweepServo.writeMicroseconds(SWEEP_NEUTRAL_US);  // ensure stopped
  delay(300);

  // Boot UNFOLDED: set AoA flat, do NOT spin the sweep servo (no position feedback).
  folded = false;
  if (Serial) Serial.println(F("# Boot: assuming UNFOLDED. Set wings unfolded by hand. AoA -> flat."));
  aoaFlat();

  if (Serial) Serial.println(F("# Ready. Press the A2 button to toggle fold/unfold."));
}

void loop() {
  // Debounced, cooldown-gated button edge.
  int read = digitalRead(BUTTON_PIN);
  if (read != btnLastRead) {
    btnLastChange = millis();
    btnLastRead = read;
  }
  if (millis() - btnLastChange > DEBOUNCE_MS && read != btnStable) {
    btnStable = read;
    if (btnStable == LOW) {                 // press edge (release does nothing)
      if (pressCount > 0 && millis() - lastToggleMs < COOLDOWN_MS) {
        if (Serial) Serial.println(F("# press ignored (cooldown)"));
      } else {
        pressCount++;
        lastToggleMs = millis();
        if (folded) { folded = false; doUnfold(); }
        else        { folded = true;  doFold();   }
        lastToggleMs = millis();            // restart cooldown AFTER the move completes
        if (Serial) {
          Serial.print(F("# state now: "));
          Serial.println(folded ? F("FOLDED") : F("UNFOLDED"));
        }
      }
    }
  }

  // Servos hold their last commanded pulse in hardware, so do NOT re-write the
  // sweep here. D4 (AoA-left) and D5 (sweep) share an RP2040 PWM slice; hammering
  // the sweep write in the idle loop glitches that slice and makes the AoA jitter.
  // Only the button poll runs while idle.
  delay(2);
}
