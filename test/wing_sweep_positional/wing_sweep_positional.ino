#include <Servo.h>

/*
  Wing fold/unfold test - POSITIONAL sweep servo version.

  Board: Arduino Nano RP2040 Connect
  FQBN:  arduino:mbed_nano:nanorp2040connect

  This is the positional-servo sibling of wing_sweep_test.ino. The sweep servo here
  is a normal POSITIONAL (angle) servo, so each state is an ABSOLUTE commanded pulse:
  travel is repeatable and load-independent (no open-loop drift like the continuous
  version). Recommended for flight.

  WHAT IT DOES
    Each button press toggles the wings between FOLDED and UNFOLDED.
    - Sweep servo (D5): commanded to SWEEP_UNFOLDED_US or SWEEP_FOLDED_US (absolute).
    - AoA servos (D4/D3): commanded to fixed FLAT or FOLDED pulses.
    Quirk: the AoA surfaces must be at the FOLDED position before the wing folds so
    they clear the fold path. Ordering keeps AoA folded during the sweep:
      FOLD   : AoA folded -> settle -> sweep to folded.
      UNFOLD : sweep to unfolded -> AoA flat.

  BUTTON / STATE
    Momentary button on A2 (board slot "10"), INPUT_PULLUP, pressed = LOW (short to
    GND). State lives in RAM, read live. 40 ms debounce + 3 s cooldown so a bouncy or
    double tap can't fire a second toggle mid-move. (RST-toggle via watchdog SCRATCH
    was tested 2026-06-25 and does NOT survive a reset on this mbed core - hence the
    GPIO button.)

  BOOT BEHAVIOR
    A positional servo CAN be safely commanded to a known absolute position, so on
    boot this drives the sweep to UNFOLDED and AoA to flat (a known home state). If
    the wing happens to be folded at power-up it will open - power on with it
    unfolded, or accept the one boot move.
*/

// ----------------------------- Pin map -----------------------------
static const int SWEEP_PIN     = 5;   // D5  - POSITIONAL sweep servo
static const int AOA_LEFT_PIN  = 4;   // D4  - left  wing AoA servo
static const int AOA_RIGHT_PIN = 3;   // D3  - right wing AoA servo
static const int BUTTON_PIN    = A2;  // board slot "10" - fold/unfold button (to GND)

// ----------------------------- Button -----------------------------
static const uint32_t DEBOUNCE_MS = 40;    // contact debounce
static const uint32_t COOLDOWN_MS = 3000;  // ignore presses for this long after a toggle

// ------------------------- Sweep servo (POSITIONAL) -------------------------
// Each state is an ABSOLUTE commanded pulse -> repeatable, load-independent. TUNE the
// two endpoints to your mechanism; they should be ~90deg apart (on a 500-2500us / 180deg
// servo, ~90deg ~= 1000us of pulse).
static const int SWEEP_UNFOLDED_US = 1000;  // wing OUT (flight)        - TUNE
static const int SWEEP_FOLDED_US   = 2000;  // wing FOLDED/stowed       - TUNE (~90deg from unfolded)
static const int SWEEP_MOVE_MS     = 700;   // time to let the servo finish its travel before continuing
// If it folds the WRONG way, swap SWEEP_UNFOLDED_US / SWEEP_FOLDED_US.

// ------------------------- AoA servos (POSITIONAL) -------------------------
// Fixed FLAT / FOLDED pulses (no degree math). Attach 500-2500us for full travel.
static const int AOA_MIN_US = 500;
static const int AOA_MAX_US = 2500;

static const int AOA_LEFT_FLAT_US  = 1575;  // measured flat / 0 incidence (left)
static const int AOA_RIGHT_FLAT_US = 1500;  // measured flat / 0 incidence (right)

// FOLDED: ~30deg incidence. Left servo reversed (lower pulse), right normal (higher pulse).
static const int AOA_LEFT_FOLDED_US  = 1250;
static const int AOA_RIGHT_FOLDED_US = 1800;

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

// Command the positional sweep servo to an absolute pulse and wait for it to arrive.
static void sweepTo(int targetUs) {
  sweepServo.writeMicroseconds(targetUs);
  delay(SWEEP_MOVE_MS);   // positional servo slews at its own rate; give it time
}

static void doFold() {
  if (Serial) Serial.println(F("# FOLD: AoA folded, then sweep to folded"));
  digitalWrite(LED_BUILTIN, HIGH);
  aoaFolded();
  delay(AOA_SETTLE_MS);
  sweepTo(SWEEP_FOLDED_US);
  digitalWrite(LED_BUILTIN, LOW);
}

static void doUnfold() {
  if (Serial) Serial.println(F("# UNFOLD: sweep to unfolded, then AoA flat"));
  digitalWrite(LED_BUILTIN, HIGH);
  sweepTo(SWEEP_UNFOLDED_US);        // AoA still folded during the sweep
  aoaFlat();                         // drop to flat only once swept out
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 1500) delay(10);

  aoaLeft.attach(AOA_LEFT_PIN, AOA_MIN_US, AOA_MAX_US);
  aoaRight.attach(AOA_RIGHT_PIN, AOA_MIN_US, AOA_MAX_US);

  // Re-attach kick: first attach after reset can fail silently on this Servo lib.
  sweepServo.attach(SWEEP_PIN, 500, 2500);
  delay(50);
  sweepServo.detach();
  delay(50);
  sweepServo.attach(SWEEP_PIN, 500, 2500);

  // Boot to known home: positional servo -> command UNFOLDED; AoA -> flat.
  folded = false;
  if (Serial) Serial.println(F("# Boot: home = UNFOLDED. Sweep -> unfolded, AoA -> flat."));
  sweepServo.writeMicroseconds(SWEEP_UNFOLDED_US);
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

  // Servos hold their last commanded pulse in hardware; do NOT re-write the sweep
  // here (D4/D5 share a PWM slice; re-writing the sweep would jitter the AoA).
  delay(2);
}
