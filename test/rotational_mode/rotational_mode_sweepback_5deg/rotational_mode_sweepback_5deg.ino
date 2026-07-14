/*
  Rotational-mode 5-degree sweep-back bench test.

  Uses the servo connections/calibration from sweep_aero_logger:
    left AoA  -> D4
    right AoA -> D3
    sweep     -> D6

  On startup, the sketch:
    1. commands the sweep servo to the known unfolded position,
    2. moves it 5 servo degrees back toward folded, and
    3. holds both AoA servos at +5 degrees using the existing rotational-mode
       direction convention (5 degrees = 56 us below each flat command).

  There is no IMU, GPS, SD, control loop, or button handling in this bench
  sketch. Keep the mechanism restrained while verifying directions.
*/

#include <Arduino.h>
#include <Servo.h>

static const uint32_t SERIAL_BAUD = 115200;

// Connections from sweep_aero_logger.
static const int SERVO_LEFT_PIN = 4;
static const int SERVO_RIGHT_PIN = 3;
static const int SERVO_SWEEP_PIN = 6;
static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2500;

// Measured sweep endpoints from sweep_aero_logger.
static const int SWEEP_UNFOLDED_US = 2475;
static const int SWEEP_FOLDED_US = 500;

// Standard 500--2500 us servo range: 5 / 180 * 2000 = 55.6 us.
static const float SWEEP_BACK_DEG = 5.0f;
static const int SWEEP_BACK_US = 56;
static const int SWEEP_TARGET_US = SWEEP_UNFOLDED_US - SWEEP_BACK_US;

// Flat references from sweep_aero_logger. The current rotational convention
// decreases PWM for positive AoA rotation, so 5 degrees is 56 us below flat.
static const float AOA_TARGET_DEG = 5.0f;
static const int AOA_5_DEG_US = 56;
static const int AOA_LEFT_TARGET_US = 1575 - AOA_5_DEG_US;
static const int AOA_RIGHT_TARGET_US = 1500 - AOA_5_DEG_US;

static const uint16_t SWEEP_STEPS = 30;
static const uint16_t SWEEP_STEP_MS = 50;
static const uint32_t UNFOLDED_SETTLE_MS = 1000;

static Servo leftAoa;
static Servo rightAoa;
static Servo sweep;

static void writeAoaTargets() {
  leftAoa.writeMicroseconds(AOA_LEFT_TARGET_US);
  rightAoa.writeMicroseconds(AOA_RIGHT_TARGET_US);
}

static void slowSweepBackFromUnfolded() {
  for (uint16_t step = 1; step <= SWEEP_STEPS; ++step) {
    const float fraction = (float)step / (float)SWEEP_STEPS;
    const int commandUs = SWEEP_UNFOLDED_US + (int)lroundf(
        fraction * (SWEEP_TARGET_US - SWEEP_UNFOLDED_US));
    sweep.writeMicroseconds(commandUs);
    delay(SWEEP_STEP_MS);
  }
  sweep.writeMicroseconds(SWEEP_TARGET_US);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t serialStartMs = millis();
  while (!Serial && millis() - serialStartMs < 1500) delay(10);

  leftAoa.attach(SERVO_LEFT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  rightAoa.attach(SERVO_RIGHT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  sweep.attach(SERVO_SWEEP_PIN, SERVO_MIN_US, SERVO_MAX_US);

  writeAoaTargets();
  sweep.writeMicroseconds(SWEEP_UNFOLDED_US);
  delay(UNFOLDED_SETTLE_MS);
  slowSweepBackFromUnfolded();

  Serial.println(F("Rotational 5-degree sweep-back test holding."));
  Serial.print(F("Sweep: unfolded=")); Serial.print(SWEEP_UNFOLDED_US);
  Serial.print(F(" us, target=")); Serial.print(SWEEP_TARGET_US);
  Serial.println(F(" us"));
  Serial.print(F("AoA +5 deg: left=")); Serial.print(AOA_LEFT_TARGET_US);
  Serial.print(F(" us, right=")); Serial.print(AOA_RIGHT_TARGET_US);
  Serial.println(F(" us"));
}

void loop() {
  // Reassert the held commands without changing their position.
  writeAoaTargets();
  sweep.writeMicroseconds(SWEEP_TARGET_US);
  delay(250);
}
