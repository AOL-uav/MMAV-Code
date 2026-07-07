#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

/*
  703_BenchParamID_Standalone

  Standalone bench/open-loop identification sketch.
  This file is intentionally separate from the full 703_5Model flight code.

  FIX versus 626_BenchParamID_Standalone:
    The 626 bench sketch applied SERVO_LEFT_REVERSE at the PWM stage and used
    left = neutral + sym - diff. The flight code applies the reversal at the
    DEGREE stage (wingServoFromIncidence) and writes the PWM un-mirrored, with
    left incidence = sym + diff and right incidence = sym - diff. The two
    conventions disagreed:
      - bench left "neutral" sat at raw 85 deg instead of the flight neutral
        raw 95 deg (a +10 deg incidence offset), and
      - the bench "diff" stages excited the OPPOSITE differential sign,
        so B_roll identified from 626 bench data has an inverted sign
        relative to the flight uDiff convention.
    This sketch now uses exactly the flight-code mixing, so neutral PWM and
    stage deflections match what 703_5Model commands in flight, and the CSV
    logs commanded incidences directly comparable with flight logs.

  It does not run mission manager, ESEKF, or SMC. It commands repeatable servo
  schedules and logs IMU/servo data for bench-measurable parameters:

    beta_f, beta_o:
      Measure physical folded/open wing-sweep angle while morph_cmd_deg is held.

    T2_open, t2_margin:
      Use logged morph_cmd step time plus video/position observation.

    t2_set:
      Use gyro_mag_filt after deployment to estimate mechanical settling.

    kappa_1_0, kappa_3_0:
      Tail neutral/offset commands for trim experiments.

    B_roll, B_pitch:
      Wing differential/symmetric pulses provide commanded input. Real
      aerodynamic B still needs airflow rig or safe drop test.
*/

static const char LOG_TAG[] = "BID";

static const uint32_t SERIAL_BAUD = 115200;
static const int SD_CS_PIN = 10;
static const uint8_t SD_FLUSH_EVERY_N = 10;

static const int SERVO_LEFT_PIN = 4;
static const int SERVO_RIGHT_PIN = 3;
static const int SERVO_MORPH_PIN = 5;
static const int SERVO_TAIL_PIN = 6;

static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2500;
static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 180.0f;

// Same values and same meaning as 703_5Model: the reversal flips the
// incidence direction at the degree level; neutral is a RAW servo angle and
// the PWM conversion is never mirrored for the wings.
static const bool SERVO_LEFT_REVERSE = true;
static const bool SERVO_RIGHT_REVERSE = false;
static const bool SERVO_MORPH_REVERSE = false;
static const bool SERVO_TAIL_REVERSE = false;
static const float SERVO_LEFT_GAIN = 1.00f;
static const float SERVO_RIGHT_GAIN = 1.00f;

static const float SERVO_LEFT_NEUTRAL_DEG = 95.0f;
static const float SERVO_RIGHT_NEUTRAL_DEG = 95.0f;
static const float SERVO_TAIL_NEUTRAL_DEG = 90.0f;
static const float SERVO_MORPH_FOLDED_DEG = 90.0f;
static const float SERVO_MORPH_OPEN_DEG = 150.0f;

static const float WING_PULSE_DEG = 5.0f;
static const float TAIL_TEST_OFFSET_DEG = 10.0f;

static const float LOG_HZ = 80.0f;
static const uint32_t LOG_DT_US = (uint32_t)(1000000.0f / LOG_HZ);
static const float GYRO_MAG_ALPHA = 0.15f;

static Servo servoLeft;
static Servo servoRight;
static Servo servoMorph;
static Servo servoTail;
static File logFile;

static uint32_t nextLogUs = 0;
static uint32_t startMs = 0;
static uint32_t rowCount = 0;
static uint8_t deployCycle = 0;
static float gyroMagFilt = 0.0f;

static float leftCmdDeg = SERVO_LEFT_NEUTRAL_DEG;
static float rightCmdDeg = SERVO_RIGHT_NEUTRAL_DEG;
static float morphCmdDeg = SERVO_MORPH_FOLDED_DEG;
static float tailCmdDeg = SERVO_TAIL_NEUTRAL_DEG;
static float leftIncCmdDeg = 0.0f;
static float rightIncCmdDeg = 0.0f;

enum BenchStage : uint8_t {
  ST_FOLDED_HOLD = 0,
  ST_DEPLOY_STEP = 1,
  ST_OPEN_HOLD = 2,
  ST_REFOLD = 3,
  ST_TAIL_NEUTRAL = 4,
  ST_TAIL_POS = 5,
  ST_TAIL_NEG = 6,
  ST_WING_NEUTRAL = 7,
  ST_DIFF_POS = 8,
  ST_DIFF_NEG = 9,
  ST_SYM_POS = 10,
  ST_SYM_NEG = 11,
  ST_DONE = 12
};

static BenchStage stage = ST_FOLDED_HOLD;
static uint32_t stageStartMs = 0;

static float clampfLocal(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static int pwmFromServoDegrees(float degrees, bool reverse) {
  degrees = clampfLocal(degrees, SERVO_MIN_DEG, SERVO_MAX_DEG);
  if (reverse) degrees = SERVO_MIN_DEG + SERVO_MAX_DEG - degrees;
  const float fraction =
      (degrees - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
  return (int)lroundf(
      SERVO_MIN_US + fraction * (SERVO_MAX_US - SERVO_MIN_US));
}

// Identical mapping to 703_5Model wingServoFromIncidence: reversal at the
// degree level, neutral in raw servo degrees.
static float wingServoFromIncidence(
    float incidenceDeg,
    float neutralDeg,
    float gain,
    bool reverse) {
  const float sign = reverse ? -1.0f : 1.0f;
  return clampfLocal(
      neutralDeg + sign * gain * incidenceDeg,
      SERVO_MIN_DEG,
      SERVO_MAX_DEG);
}

static const char *stageName(BenchStage s) {
  switch (s) {
    case ST_FOLDED_HOLD: return "folded_hold_measure_beta_f";
    case ST_DEPLOY_STEP: return "deploy_step_measure_T2_open";
    case ST_OPEN_HOLD: return "open_hold_measure_beta_o_settle";
    case ST_REFOLD: return "refold_between_cycles";
    case ST_TAIL_NEUTRAL: return "tail_neutral_kappa";
    case ST_TAIL_POS: return "tail_positive_offset";
    case ST_TAIL_NEG: return "tail_negative_offset";
    case ST_WING_NEUTRAL: return "wing_neutral";
    case ST_DIFF_POS: return "diff_positive_B_roll";
    case ST_DIFF_NEG: return "diff_negative_B_roll";
    case ST_SYM_POS: return "sym_positive_B_pitch";
    case ST_SYM_NEG: return "sym_negative_B_pitch";
    case ST_DONE: return "done";
    default: return "unknown";
  }
}

static void makeLogFilename(char *filename, size_t filenameSize, int index) {
  snprintf(filename, filenameSize, "%s_%02d.CSV", LOG_TAG, index);
}

static void writeServos() {
  // Wings: reversal already applied at the degree level, so PWM conversion
  // uses reverse=false, exactly like the flight code.
  servoLeft.writeMicroseconds(pwmFromServoDegrees(leftCmdDeg, false));
  servoRight.writeMicroseconds(pwmFromServoDegrees(rightCmdDeg, false));
  servoMorph.writeMicroseconds(
      pwmFromServoDegrees(morphCmdDeg, SERVO_MORPH_REVERSE));
  servoTail.writeMicroseconds(
      pwmFromServoDegrees(tailCmdDeg, SERVO_TAIL_REVERSE));
}

static void setStage(BenchStage next) {
  stage = next;
  stageStartMs = millis();
  Serial.print(F("[Bench] stage="));
  Serial.println(stageName(stage));
}

// Flight convention: left incidence = sym + diff, right incidence = sym - diff.
// Positive diff here therefore matches positive uDiff in 703_5Model, so the
// sign of B_roll identified on this rig transfers directly to flight.
static void setWingIncidence(float symDeg, float diffDeg) {
  leftIncCmdDeg = symDeg + diffDeg;
  rightIncCmdDeg = symDeg - diffDeg;
  leftCmdDeg = wingServoFromIncidence(
      leftIncCmdDeg, SERVO_LEFT_NEUTRAL_DEG, SERVO_LEFT_GAIN,
      SERVO_LEFT_REVERSE);
  rightCmdDeg = wingServoFromIncidence(
      rightIncCmdDeg, SERVO_RIGHT_NEUTRAL_DEG, SERVO_RIGHT_GAIN,
      SERVO_RIGHT_REVERSE);
}

static void updateBenchSchedule() {
  const uint32_t elapsed = millis() - stageStartMs;

  switch (stage) {
    case ST_FOLDED_HOLD:
      morphCmdDeg = SERVO_MORPH_FOLDED_DEG;
      tailCmdDeg = SERVO_TAIL_NEUTRAL_DEG;
      setWingIncidence(0.0f, 0.0f);
      if (elapsed >= 1500) setStage(ST_DEPLOY_STEP);
      break;
    case ST_DEPLOY_STEP:
      morphCmdDeg = SERVO_MORPH_OPEN_DEG;
      if (elapsed >= 250) setStage(ST_OPEN_HOLD);
      break;
    case ST_OPEN_HOLD:
      morphCmdDeg = SERVO_MORPH_OPEN_DEG;
      if (elapsed >= 2500) {
        deployCycle++;
        if (deployCycle < 3) setStage(ST_REFOLD);
        else setStage(ST_TAIL_NEUTRAL);
      }
      break;
    case ST_REFOLD:
      morphCmdDeg = SERVO_MORPH_FOLDED_DEG;
      if (elapsed >= 2000) setStage(ST_FOLDED_HOLD);
      break;
    case ST_TAIL_NEUTRAL:
      morphCmdDeg = SERVO_MORPH_OPEN_DEG;
      tailCmdDeg = SERVO_TAIL_NEUTRAL_DEG;
      setWingIncidence(0.0f, 0.0f);
      if (elapsed >= 2000) setStage(ST_TAIL_POS);
      break;
    case ST_TAIL_POS:
      tailCmdDeg = SERVO_TAIL_NEUTRAL_DEG + TAIL_TEST_OFFSET_DEG;
      if (elapsed >= 2000) setStage(ST_TAIL_NEG);
      break;
    case ST_TAIL_NEG:
      tailCmdDeg = SERVO_TAIL_NEUTRAL_DEG - TAIL_TEST_OFFSET_DEG;
      if (elapsed >= 2000) setStage(ST_WING_NEUTRAL);
      break;
    case ST_WING_NEUTRAL:
      tailCmdDeg = SERVO_TAIL_NEUTRAL_DEG;
      setWingIncidence(0.0f, 0.0f);
      if (elapsed >= 2000) setStage(ST_DIFF_POS);
      break;
    case ST_DIFF_POS:
      setWingIncidence(0.0f, WING_PULSE_DEG);
      if (elapsed >= 1200) setStage(ST_DIFF_NEG);
      break;
    case ST_DIFF_NEG:
      setWingIncidence(0.0f, -WING_PULSE_DEG);
      if (elapsed >= 1200) setStage(ST_SYM_POS);
      break;
    case ST_SYM_POS:
      setWingIncidence(WING_PULSE_DEG, 0.0f);
      if (elapsed >= 1200) setStage(ST_SYM_NEG);
      break;
    case ST_SYM_NEG:
      setWingIncidence(-WING_PULSE_DEG, 0.0f);
      if (elapsed >= 1200) setStage(ST_DONE);
      break;
    case ST_DONE:
    default:
      morphCmdDeg = SERVO_MORPH_OPEN_DEG;
      tailCmdDeg = SERVO_TAIL_NEUTRAL_DEG;
      setWingIncidence(0.0f, 0.0f);
      break;
  }
}

static void writeCsvHeader(Print &out) {
  out.println(F(
      "ms,stage_id,stage_name,stage_elapsed_ms,cycle,"
      "ax,ay,az,gx,gy,gz,gyro_mag,gyro_mag_filt,"
      "left_cmd_deg,right_cmd_deg,morph_cmd_deg,tail_cmd_deg,"
      "left_inc_deg,right_inc_deg,"
      "left_pwm,right_pwm,morph_pwm,tail_pwm,notes"));
}

static void writeLogRow() {
  float ax = 0.0f, ay = 0.0f, az = 0.0f;
  float gx = 0.0f, gy = 0.0f, gz = 0.0f;
  if (IMU.accelerationAvailable()) IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable()) IMU.readGyroscope(gx, gy, gz);

  const float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
  gyroMagFilt += GYRO_MAG_ALPHA * (gyroMag - gyroMagFilt);
  if (!logFile) return;

  logFile.print(millis() - startMs); logFile.print(',');
  logFile.print((int)stage); logFile.print(',');
  logFile.print(stageName(stage)); logFile.print(',');
  logFile.print(millis() - stageStartMs); logFile.print(',');
  logFile.print(deployCycle); logFile.print(',');
  logFile.print(ax, 5); logFile.print(',');
  logFile.print(ay, 5); logFile.print(',');
  logFile.print(az, 5); logFile.print(',');
  logFile.print(gx, 5); logFile.print(',');
  logFile.print(gy, 5); logFile.print(',');
  logFile.print(gz, 5); logFile.print(',');
  logFile.print(gyroMag, 5); logFile.print(',');
  logFile.print(gyroMagFilt, 5); logFile.print(',');
  logFile.print(leftCmdDeg, 2); logFile.print(',');
  logFile.print(rightCmdDeg, 2); logFile.print(',');
  logFile.print(morphCmdDeg, 2); logFile.print(',');
  logFile.print(tailCmdDeg, 2); logFile.print(',');
  logFile.print(leftIncCmdDeg, 2); logFile.print(',');
  logFile.print(rightIncCmdDeg, 2); logFile.print(',');
  logFile.print(pwmFromServoDegrees(leftCmdDeg, false)); logFile.print(',');
  logFile.print(pwmFromServoDegrees(rightCmdDeg, false)); logFile.print(',');
  logFile.print(pwmFromServoDegrees(morphCmdDeg, SERVO_MORPH_REVERSE)); logFile.print(',');
  logFile.print(pwmFromServoDegrees(tailCmdDeg, SERVO_TAIL_REVERSE)); logFile.print(',');
  logFile.println(F("bench_identification"));

  rowCount++;
  if ((rowCount % SD_FLUSH_EVERY_N) == 0) logFile.flush();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  servoLeft.attach(SERVO_LEFT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoRight.attach(SERVO_RIGHT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoMorph.attach(SERVO_MORPH_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoTail.attach(SERVO_TAIL_PIN, SERVO_MIN_US, SERVO_MAX_US);
  setWingIncidence(0.0f, 0.0f);
  writeServos();

  if (!IMU.begin()) Serial.println(F("[Bench] ERROR: IMU not detected."));

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  SPI.begin();
  if (SD.begin(SD_CS_PIN)) {
    char filename[16];
    for (int i = 0; i < 100; i++) {
      makeLogFilename(filename, sizeof(filename), i);
      if (!SD.exists(filename)) {
        logFile = SD.open(filename, FILE_WRITE);
        break;
      }
    }
    if (logFile) {
      writeCsvHeader(logFile);
      logFile.flush();
      Serial.print(F("[Bench] Logging to "));
      Serial.println(logFile.name());
    } else {
      Serial.println(F("[Bench] ERROR: cannot create log file."));
    }
  } else {
    Serial.println(F("[Bench] ERROR: SD.begin failed."));
  }

  startMs = millis();
  stageStartMs = startMs;
  nextLogUs = micros();
  Serial.println(F("[Bench] Start video; CSV stage names mark tests."));
}

void loop() {
  updateBenchSchedule();
  writeServos();

  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextLogUs) < 0) return;
  while ((int32_t)(nowUs - nextLogUs) >= 0) nextLogUs += LOG_DT_US;
  writeLogRow();
}
