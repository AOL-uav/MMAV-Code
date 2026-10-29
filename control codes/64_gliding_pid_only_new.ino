#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

/*
  Objective: Achieve glider stabilization w/ PID and comp. filter
  Authors: Jeff & Jeff
  Assumptions:
    - Nano RP2040 Connect built-in LSM6DSOX IMU.
    - Two servos only: left/right wing AoA.
    - Servo 1 signal D2, servo 2 signal D3.
    - External 5V servo power with common GND.
    - External SPI SD module, CS pin D10.
*/

// ==================================== Intialization ====================================

// Serial communication settings
static const uint32_t SERIAL_BAUD = 115200;  // Serial monitor speed.
static const uint32_t SERIAL_WAIT_MS = 1500;  // USB wait time at boot.

// Control loop and SD logging rates
static const float CONTROL_HZ = 100.0f;  // Servo control loop rate.
static const float SD_LOG_HZ = 20.0f;  // SD CSV write rate.
static const float DT_NOMINAL = 1.0f / CONTROL_HZ;  // Backup dt.
static const uint32_t CONTROL_DT_US = (uint32_t)(1000000.0f / CONTROL_HZ);  // Control period.
static const uint32_t SD_LOG_DT_US = (uint32_t)(1000000.0f / SD_LOG_HZ);  // SD log period.

// SD card recording values and set up
static const float RECORD_START_S = 0.0f;  // Log start after setup.
static const float RECORD_END_S = 3600.0f;  // Log stop after setup.
static const int SD_CS_PIN = 10;  // SD chip-select pin.
static const bool SD_FLUSH_EVERY_SAMPLE = false;  // Disabled for performance.
static const uint32_t SD_SYNC_INTERVAL_MS = 5000;  // Flush every 5s.

// Servo motor initialization
static const int SERVO_LEFT_PIN = 4;  // Left wing servo signal.
static const int SERVO_RIGHT_PIN = 3;  // Right wing servo signal.
static const bool SERVO_LEFT_REVERSE = true;  // Flip left servo direction.
static const bool SERVO_RIGHT_REVERSE = false;  // Flip right servo direction.
static const float SERVO_LEFT_GAIN = 1.00f;  // Scale left physical travel.
static const float SERVO_RIGHT_GAIN = 1.00f;  // Scale right physical travel.

// Servo motor speed and range limitations
static const int SERVO_MIN_US = 1000;  // Servo minimum PWM.
static const int SERVO_MAX_US = 2000;  // Servo maximum PWM.
static const float SERVO_MIN_DEG = 0.0f;  // Servo angle minimum.
static const float SERVO_MAX_DEG = 180.0f;  // Servo angle maximum.
//static const float AOA_NEUTRAL_DEG = 90.0f;  // Neutral AoA command.
static const float SERVO_LEFT_NEUTRAL_DEG  = 103.5f;  // measured (jog): raw servo deg for LEFT  wing 0 incidence
static const float SERVO_RIGHT_NEUTRAL_DEG =  90.0f;  // measured (jog): raw servo deg for RIGHT wing 0 incidence
static const float AOA_RATE_LIMIT_DEG_PER_S = 240.0f;  // Command slew limit.
static const float AOA_CMD_LIMIT_DEG = 60.0f;  // Max AoA correction.

// Unit converstion values
static const float RAD2DEG_LOCAL = 57.29577951308232f;  // rad to deg.
static const float DEG2RAD_LOCAL = 0.017453292519943295f;  // deg to rad.

// IMU calibration
static const uint16_t CAL_SAMPLES = 500;  // Startup gyro samples.
static const uint16_t CAL_DT_MS = 4;  // Calibration sample gap.

// IMU noise values
static const float ACCEL_NOISE_G = 0.0010f;  // Measured accel noise (a_norm_std=0.00094).
static const float ACCEL_ANGLE_NOISE_DEG = 0.055f;  // Measured: roll=0.032, pitch=0.051 deg.
static const float GYRO_NOISE_DPS_X = 0.043f;  // Measured gx std (roll axis).
static const float GYRO_NOISE_DPS_Y = 0.239f;  // Measured gy std (pitch axis).

// Complementary filter values
static const float ACC_GATE_LOW_G = 0.82f;  // Lower accel trust gate.
static const float ACC_GATE_HIGH_G = 1.18f;  // Upper accel trust gate.
static const float COMP_ALPHA = 0.980f;  // Gyro weight in filter.
static const float RATE_LPF_ALPHA = 0.22f;  // Gyro rate LPF gain.
static const float CMD_LPF_ALPHA = 0.55f;  // Servo command LPF gain.
static const float LARGE_ANGLE_START_DEG = 12.0f;  // Accel assist starts.
static const float LARGE_ANGLE_FULL_DEG = 45.0f;  // Accel assist full.
static const float LARGE_ANGLE_ACCEL_BLEND_MAX = 0.65f;  // Max accel assist.

// Control target values
static const float ROLL_TARGET_DEG = 0.0f;  // Desired roll.
static const float PITCH_TARGET_DEG = 0.0f;  // Desired pitch.

// Deadband values
static const float ANGLE_DEADBAND_DEG = 2.5f * ACCEL_ANGLE_NOISE_DEG;  // Smaller deadband for faster response.
static const float RATE_DEADBAND_P_DPS = 3.0f * GYRO_NOISE_DPS_X;  // Roll rate deadband.
static const float RATE_DEADBAND_Q_DPS = 3.0f * GYRO_NOISE_DPS_Y;  // Pitch rate deadband.

// PID controller values
static const float KP_ROLL = 1.10f;  // Roll P gain.
static const float KI_ROLL = 0.00f;  // Roll I gain.
static const float KD_ROLL = 0.060f;  // Roll D gain.
static const float KP_PITCH = 1.10f;  // Pitch P gain.
static const float KI_PITCH = 0.00f;  // Pitch I gain.
static const float KD_PITCH = 0.060f;  // Pitch D gain.
static const float INTEGRATOR_LIMIT_DEG = 5.0f;  // I-term clamp.

struct ImuSample {
  float ax;  // Accel x, g.
  float ay;  // Accel y, g.
  float az;  // Accel z, g.
  float gx;  // Gyro x, deg/s.
  float gy;  // Gyro y, deg/s.
  float gz;  // Gyro z, deg/s.
  bool valid;  // IMU read status.
};

struct ControlRecord {
  uint32_t ms;  // Log time.
  float ax;  // Accel x.
  float ay;  // Accel y.
  float az;  // Accel z.
  float aNorm;  // Accel magnitude.
  float gx;  // Gyro x.
  float gy;  // Gyro y.
  float gz;  // Gyro z.
  float rollAccDeg;  // Accel roll.
  float pitchAccDeg;  // Accel pitch.
  float rollCfDeg;  // Filtered roll.
  float pitchCfDeg;  // Filtered pitch.
  float pFiltDps;  // Filtered roll rate.
  float qFiltDps;  // Filtered pitch rate.
  float uRollDeg;  // Roll control output.
  float uPitchDeg;  // Pitch control output.
  float leftCmdDeg;  // Left AoA offset.
  float rightCmdDeg;  // Right AoA offset.
  float leftServoDeg;  // Left servo angle.
  float rightServoDeg;  // Right servo angle.
  uint16_t leftPwmUs;  // Left PWM.
  uint16_t rightPwmUs;  // Right PWM.
};

static Servo servoLeft;  // Left servo object.
static Servo servoRight;  // Right servo object.
static ImuSample imu = {};  // Latest IMU sample.
static File logFile;  // Current SD CSV file.

static bool estimatorReady = false;  // True after startup calibration.
static float gyroBiasDps[3] = {};  // Gyro zero offsets.
static float rollAccDeg = 0.0f;  // Accel-only roll.
static float pitchAccDeg = 0.0f;  // Accel-only pitch.
static float rollDeg = 0.0f;  // Filtered roll.
static float pitchDeg = 0.0f;  // Filtered pitch.
static float pFiltDps = 0.0f;  // Filtered roll rate.
static float qFiltDps = 0.0f;  // Filtered pitch rate.
static float rFiltDps = 0.0f;  // Filtered yaw rate.
static float rollI = 0.0f;  // Roll integral state.
static float pitchI = 0.0f;  // Pitch integral state.
static float uRollDeg = 0.0f;  // Roll control output.
static float uPitchDeg = 0.0f;  // Pitch control output.
static float leftCmdDeg = 0.0f;  // Raw left AoA offset.
static float rightCmdDeg = 0.0f;  // Raw right AoA offset.
static float leftCmdFiltDeg = 0.0f;  // Smoothed left command.
static float rightCmdFiltDeg = 0.0f;  // Smoothed right command.
//static float leftServoDeg = AOA_NEUTRAL_DEG;  // Final left servo angle.
//static float rightServoDeg = AOA_NEUTRAL_DEG;  // Final right servo angle.
static float leftServoDeg  = SERVO_LEFT_NEUTRAL_DEG;   
static float rightServoDeg = SERVO_RIGHT_NEUTRAL_DEG;
static uint16_t leftPwmUs = 1500;  // Final left PWM.
static uint16_t rightPwmUs = 1500;  // Final right PWM.
static uint32_t nextControlUs = 0;  // Next control tick.
static uint32_t lastControlUs = 0;  // Previous control tick.
static uint32_t lastStatsMs = 0;  // Last stats print.
static uint32_t loopCount = 0;  // Loops since last print.
static uint32_t maxLoopTimeUs = 0;  // Longest loop execution.
static uint32_t logClockStartMs = 0;  // SD log time origin.
static uint32_t nextSdLogUs = 0;  // Next SD write tick.
static uint32_t lastSdSyncMs = 0;  // Periodic sync timestamp.
static String inputLine;  // Serial command buffer.

static uint32_t recordCount = 0;  // Rows written to SD.
static bool sdReady = false;  // SD init status.
static bool recording = false;  // Inside log window.
static bool recordFinished = false;  // Log file closed.

// ==================================== Helper Functions ====================================

// Math functions ----------------------------------------------------------------
static float sqf(float x) {
  // Square helper.
  return x * x;
}

static float clampfLocal(float x, float lo, float hi) {
  // Limit value to range.
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float deadband(float x, float band) {
  // Remove small noise.
  if (fabsf(x) <= band) return 0.0f;
  return x > 0.0f ? x - band : x + band;
}

static float rateLimit(float target, float current, float maxStep) {
  // Limit command speed.
  return current + clampfLocal(target - current, -maxStep, maxStep);
}

// SD Logging  ----------------------------------------------------------------
static uint32_t recordStartMs() {
  // Log start time in ms.
  return (uint32_t)(RECORD_START_S * 1000.0f + 0.5f);
}

static uint32_t recordEndMs() {
  // Log end time in ms.
  return (uint32_t)(RECORD_END_S * 1000.0f + 0.5f);
}

// Servo Commands ----------------------------------------------------------------
static int pwmFromDeg(float deg) {
  // Convert servo angle to PWM.
  const float d = clampfLocal(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  const float s = (d - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
  return (int)lroundf(SERVO_MIN_US + s * (SERVO_MAX_US - SERVO_MIN_US));
}

//static float maybeReverseDeg(bool reverse, float deg) {
//  // Optional servo direction flip.
//  if (!reverse) return deg;
//  return SERVO_MIN_DEG + SERVO_MAX_DEG - deg;
//}
//
//static float scaleServoTravel(float deg, float gain) {
//  // Keep neutral fixed, scale travel.
//  return clampfLocal(AOA_NEUTRAL_DEG + (deg - AOA_NEUTRAL_DEG) * gain, SERVO_MIN_DEG, SERVO_MAX_DEG);
//}
static int   revSign(bool reverse) { return reverse ? -1 : +1; }
static float sideServoOut(float incidence, float neutral, float gain, bool reverse) {
  // servo = NEUTRAL[side] + REVSIGN[side] * GAIN[side] * incidence
  return clampfLocal(neutral + revSign(reverse) * gain * incidence, SERVO_MIN_DEG, SERVO_MAX_DEG);
}

// IMU Reading ----------------------------------------------------------------
static bool readImu(ImuSample &s) {
  // Read accel and gyro together.
  const uint32_t waitStartUs = micros();
  while (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
    if ((uint32_t)(micros() - waitStartUs) > 20000UL) {
      s.valid = false;
      return false;
    }
  }

  IMU.readAcceleration(s.ax, s.ay, s.az);
  IMU.readGyroscope(s.gx, s.gy, s.gz);
  s.valid = true;
  return true;
}

// Accelerometer Attitude Calculations ----------------------------------------------------------------
static float accelNormG(const ImuSample &s) {
  // Accel magnitude for gating.
  return sqrtf(sqf(s.ax) + sqf(s.ay) + sqf(s.az));
}

static float accelRollDeg(const ImuSample &s) {
  // Roll from gravity.
  return atan2f(s.ay, s.az) * RAD2DEG_LOCAL;
}

static float accelPitchDeg(const ImuSample &s) {
  // Pitch from gravity.
  return atan2f(-s.ax, sqrtf(sqf(s.ay) + sqf(s.az))) * RAD2DEG_LOCAL;
}

static bool accelUsable(const ImuSample &s) {
  const float a = accelNormG(s);
  // Trust accel near 1 g.
  return a >= ACC_GATE_LOW_G && a <= ACC_GATE_HIGH_G;
}

// Wing AoA Actuation ----------------------------------------------------------------
static void writeServos(float leftDeg, float rightDeg, float dt) {
  // Send final servo PWM.
//  const float step = AOA_RATE_LIMIT_DEG_PER_S * dt;
//  const float leftScaledDeg = scaleServoTravel(leftDeg, SERVO_LEFT_GAIN);
//  const float rightScaledDeg = scaleServoTravel(rightDeg, SERVO_RIGHT_GAIN);
//  leftServoDeg = rateLimit(leftScaledDeg, leftServoDeg, step);
//  rightServoDeg = rateLimit(rightScaledDeg, rightServoDeg, step);
//
//  leftPwmUs = (uint16_t)pwmFromDeg(maybeReverseDeg(SERVO_LEFT_REVERSE, leftServoDeg));
//  rightPwmUs = (uint16_t)pwmFromDeg(maybeReverseDeg(SERVO_RIGHT_REVERSE, rightServoDeg));

  const float incL = leftDeg  - AOA_NEUTRAL_DEG;   // callers pass AOA_NEUTRAL_DEG + incidence
  const float incR = rightDeg - AOA_NEUTRAL_DEG;
  const float step = AOA_RATE_LIMIT_DEG_PER_S * dt;
  const float l = sideServoOut(incL, SERVO_LEFT_NEUTRAL_DEG,  SERVO_LEFT_GAIN,  SERVO_LEFT_REVERSE);
  const float r = sideServoOut(incR, SERVO_RIGHT_NEUTRAL_DEG, SERVO_RIGHT_GAIN, SERVO_RIGHT_REVERSE);
  leftServoDeg  = rateLimit(l, leftServoDeg, step);
  rightServoDeg = rateLimit(r, rightServoDeg, step);
  leftPwmUs  = (uint16_t)pwmFromDeg(leftServoDeg);   // reverse includes in sideServoOut
  rightPwmUs = (uint16_t)pwmFromDeg(rightServoDeg);
  servoLeft.writeMicroseconds(leftPwmUs);
  servoRight.writeMicroseconds(rightPwmUs);
}

// ==================================== Major Functions ====================================

// Save the state (snapshot) of the aircraft at that current state
static ControlRecord makeRecord(uint32_t ms) {
  ControlRecord rec = {};
  rec.ms = ms;
  rec.ax = imu.ax;
  rec.ay = imu.ay;
  rec.az = imu.az;
  rec.aNorm = accelNormG(imu);
  rec.gx = imu.gx;
  rec.gy = imu.gy;
  rec.gz = imu.gz;
  rec.rollAccDeg = rollAccDeg;
  rec.pitchAccDeg = pitchAccDeg;
  rec.rollCfDeg = rollDeg;
  rec.pitchCfDeg = pitchDeg;
  rec.pFiltDps = pFiltDps;
  rec.qFiltDps = qFiltDps;
  rec.uRollDeg = uRollDeg;
  rec.uPitchDeg = uPitchDeg;
  rec.leftCmdDeg = leftCmdFiltDeg;
  rec.rightCmdDeg = rightCmdFiltDeg;
  rec.leftServoDeg = leftServoDeg;
  rec.rightServoDeg = rightServoDeg;
  rec.leftPwmUs = leftPwmUs;
  rec.rightPwmUs = rightPwmUs;
  return rec;
}

static void printLastSdFile() {
  // Find the last CSV written (highest index P62_xx.CSV that exists).
  if (!sdReady) {
    Serial.println(F("# No SD card: nothing to dump."));
    return;
  }

  char name[16];
  int lastIdx = -1;
  for (int i = 0; i < 100; i++) {
    snprintf(name, sizeof(name), "P62_%02d.CSV", i);
    if (SD.exists(name)) lastIdx = i;
  }

  if (lastIdx < 0) {
    Serial.println(F("# No previous log file found on SD."));
    return;
  }

  snprintf(name, sizeof(name), "P62_%02d.CSV", lastIdx);
  File f = SD.open(name, FILE_READ);
  if (!f) {
    Serial.print(F("# Cannot open "));
    Serial.println(name);
    return;
  }

  Serial.print(F("# Dumping "));
  Serial.println(name);

  // Stream file line by line to avoid large RAM buffer.
  while (f.available()) {
    Serial.write(f.read());
  }
  f.close();
  Serial.println(F("# Dump complete."));
}

static void writeCsvHeader(Print &out) {
  out.println(F("ms,ax_g,ay_g,az_g,a_norm_g,gx_dps,gy_dps,gz_dps,roll_acc_deg,pitch_acc_deg,roll_cf_deg,pitch_cf_deg,p_filt_dps,q_filt_dps,u_roll_deg,u_pitch_deg,left_cmd_deg,right_cmd_deg,left_servo_deg,right_servo_deg,left_pwm,right_pwm"));
}

static void writeRecord(Print &out, const ControlRecord &rec) {
  out.print(rec.ms); out.print(',');
  out.print(rec.ax, 6); out.print(',');
  out.print(rec.ay, 6); out.print(',');
  out.print(rec.az, 6); out.print(',');
  out.print(rec.aNorm, 6); out.print(',');
  out.print(rec.gx, 6); out.print(',');
  out.print(rec.gy, 6); out.print(',');
  out.print(rec.gz, 6); out.print(',');
  out.print(rec.rollAccDeg, 3); out.print(',');
  out.print(rec.pitchAccDeg, 3); out.print(',');
  out.print(rec.rollCfDeg, 3); out.print(',');
  out.print(rec.pitchCfDeg, 3); out.print(',');
  out.print(rec.pFiltDps, 3); out.print(',');
  out.print(rec.qFiltDps, 3); out.print(',');
  out.print(rec.uRollDeg, 3); out.print(',');
  out.print(rec.uPitchDeg, 3); out.print(',');
  out.print(rec.leftCmdDeg, 3); out.print(',');
  out.print(rec.rightCmdDeg, 3); out.print(',');
  out.print(rec.leftServoDeg, 2); out.print(',');
  out.print(rec.rightServoDeg, 2); out.print(',');
  out.print(rec.leftPwmUs); out.print(',');
  out.println(rec.rightPwmUs);
}

static void initSdLog() {
  // SD over SPI, CS=D10.
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  SPI.begin();
  delay(100);

  sdReady = SD.begin(SD_CS_PIN);
  if (!sdReady) {
    if (Serial) Serial.println(F("# ERROR: SD.begin failed."));
    return;
  }

  char name[16];
  for (int i = 0; i < 100; i++) {
    // Pick first unused CSV name.
    snprintf(name, sizeof(name), "P62_%02d.CSV", i);
    if (!SD.exists(name)) {
      logFile = SD.open(name, FILE_WRITE);
      break;
    }
  }

  if (!logFile) {
    sdReady = false;
    if (Serial) Serial.println(F("# ERROR: cannot create SD log file."));
    return;
  }

  writeCsvHeader(logFile);
  // Save header immediately.
  logFile.flush();

  if (Serial) {
    Serial.print(F("# SD logging file: "));
    Serial.println(logFile.name());
  }
}

static void stopSdLog(const __FlashStringHelper *reason) {
  if (recordFinished) return;
  recordFinished = true;
  recording = false;

  if (sdReady && logFile) {
    // Close file cleanly.
    logFile.print(F("# stop="));
    logFile.println(reason);
    logFile.print(F("# records="));
    logFile.println(recordCount);
    logFile.flush();
    logFile.close();
  }
}

static void calibrateGyroAndLevel() {
  if (Serial) Serial.println(F("# Keep board still: gyro calibration"));

  float sumG[3] = {};
  float lastRollAcc = 0.0f;
  float lastPitchAcc = 0.0f;
  uint16_t good = 0;

  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    ImuSample s;
    if (readImu(s)) {
      sumG[0] += s.gx;
      sumG[1] += s.gy;
      sumG[2] += s.gz;
      if (accelUsable(s)) {
        lastRollAcc = accelRollDeg(s);
        lastPitchAcc = accelPitchDeg(s);
        good++;
      }
    }
    delay(CAL_DT_MS);
  }

  gyroBiasDps[0] = sumG[0] / (float)CAL_SAMPLES;
  gyroBiasDps[1] = sumG[1] / (float)CAL_SAMPLES;
  gyroBiasDps[2] = sumG[2] / (float)CAL_SAMPLES;

  // Initialize attitude from accel.
  rollDeg = lastRollAcc;
  pitchDeg = lastPitchAcc;
  rollAccDeg = lastRollAcc;
  pitchAccDeg = lastPitchAcc;
  estimatorReady = (good > CAL_SAMPLES / 2);

  if (Serial) {
    Serial.print(F("# gyro_bias_dps="));
    Serial.print(gyroBiasDps[0], 5); Serial.print(',');
    Serial.print(gyroBiasDps[1], 5); Serial.print(',');
    Serial.println(gyroBiasDps[2], 5);
    Serial.print(F("# accel_good_samples="));
    Serial.println(good);
  }
}

static void updateComplementary(float dt) {
  // Bias-corrected gyro rates.
  const float pDps = imu.gx - gyroBiasDps[0];
  const float qDps = imu.gy - gyroBiasDps[1];
  const float rDps = imu.gz - gyroBiasDps[2];

  // Low-pass gyro noise.
  pFiltDps += RATE_LPF_ALPHA * (pDps - pFiltDps);
  qFiltDps += RATE_LPF_ALPHA * (qDps - qFiltDps);
  rFiltDps += RATE_LPF_ALPHA * (rDps - rFiltDps);

  // Euler angle integration.
  const float rollRad = rollDeg * DEG2RAD_LOCAL;
  const float pitchRad = pitchDeg * DEG2RAD_LOCAL;
  const float cosPitch = cosf(pitchRad);
  // Protect tan(pitch).
  const float safeCosPitch = fabsf(cosPitch) < 0.20f ? (cosPitch >= 0.0f ? 0.20f : -0.20f) : cosPitch;
  const float tanPitch = sinf(pitchRad) / safeCosPitch;

  rollDeg += (pFiltDps +
              qFiltDps * sinf(rollRad) * tanPitch +
              rFiltDps * cosf(rollRad) * tanPitch) * dt;
  pitchDeg += (qFiltDps * cosf(rollRad) -
               rFiltDps * sinf(rollRad)) * dt;

  rollAccDeg = accelRollDeg(imu);
  pitchAccDeg = accelPitchDeg(imu);
  if (accelUsable(imu)) {
    // Gyro fast, accel drift fix.
    rollDeg = COMP_ALPHA * rollDeg + (1.0f - COMP_ALPHA) * rollAccDeg;
    pitchDeg = COMP_ALPHA * pitchDeg + (1.0f - COMP_ALPHA) * pitchAccDeg;
  }
}

static void updatePidAndServos(float dt) {
  if (!estimatorReady) {
    leftCmdDeg = 0.0f;
    rightCmdDeg = 0.0f;
    leftCmdFiltDeg = 0.0f;
    rightCmdFiltDeg = 0.0f;
    writeServos(AOA_NEUTRAL_DEG, AOA_NEUTRAL_DEG, dt);
    return;
  }

  float controlRollDeg = rollDeg;
  float controlPitchDeg = pitchDeg;
  if (accelUsable(imu)) {
    // Large-angle accel assist.
    const float largeAngleDeg = fmaxf(fabsf(rollAccDeg), fabsf(pitchAccDeg));
    const float largeBlend01 = clampfLocal(
      (largeAngleDeg - LARGE_ANGLE_START_DEG) / (LARGE_ANGLE_FULL_DEG - LARGE_ANGLE_START_DEG),
      0.0f,
      1.0f
    );
    const float accelBlend = LARGE_ANGLE_ACCEL_BLEND_MAX * largeBlend01;
    controlRollDeg = (1.0f - accelBlend) * rollDeg + accelBlend * rollAccDeg;
    controlPitchDeg = (1.0f - accelBlend) * pitchDeg + accelBlend * pitchAccDeg;
  }

  const float rollErr = deadband(controlRollDeg - ROLL_TARGET_DEG, ANGLE_DEADBAND_DEG);
  const float pitchErr = deadband(controlPitchDeg - PITCH_TARGET_DEG, ANGLE_DEADBAND_DEG);
  const float pRate = deadband(pFiltDps, RATE_DEADBAND_P_DPS);
  const float qRate = deadband(qFiltDps, RATE_DEADBAND_Q_DPS);

  // KI=0, so this is PD.
  rollI = clampfLocal(rollI + rollErr * dt, -INTEGRATOR_LIMIT_DEG, INTEGRATOR_LIMIT_DEG);
  pitchI = clampfLocal(pitchI + pitchErr * dt, -INTEGRATOR_LIMIT_DEG, INTEGRATOR_LIMIT_DEG);

  // Attitude error + rate damping.
  uRollDeg = KP_ROLL * rollErr + KI_ROLL * rollI + KD_ROLL * pRate;
  uPitchDeg = -KP_PITCH * pitchErr - KI_PITCH * pitchI - KD_PITCH * qRate;
  uRollDeg = clampfLocal(uRollDeg, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);
  uPitchDeg = clampfLocal(uPitchDeg, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);

  // Pitch same, roll opposite.
  leftCmdDeg = clampfLocal(uPitchDeg + uRollDeg, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);
  rightCmdDeg = clampfLocal(uPitchDeg - uRollDeg, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);

  // Smooth servo command.
  leftCmdFiltDeg += CMD_LPF_ALPHA * (leftCmdDeg - leftCmdFiltDeg);
  rightCmdFiltDeg += CMD_LPF_ALPHA * (rightCmdDeg - rightCmdFiltDeg);

  writeServos(AOA_NEUTRAL_DEG + leftCmdFiltDeg, AOA_NEUTRAL_DEG + rightCmdFiltDeg, dt);
}

static void maybeWriteSdLog() {
  if (!sdReady || !logFile || recordFinished) return;

  const uint32_t elapsedMs = millis() - logClockStartMs;
  // Auto log time window.
  if (elapsedMs < recordStartMs()) return;

  if (!recording) {
    recording = true;
    nextSdLogUs = micros();
    lastSdSyncMs = millis();
  }

  if (elapsedMs >= recordEndMs()) {
    stopSdLog(F("time_end"));
    return;
  }

  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextSdLogUs) < 0) return;

  while ((int32_t)(nowUs - nextSdLogUs) >= 0) {
    nextSdLogUs += SD_LOG_DT_US;
  }

  const uint32_t logMs = elapsedMs - recordStartMs();
  writeRecord(logFile, makeRecord(logMs));
  recordCount++;

  // Flush periodically instead of every sample.
  if (SD_FLUSH_EVERY_SAMPLE || (millis() - lastSdSyncMs >= SD_SYNC_INTERVAL_MS)) {
    logFile.flush();
    lastSdSyncMs = millis();
  }

  if (logFile.getWriteError()) {
    stopSdLog(F("sd_write_error"));
  }
}

static void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd == "z") {
    rollDeg = 0.0f;
    pitchDeg = 0.0f;
    rollI = 0.0f;
    pitchI = 0.0f;
    if (Serial) Serial.println(F("# attitude zeroed"));
  } else if (cmd == "d") {
    printLastSdFile();
  } else if (cmd == "h") {
    if (Serial) Serial.println(F("# Commands: z=zero attitude, d=dump last SD file, h=help"));
  } else {
    if (Serial) Serial.println(F("# unknown command"));
  }
}

static void processSerial() {
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      handleCommand(inputLine);
      inputLine = "";
    } else {
      inputLine += ch;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t serialStartMs = millis();
  while (!Serial && (uint32_t)(millis() - serialStartMs) < SERIAL_WAIT_MS) {
    delay(10);
  }

  if (Serial) {
    Serial.println(F("# 62 IMU complementary PID servo + SD log"));
    Serial.print(F("# noise accel_g="));
    Serial.print(ACCEL_NOISE_G, 4);
    Serial.print(F(", accel_angle_deg="));
    Serial.print(ACCEL_ANGLE_NOISE_DEG, 3);
    Serial.print(F(", gyro_dps_x="));
    Serial.print(GYRO_NOISE_DPS_X, 3);
    Serial.print(F(", gyro_dps_y="));
    Serial.println(GYRO_NOISE_DPS_Y, 3);
  }

  if (!IMU.begin()) {
    if (Serial) Serial.println(F("# ERROR: LSM6DSOX IMU not detected."));
    while (true) delay(1000);
  }

  servoLeft.attach(SERVO_LEFT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoRight.attach(SERVO_RIGHT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  writeServos(AOA_NEUTRAL_DEG, AOA_NEUTRAL_DEG, DT_NOMINAL);

  initSdLog();

  if (Serial) printLastSdFile();

  calibrateGyroAndLevel();

  logClockStartMs = millis();
  lastStatsMs = logClockStartMs;
  lastControlUs = micros();
  nextControlUs = lastControlUs + CONTROL_DT_US;
  nextSdLogUs = lastControlUs;
}

void loop() {
  const uint32_t loopStartUs = micros();
  processSerial();

  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextControlUs) < 0) return;

  while ((int32_t)(nowUs - nextControlUs) >= 0) {
    nextControlUs += CONTROL_DT_US;
  }

  float dt = (nowUs - lastControlUs) * 1.0e-6f;
  lastControlUs = nowUs;
  if (dt <= 0.0f || dt > 0.1f) dt = DT_NOMINAL;

  if (readImu(imu)) {
    updateComplementary(dt);
    updatePidAndServos(dt);
    maybeWriteSdLog();
  }

  // Performance monitoring
  loopCount++;
  uint32_t loopTimeUs = micros() - loopStartUs;
  if (loopTimeUs > maxLoopTimeUs) maxLoopTimeUs = loopTimeUs;

  if (millis() - lastStatsMs >= 1000) {
    if (Serial) {
      Serial.print(F("# Hz: "));
      Serial.print(loopCount);
      Serial.print(F(" | Max Loop Us: "));
      Serial.println(maxLoopTimeUs);
    }
    loopCount = 0;
    maxLoopTimeUs = 0;
    lastStatsMs = millis();
  }
}