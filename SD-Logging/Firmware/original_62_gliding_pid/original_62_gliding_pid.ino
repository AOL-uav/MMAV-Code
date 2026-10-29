#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <Servo.h>
#include <math.h>
#include <SPI.h>
#include "SdFat.h"

// SD Pins for Nano RP2040 Connect on Philhower core
#define SD_CS_PIN 5
#define SD_MOSI_PIN 7
#define SD_MISO_PIN 4
#define SD_SCK_PIN 6

SdFat sd;
FsFile logFile;
char logFileName[16];
bool sdAvailable = false;

/*
  Assumptions:
    - Arduino Nano RP2040 Connect built-in LSM6DSOX IMU.
    - Four servo signal pins: D2, D3, D4, D5.
    - Servos use external 5V power with common GND.
    - Gliding only: no GPS, no phase change.
*/

static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t SERIAL_WAIT_MS = 4000;

static const float CONTROL_HZ = 100.0f;
static const float DT_NOMINAL = 1.0f / CONTROL_HZ;
static const uint32_t CONTROL_DT_US = (uint32_t)(1000000.0f / CONTROL_HZ);

static const int SERVO_COUNT = 4;
static const int SERVO_PINS[SERVO_COUNT] = {2, 3, 4, 5};
static const bool SERVO_REVERSE[SERVO_COUNT] = {false, false, false, false};

static const int SERVO_MIN_US = 1000;
static const int SERVO_MAX_US = 2000;
static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 180.0f;
static const float WING_NEUTRAL_DEG = 90.0f;
static const float SERVO_RATE_LIMIT_DEG_PER_S = 120.0f;

static const float DEG_TO_RAD = 0.017453292519943295f;
static const float RAD_TO_DEG = 57.29577951308232f;

static const uint16_t CAL_SAMPLES = 500;
static const uint16_t CAL_DT_MS = 4;

static const float ACCEL_NOISE_G = 0.0010f;
static const float ACCEL_ROLL_NOISE_DEG = 0.0319f;
static const float ACCEL_PITCH_NOISE_DEG = 0.0513f;
static const float ACCEL_ANGLE_NOISE_DEG = 0.060f;
static const float GYRO_NOISE_DPS = 0.24f;
static const float GYRO_NOISE_X_DPS = 0.043f;
static const float GYRO_NOISE_Y_DPS = 0.239f;
static const float GYRO_NOISE_Z_DPS = 0.031f;
static const float ACC_GATE_LOW_G = 0.82f;
static const float ACC_GATE_HIGH_G = 1.18f;
static const float COMP_ALPHA = 0.985f;
static const float RATE_LPF_ALPHA = 0.22f;
static const float CMD_LPF_ALPHA = 0.35f;

static const float ROLL_TARGET_DEG = 0.0f;
static const float PITCH_TARGET_DEG = 0.0f;
static const float ANGLE_DEADBAND_DEG = 5.0f * ACCEL_ANGLE_NOISE_DEG;
static const float RATE_DEADBAND_DPS = 4.0f * GYRO_NOISE_DPS;

static const float KP_ROLL = 0.55f;
static const float KI_ROLL = 0.00f;
static const float KD_ROLL = 0.040f;
static const float KP_PITCH = 0.50f;
static const float KI_PITCH = 0.00f;
static const float KD_PITCH = 0.040f;

static const float INTEGRATOR_LIMIT_DEG = 5.0f;
static const float CMD_LIMIT_DEG = 14.0f;
static const uint32_t LOG_PERIOD_MS = 50;

struct ImuSample {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  bool valid;
};

static Servo servos[SERVO_COUNT];
static ImuSample imu = {};

static bool armed = false;
static bool estimatorReady = false;
static float gyroBiasDps[3] = {};
static float rollDeg = 0.0f;
static float pitchDeg = 0.0f;
static float pFiltDps = 0.0f;
static float qFiltDps = 0.0f;
static float rFiltDps = 0.0f;
static float rollI = 0.0f;
static float pitchI = 0.0f;
static float leftCmdFiltDeg = 0.0f;
static float rightCmdFiltDeg = 0.0f;
static float servoDeg[SERVO_COUNT] = {};
static uint32_t nextControlUs = 0;
static uint32_t lastControlUs = 0;
static uint32_t lastLogMs = 0;
static String inputLine;

static float sqf(float x) {
  return x * x;
}

static float clampfLocal(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float deadband(float x, float band) {
  if (fabsf(x) <= band) return 0.0f;
  return x > 0.0f ? x - band : x + band;
}

static float rateLimit(float target, float current, float maxStep) {
  return current + clampfLocal(target - current, -maxStep, maxStep);
}

static int pwmFromDeg(float deg) {
  const float d = clampfLocal(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  const float s = (d - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
  return (int)lroundf(SERVO_MIN_US + s * (SERVO_MAX_US - SERVO_MIN_US));
}

static float maybeReverseDeg(int idx, float deg) {
  if (!SERVO_REVERSE[idx]) return deg;
  return SERVO_MIN_DEG + SERVO_MAX_DEG - deg;
}

static bool readImu(ImuSample &s) {
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

static float accelNormG(const ImuSample &s) {
  return sqrtf(sqf(s.ax) + sqf(s.ay) + sqf(s.az));
}

static float accelRollDeg(const ImuSample &s) {
  return atan2f(s.ay, s.az) * RAD_TO_DEG;
}

static float accelPitchDeg(const ImuSample &s) {
  return atan2f(-s.ax, sqrtf(sqf(s.ay) + sqf(s.az))) * RAD_TO_DEG;
}

static bool accelUsable(const ImuSample &s) {
  const float a = accelNormG(s);
  return a >= ACC_GATE_LOW_G && a <= ACC_GATE_HIGH_G;
}

static void writeServoTargets(float leftDeg, float rightDeg, float dt) {
  const float target[SERVO_COUNT] = {
    leftDeg,
    rightDeg,
    leftDeg,
    rightDeg
  };
  const float step = SERVO_RATE_LIMIT_DEG_PER_S * dt;

  for (int i = 0; i < SERVO_COUNT; i++) {
    servoDeg[i] = rateLimit(target[i], servoDeg[i], step);
    servos[i].writeMicroseconds(pwmFromDeg(maybeReverseDeg(i, servoDeg[i])));
  }
}

static void writeNeutralServos(float dt) {
  writeServoTargets(WING_NEUTRAL_DEG, WING_NEUTRAL_DEG, dt);
}

static void calibrateGyroAndLevel() {
  Serial.println(F("# Keep aircraft still: gyro calibration"));

  float sumG[3] = {};
  float lastRollAccDeg = 0.0f;
  float lastPitchAccDeg = 0.0f;
  uint16_t good = 0;

  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    ImuSample s;
    if (readImu(s)) {
      sumG[0] += s.gx;
      sumG[1] += s.gy;
      sumG[2] += s.gz;
      if (accelUsable(s)) {
        lastRollAccDeg = accelRollDeg(s);
        lastPitchAccDeg = accelPitchDeg(s);
        good++;
      }
    }
    delay(CAL_DT_MS);
  }

  gyroBiasDps[0] = sumG[0] / (float)CAL_SAMPLES;
  gyroBiasDps[1] = sumG[1] / (float)CAL_SAMPLES;
  gyroBiasDps[2] = sumG[2] / (float)CAL_SAMPLES;

  rollDeg = lastRollAccDeg;
  pitchDeg = lastPitchAccDeg;
  estimatorReady = (good > CAL_SAMPLES / 2);

  Serial.print(F("# gyro_bias_dps="));
  Serial.print(gyroBiasDps[0], 5); Serial.print(',');
  Serial.print(gyroBiasDps[1], 5); Serial.print(',');
  Serial.println(gyroBiasDps[2], 5);
  Serial.print(F("# accel_level_good_samples="));
  Serial.println(good);
}

static void updateEstimator(float dt) {
  const float pDps = imu.gx - gyroBiasDps[0];
  const float qDps = imu.gy - gyroBiasDps[1];
  const float rDps = imu.gz - gyroBiasDps[2];

  pFiltDps += RATE_LPF_ALPHA * (pDps - pFiltDps);
  qFiltDps += RATE_LPF_ALPHA * (qDps - qFiltDps);
  rFiltDps += RATE_LPF_ALPHA * (rDps - rFiltDps);

  rollDeg += pFiltDps * dt;
  pitchDeg += qFiltDps * dt;

  if (accelUsable(imu)) {
    const float rollAcc = accelRollDeg(imu);
    const float pitchAcc = accelPitchDeg(imu);
    rollDeg = COMP_ALPHA * rollDeg + (1.0f - COMP_ALPHA) * rollAcc;
    pitchDeg = COMP_ALPHA * pitchDeg + (1.0f - COMP_ALPHA) * pitchAcc;
  }
}

static void updatePid(float dt) {
  float rollErrDeg = deadband(rollDeg - ROLL_TARGET_DEG, ANGLE_DEADBAND_DEG);
  float pitchErrDeg = deadband(pitchDeg - PITCH_TARGET_DEG, ANGLE_DEADBAND_DEG);
  float pRateDps = deadband(pFiltDps, RATE_DEADBAND_DPS);
  float qRateDps = deadband(qFiltDps, RATE_DEADBAND_DPS);

  if (armed) {
    rollI = clampfLocal(rollI + rollErrDeg * dt, -INTEGRATOR_LIMIT_DEG, INTEGRATOR_LIMIT_DEG);
    pitchI = clampfLocal(pitchI + pitchErrDeg * dt, -INTEGRATOR_LIMIT_DEG, INTEGRATOR_LIMIT_DEG);
  } else {
    rollI = 0.0f;
    pitchI = 0.0f;
  }

  float uRollDeg = -KP_ROLL * rollErrDeg - KI_ROLL * rollI - KD_ROLL * pRateDps;
  float uPitchDeg = -KP_PITCH * pitchErrDeg - KI_PITCH * pitchI - KD_PITCH * qRateDps;

  uRollDeg = clampfLocal(uRollDeg, -CMD_LIMIT_DEG, CMD_LIMIT_DEG);
  uPitchDeg = clampfLocal(uPitchDeg, -CMD_LIMIT_DEG, CMD_LIMIT_DEG);

  float leftCmdDeg = clampfLocal(uPitchDeg + uRollDeg, -CMD_LIMIT_DEG, CMD_LIMIT_DEG);
  float rightCmdDeg = clampfLocal(uPitchDeg - uRollDeg, -CMD_LIMIT_DEG, CMD_LIMIT_DEG);

  leftCmdFiltDeg += CMD_LPF_ALPHA * (leftCmdDeg - leftCmdFiltDeg);
  rightCmdFiltDeg += CMD_LPF_ALPHA * (rightCmdDeg - rightCmdFiltDeg);

  if (armed && estimatorReady) {
    writeServoTargets(WING_NEUTRAL_DEG + leftCmdFiltDeg, WING_NEUTRAL_DEG + rightCmdFiltDeg, dt);
  } else {
    leftCmdFiltDeg = 0.0f;
    rightCmdFiltDeg = 0.0f;
    writeNeutralServos(dt);
  }
}

static void printLog() {
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastLogMs) < LOG_PERIOD_MS) return;
  lastLogMs = nowMs;

  Serial.print(nowMs); Serial.print(',');
  Serial.print(armed ? 1 : 0); Serial.print(',');
  Serial.print(estimatorReady ? 1 : 0); Serial.print(',');
  Serial.print(accelUsable(imu) ? 1 : 0); Serial.print(',');
  Serial.print(rollDeg, 3); Serial.print(',');
  Serial.print(pitchDeg, 3); Serial.print(',');
  Serial.print(pFiltDps, 3); Serial.print(',');
  Serial.print(qFiltDps, 3); Serial.print(',');
  Serial.print(rFiltDps, 3); Serial.print(',');
  Serial.print(accelNormG(imu), 4); Serial.print(',');
  Serial.print(leftCmdFiltDeg, 3); Serial.print(',');
  Serial.print(rightCmdFiltDeg, 3); Serial.print(',');
  Serial.print(servoDeg[0], 2); Serial.print(',');
  Serial.print(servoDeg[1], 2); Serial.print(',');
  Serial.print(pwmFromDeg(maybeReverseDeg(0, servoDeg[0]))); Serial.print(',');
  Serial.println(pwmFromDeg(maybeReverseDeg(1, servoDeg[1])));

  if (sdAvailable) {
    logFile.print(nowMs); logFile.print(',');
    logFile.print(armed ? 1 : 0); logFile.print(',');
    logFile.print(estimatorReady ? 1 : 0); logFile.print(',');
    logFile.print(accelUsable(imu) ? 1 : 0); logFile.print(',');
    logFile.print(rollDeg, 3); logFile.print(',');
    logFile.print(pitchDeg, 3); logFile.print(',');
    logFile.print(pFiltDps, 3); logFile.print(',');
    logFile.print(qFiltDps, 3); logFile.print(',');
    logFile.print(rFiltDps, 3); logFile.print(',');
    logFile.print(accelNormG(imu), 4); logFile.print(',');
    logFile.print(leftCmdFiltDeg, 3); logFile.print(',');
    logFile.print(rightCmdFiltDeg, 3); logFile.print(',');
    logFile.print(servoDeg[0], 2); logFile.print(',');
    logFile.print(servoDeg[1], 2); logFile.print(',');
    logFile.print(pwmFromDeg(maybeReverseDeg(0, servoDeg[0]))); logFile.print(',');
    logFile.println(pwmFromDeg(maybeReverseDeg(1, servoDeg[1])));

    // Sync to SD card every 2 seconds to avoid data loss on power cut
    static uint32_t lastSyncMs = 0;
    if (nowMs - lastSyncMs > 2000) {
      logFile.sync();
      lastSyncMs = nowMs;
    }
  }
}

static void printHelp() {
  Serial.println(F("# Commands: a=arm, d=disarm, n=neutral, z=zero attitude, p=status, h=help"));
}

static void printStatus() {
  Serial.print(F("# armed="));
  Serial.print(armed ? 1 : 0);
  Serial.print(F(", roll_deg="));
  Serial.print(rollDeg, 2);
  Serial.print(F(", pitch_deg="));
  Serial.print(pitchDeg, 2);
  Serial.print(F(", gyro_noise_dps="));
  Serial.print(GYRO_NOISE_DPS, 2);
  Serial.print(F(", accel_angle_noise_deg="));
  Serial.print(ACCEL_ANGLE_NOISE_DEG, 2);
  Serial.print(F(", roll_pitch_noise_deg="));
  Serial.print(ACCEL_ROLL_NOISE_DEG, 3);
  Serial.print('/');
  Serial.print(ACCEL_PITCH_NOISE_DEG, 3);
  Serial.print(F(", gyro_xyz_noise_dps="));
  Serial.print(GYRO_NOISE_X_DPS, 3);
  Serial.print('/');
  Serial.print(GYRO_NOISE_Y_DPS, 3);
  Serial.print('/');
  Serial.print(GYRO_NOISE_Z_DPS, 3);
  Serial.print(F(", angle_deadband_deg="));
  Serial.print(ANGLE_DEADBAND_DEG, 2);
  Serial.print(F(", rate_deadband_dps="));
  Serial.println(RATE_DEADBAND_DPS, 2);
}

static void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  if (cmd == "a") {
    armed = true;
    Serial.println(F("# ARMED"));
  } else if (cmd == "d" || cmd == "n") {
    armed = false;
    Serial.println(F("# DISARMED"));
  } else if (cmd == "z") {
    rollDeg = 0.0f;
    pitchDeg = 0.0f;
    rollI = 0.0f;
    pitchI = 0.0f;
    Serial.println(F("# attitude zeroed"));
  } else if (cmd == "p") {
    printStatus();
  } else if (cmd == "h") {
    printHelp();
  } else {
    Serial.println(F("# unknown command"));
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

  Serial.println(F("# 524 gliding-only PID control"));
  Serial.println(F("# ms,armed,est_ok,acc_ok,roll_deg,pitch_deg,p_dps,q_dps,r_dps,a_norm_g,left_cmd_deg,right_cmd_deg,left_servo_deg,right_servo_deg,left_pwm,right_pwm"));

  // Initialize SD card
  SPI.setRX(SD_MISO_PIN);
  SPI.setTX(SD_MOSI_PIN);
  SPI.setSCK(SD_SCK_PIN);
  SPI.begin();

  if (!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(12)))) {
    Serial.println(F("# SD init failed. Logging disabled."));
  } else {
    // Find a unique filename
    for (int i = 0; i < 1000; i++) {
      sprintf(logFileName, "log%03d.csv", i);
      if (!sd.exists(logFileName)) break;
    }
    if (logFile.open(logFileName, O_WRONLY | O_CREAT | O_EXCL)) {
      sdAvailable = true;
      logFile.println(F("ms,armed,est_ok,acc_ok,roll_deg,pitch_deg,p_dps,q_dps,r_dps,a_norm_g,left_cmd_deg,right_cmd_deg,left_servo_deg,right_servo_deg,left_pwm,right_pwm"));
      Serial.print(F("# Logging to: ")); Serial.println(logFileName);
    } else {
      Serial.println(F("# Failed to open log file."));
    }
  }

  if (!IMU.begin()) {
    Serial.println(F("# ERROR: LSM6DSOX IMU not detected."));
    while (true) delay(1000);
  }

  for (int i = 0; i < SERVO_COUNT; i++) {
    servoDeg[i] = WING_NEUTRAL_DEG;
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
  }
  writeNeutralServos(DT_NOMINAL);

  calibrateGyroAndLevel();
  printHelp();

  lastControlUs = micros();
  nextControlUs = lastControlUs + CONTROL_DT_US;
}

void loop() {
  processSerial();

  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextControlUs) < 0) return;

  while ((int32_t)(nowUs - nextControlUs) >= 0) {
    nextControlUs += CONTROL_DT_US;
  }

  float dt = (nowUs - lastControlUs) * 1.0e-6f;
  lastControlUs = nowUs;
  if (dt <= 0.0f || dt > 0.1f) dt = DT_NOMINAL;

  if (!readImu(imu)) {
    writeNeutralServos(dt);
    return;
  }

  updateEstimator(dt);
  updatePid(dt);
  printLog();
}
