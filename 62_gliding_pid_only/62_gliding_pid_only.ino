#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

/*
  Objective: 

  Assumptions:
    - Nano RP2040 Connect built-in LSM6DSOX IMU.
    - Two servos only: left/right wing AoA.
    - Servo 1 signal D2, servo 2 signal D3.
    - External 5V servo power with common GND.
    - External SPI SD module, CS pin D10.
*/

static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t SERIAL_WAIT_MS = 1500;

static const float CONTROL_HZ = 100.0f;
static const float LIVE_PRINT_HZ = 20.0f;
static const float SD_LOG_HZ = 20.0f;
static const float DT_NOMINAL = 1.0f / CONTROL_HZ;
static const uint32_t CONTROL_DT_US = (uint32_t)(1000000.0f / CONTROL_HZ);
static const uint32_t LIVE_PRINT_DT_MS = (uint32_t)(1000.0f / LIVE_PRINT_HZ);
static const uint32_t SD_LOG_DT_US = (uint32_t)(1000000.0f / SD_LOG_HZ);

static const float RECORD_START_S = 0.0f;
static const float RECORD_END_S = 3600.0f;
static const int SD_CS_PIN = 10;
static const bool SD_FLUSH_EVERY_SAMPLE = true;

static const int SERVO_LEFT_PIN = 2;
static const int SERVO_RIGHT_PIN = 3;
static const bool SERVO_LEFT_REVERSE = false;
static const bool SERVO_RIGHT_REVERSE = false;

static const int SERVO_MIN_US = 1000;
static const int SERVO_MAX_US = 2000;
static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 180.0f;
static const float AOA_NEUTRAL_DEG = 90.0f;
static const float AOA_RATE_LIMIT_DEG_PER_S = 180.0f;
static const float AOA_CMD_LIMIT_DEG = 18.0f;

static const float RAD2DEG_LOCAL = 57.29577951308232f;

static const uint16_t CAL_SAMPLES = 500;
static const uint16_t CAL_DT_MS = 4;

static const float ACCEL_NOISE_G = 0.0010f;
static const float ACCEL_ANGLE_NOISE_DEG = 0.060f;
static const float GYRO_NOISE_DPS = 0.24f;

static const float ACC_GATE_LOW_G = 0.82f;
static const float ACC_GATE_HIGH_G = 1.18f;
static const float COMP_ALPHA = 0.985f;
static const float RATE_LPF_ALPHA = 0.22f;
static const float CMD_LPF_ALPHA = 0.35f;

static const float ROLL_TARGET_DEG = 0.0f;
static const float PITCH_TARGET_DEG = 0.0f;
static const float ANGLE_DEADBAND_DEG = 5.0f * ACCEL_ANGLE_NOISE_DEG;
static const float RATE_DEADBAND_DPS = 4.0f * GYRO_NOISE_DPS;

static const float KP_ROLL = 0.65f;
static const float KI_ROLL = 0.00f;
static const float KD_ROLL = 0.045f;
static const float KP_PITCH = 0.55f;
static const float KI_PITCH = 0.00f;
static const float KD_PITCH = 0.045f;
static const float INTEGRATOR_LIMIT_DEG = 5.0f;

struct ImuSample {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  bool valid;
};

struct ControlRecord {
  uint32_t ms;
  float ax;
  float ay;
  float az;
  float aNorm;
  float gx;
  float gy;
  float gz;
  float rollAccDeg;
  float pitchAccDeg;
  float rollCfDeg;
  float pitchCfDeg;
  float pFiltDps;
  float qFiltDps;
  float uRollDeg;
  float uPitchDeg;
  float leftCmdDeg;
  float rightCmdDeg;
  float leftServoDeg;
  float rightServoDeg;
  uint16_t leftPwmUs;
  uint16_t rightPwmUs;
};

static Servo servoLeft;
static Servo servoRight;
static ImuSample imu = {};
static File logFile;

static bool estimatorReady = false;
static float gyroBiasDps[3] = {};
static float rollAccDeg = 0.0f;
static float pitchAccDeg = 0.0f;
static float rollDeg = 0.0f;
static float pitchDeg = 0.0f;
static float pFiltDps = 0.0f;
static float qFiltDps = 0.0f;
static float rFiltDps = 0.0f;
static float rollI = 0.0f;
static float pitchI = 0.0f;
static float uRollDeg = 0.0f;
static float uPitchDeg = 0.0f;
static float leftCmdDeg = 0.0f;
static float rightCmdDeg = 0.0f;
static float leftCmdFiltDeg = 0.0f;
static float rightCmdFiltDeg = 0.0f;
static float leftServoDeg = AOA_NEUTRAL_DEG;
static float rightServoDeg = AOA_NEUTRAL_DEG;
static uint16_t leftPwmUs = 1500;
static uint16_t rightPwmUs = 1500;
static uint32_t nextControlUs = 0;
static uint32_t lastControlUs = 0;
static uint32_t lastLivePrintMs = 0;
static uint32_t logClockStartMs = 0;
static uint32_t nextSdLogUs = 0;
static String inputLine;

static uint32_t recordCount = 0;
static bool sdReady = false;
static bool recording = false;
static bool recordFinished = false;

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

static uint32_t recordStartMs() {
  return (uint32_t)(RECORD_START_S * 1000.0f + 0.5f);
}

static uint32_t recordEndMs() {
  return (uint32_t)(RECORD_END_S * 1000.0f + 0.5f);
}

static int pwmFromDeg(float deg) {
  const float d = clampfLocal(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  const float s = (d - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
  return (int)lroundf(SERVO_MIN_US + s * (SERVO_MAX_US - SERVO_MIN_US));
}

static float maybeReverseDeg(bool reverse, float deg) {
  if (!reverse) return deg;
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
  return atan2f(s.ay, s.az) * RAD2DEG_LOCAL;
}

static float accelPitchDeg(const ImuSample &s) {
  return atan2f(-s.ax, sqrtf(sqf(s.ay) + sqf(s.az))) * RAD2DEG_LOCAL;
}

static bool accelUsable(const ImuSample &s) {
  const float a = accelNormG(s);
  return a >= ACC_GATE_LOW_G && a <= ACC_GATE_HIGH_G;
}

static void writeServos(float leftDeg, float rightDeg, float dt) {
  const float step = AOA_RATE_LIMIT_DEG_PER_S * dt;
  leftServoDeg = rateLimit(leftDeg, leftServoDeg, step);
  rightServoDeg = rateLimit(rightDeg, rightServoDeg, step);

  leftPwmUs = (uint16_t)pwmFromDeg(maybeReverseDeg(SERVO_LEFT_REVERSE, leftServoDeg));
  rightPwmUs = (uint16_t)pwmFromDeg(maybeReverseDeg(SERVO_RIGHT_REVERSE, rightServoDeg));
  servoLeft.writeMicroseconds(leftPwmUs);
  servoRight.writeMicroseconds(rightPwmUs);
}

static void printCsvHeader() {
  Serial.println(F("ms,ax_g,ay_g,az_g,a_norm_g,gx_dps,gy_dps,gz_dps,roll_acc_deg,pitch_acc_deg,roll_cf_deg,pitch_cf_deg,p_filt_dps,q_filt_dps,u_roll_deg,u_pitch_deg,left_cmd_deg,right_cmd_deg,left_servo_deg,right_servo_deg,left_pwm,right_pwm"));
}

static void printRecord(const ControlRecord &rec) {
  Serial.print(rec.ms); Serial.print(',');
  Serial.print(rec.ax, 6); Serial.print(',');
  Serial.print(rec.ay, 6); Serial.print(',');
  Serial.print(rec.az, 6); Serial.print(',');
  Serial.print(rec.aNorm, 6); Serial.print(',');
  Serial.print(rec.gx, 6); Serial.print(',');
  Serial.print(rec.gy, 6); Serial.print(',');
  Serial.print(rec.gz, 6); Serial.print(',');
  Serial.print(rec.rollAccDeg, 3); Serial.print(',');
  Serial.print(rec.pitchAccDeg, 3); Serial.print(',');
  Serial.print(rec.rollCfDeg, 3); Serial.print(',');
  Serial.print(rec.pitchCfDeg, 3); Serial.print(',');
  Serial.print(rec.pFiltDps, 3); Serial.print(',');
  Serial.print(rec.qFiltDps, 3); Serial.print(',');
  Serial.print(rec.uRollDeg, 3); Serial.print(',');
  Serial.print(rec.uPitchDeg, 3); Serial.print(',');
  Serial.print(rec.leftCmdDeg, 3); Serial.print(',');
  Serial.print(rec.rightCmdDeg, 3); Serial.print(',');
  Serial.print(rec.leftServoDeg, 2); Serial.print(',');
  Serial.print(rec.rightServoDeg, 2); Serial.print(',');
  Serial.print(rec.leftPwmUs); Serial.print(',');
  Serial.println(rec.rightPwmUs);
}

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
  const float pDps = imu.gx - gyroBiasDps[0];
  const float qDps = imu.gy - gyroBiasDps[1];
  const float rDps = imu.gz - gyroBiasDps[2];

  pFiltDps += RATE_LPF_ALPHA * (pDps - pFiltDps);
  qFiltDps += RATE_LPF_ALPHA * (qDps - qFiltDps);
  rFiltDps += RATE_LPF_ALPHA * (rDps - rFiltDps);

  rollDeg += pFiltDps * dt;
  pitchDeg += qFiltDps * dt;

  rollAccDeg = accelRollDeg(imu);
  pitchAccDeg = accelPitchDeg(imu);
  if (accelUsable(imu)) {
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

  const float rollErr = deadband(rollDeg - ROLL_TARGET_DEG, ANGLE_DEADBAND_DEG);
  const float pitchErr = deadband(pitchDeg - PITCH_TARGET_DEG, ANGLE_DEADBAND_DEG);
  const float pRate = deadband(pFiltDps, RATE_DEADBAND_DPS);
  const float qRate = deadband(qFiltDps, RATE_DEADBAND_DPS);

  rollI = clampfLocal(rollI + rollErr * dt, -INTEGRATOR_LIMIT_DEG, INTEGRATOR_LIMIT_DEG);
  pitchI = clampfLocal(pitchI + pitchErr * dt, -INTEGRATOR_LIMIT_DEG, INTEGRATOR_LIMIT_DEG);

  uRollDeg = -KP_ROLL * rollErr - KI_ROLL * rollI - KD_ROLL * pRate;
  uPitchDeg = -KP_PITCH * pitchErr - KI_PITCH * pitchI - KD_PITCH * qRate;
  uRollDeg = clampfLocal(uRollDeg, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);
  uPitchDeg = clampfLocal(uPitchDeg, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);

  leftCmdDeg = clampfLocal(uPitchDeg + uRollDeg, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);
  rightCmdDeg = clampfLocal(uPitchDeg - uRollDeg, -AOA_CMD_LIMIT_DEG, AOA_CMD_LIMIT_DEG);

  leftCmdFiltDeg += CMD_LPF_ALPHA * (leftCmdDeg - leftCmdFiltDeg);
  rightCmdFiltDeg += CMD_LPF_ALPHA * (rightCmdDeg - rightCmdFiltDeg);

  writeServos(AOA_NEUTRAL_DEG + leftCmdFiltDeg, AOA_NEUTRAL_DEG + rightCmdFiltDeg, dt);
}

static void maybePrintLiveLog() {
  if (!Serial) return;

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastLivePrintMs) < LIVE_PRINT_DT_MS) return;
  lastLivePrintMs = nowMs;

  const uint32_t elapsedMs = nowMs - logClockStartMs;
  printRecord(makeRecord(elapsedMs));
}

static void maybeWriteSdLog() {
  if (!sdReady || !logFile || recordFinished) return;

  const uint32_t elapsedMs = millis() - logClockStartMs;
  if (elapsedMs < recordStartMs()) return;

  if (!recording) {
    recording = true;
    nextSdLogUs = micros();
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

  if (SD_FLUSH_EVERY_SAMPLE) logFile.flush();
  if (logFile.getWriteError()) {
    stopSdLog(F("sd_write_error"));
  }
}

static void printStatus() {
  if (!Serial) return;

  Serial.print(F("# roll_cf="));
  Serial.print(rollDeg, 2);
  Serial.print(F(", pitch_cf="));
  Serial.print(pitchDeg, 2);
  Serial.print(F(", left_cmd="));
  Serial.print(leftCmdFiltDeg, 2);
  Serial.print(F(", right_cmd="));
  Serial.print(rightCmdFiltDeg, 2);
  Serial.print(F(", servo="));
  Serial.print(leftServoDeg, 1);
  Serial.print('/');
  Serial.print(rightServoDeg, 1);
  Serial.print(F(", sd_records="));
  Serial.println(recordCount);
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
  } else if (cmd == "p") {
    printStatus();
  } else if (cmd == "h") {
    if (Serial) Serial.println(F("# Commands: z=zero attitude, p=status, h=help"));
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
    Serial.print(F(", gyro_dps="));
    Serial.println(GYRO_NOISE_DPS, 3);
  }

  if (!IMU.begin()) {
    if (Serial) Serial.println(F("# ERROR: LSM6DSOX IMU not detected."));
    while (true) delay(1000);
  }

  servoLeft.attach(SERVO_LEFT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoRight.attach(SERVO_RIGHT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  writeServos(AOA_NEUTRAL_DEG, AOA_NEUTRAL_DEG, DT_NOMINAL);

  initSdLog();
  calibrateGyroAndLevel();

  if (Serial) printCsvHeader();

  logClockStartMs = millis();
  lastLivePrintMs = millis();
  lastControlUs = micros();
  nextControlUs = lastControlUs + CONTROL_DT_US;
  nextSdLogUs = lastControlUs;
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

  if (!readImu(imu)) return;

  updateComplementary(dt);
  updatePidAndServos(dt);
  maybeWriteSdLog();
  maybePrintLiveLog();
}
