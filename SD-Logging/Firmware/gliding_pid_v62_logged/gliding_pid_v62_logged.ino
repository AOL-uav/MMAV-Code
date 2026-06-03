#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <Servo.h>
#include <math.h>
#include <SPI.h>
#include "SdFat.h"

/*
  Samara V2 - Gliding PID v62 (Robust 500Hz Black-Box Logging)
  Changes for reliability:
    - Explicit IMU wait for 500Hz sync
    - Verbose Serial debugging for SD/IMU init
    - Larger buffer and consolidated SD writes
    - Explicit ODR configuration after IMU.begin()
*/

// --- HARDWARE CONFIG ---
#define SD_CS_PIN 5
#define SD_MOSI_PIN 7
#define SD_MISO_PIN 4
#define SD_SCK_PIN 6

// --- TIMING CONFIG ---
static const float CONTROL_HZ = 500.0f; 
static const uint32_t CONTROL_DT_US = 2000; // 2ms

// --- BUFFER CONFIG ---
struct LogEntry {
  uint32_t ms;
  uint16_t dt_us; // dt can fit in 16-bit for 2ms
  bool armed;
  float ax, ay, az; // raw
  float gx, gy, gz; // raw
  float roll, pitch; // est
  float p, q, r;     // filt
  float roll_i, pitch_i; // integrators
  float l_cmd, r_cmd;
  float l_servo, r_servo;
};

#define BUFFER_SIZE 500 // 1 second of data
LogEntry buffer[BUFFER_SIZE];
volatile int bufferIdx = 0;

SdFat sd;
FsFile logFile;
char logFileName[16];
bool sdAvailable = false;

// --- FLIGHT CONTROLLER STATES ---
static const int SERVO_COUNT = 4;
static const int SERVO_PINS[SERVO_COUNT] = {2, 3, 4, 5};
static Servo servos[SERVO_COUNT];
static float servoDeg[SERVO_COUNT] = {90, 90, 90, 90};

static bool armed = false;
static bool estimatorReady = false;
static float gyroBiasDps[3] = {0,0,0};
static float rollDeg = 0.0f, pitchDeg = 0.0f;
static float pFiltDps = 0.0f, qFiltDps = 0.0f, rFiltDps = 0.0f;
static float rollI = 0.0f, pitchI = 0.0f;
static float leftCmdFiltDeg = 0.0f, rightCmdFiltDeg = 0.0f;

// --- CONSTANTS ---
static const float _RAD_TO_DEG = 57.2957795f;
static const float ACC_GATE_LOW_G = 0.82f;
static const float ACC_GATE_HIGH_G = 1.18f;
static const float COMP_ALPHA = 0.985f;
static const float RATE_LPF_ALPHA = 0.22f;
static const float CMD_LPF_ALPHA = 0.35f;
static const float KP_ROLL = 0.55f, KI_ROLL = 0.00f, KD_ROLL = 0.040f;
static const float KP_PITCH = 0.50f, KI_PITCH = 0.00f, KD_PITCH = 0.040f;
static const float INTEGRATOR_LIMIT_DEG = 5.0f;
static const float CMD_LIMIT_DEG = 14.0f;
static const float WING_NEUTRAL_DEG = 90.0f;
static const float SERVO_RATE_LIMIT_DEG_PER_S = 120.0f;

static float clampf(float x, float lo, float hi) { return (x < lo) ? lo : ((x > hi) ? hi : x); }

static void setImuHighSpeed() {
  Serial.println(F("# Boosting IMU ODR to 833Hz..."));
  Wire.beginTransmission(0x6A);
  Wire.write(0x10); Wire.write(0x7A); // XL: 833Hz, 4g
  if (Wire.endTransmission() != 0) Serial.println(F("# ERROR: XL ODR Fail"));

  Wire.beginTransmission(0x6A);
  Wire.write(0x11); Wire.write(0x7C); // G: 833Hz, 2000dps
  if (Wire.endTransmission() != 0) Serial.println(F("# ERROR: G ODR Fail"));
}

static bool readImu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint32_t start = micros();
  // Wait up to 3ms for fresh data to support 500Hz
  while (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
    if (micros() - start > 3000) return false; 
  }
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  return true;
}

void flushBuffer() {
  if (!sdAvailable || bufferIdx == 0) return;
  
  // Local copy of index to prevent issues if interrupt-driven (though not used here)
  int count = bufferIdx;
  bufferIdx = 0; 

  for (int i = 0; i < count; i++) {
    logFile.print(buffer[i].ms); logFile.print(',');
    logFile.print(buffer[i].dt_us); logFile.print(',');
    logFile.print(buffer[i].armed ? 1 : 0); logFile.print(',');
    logFile.print(buffer[i].ax, 4); logFile.print(',');
    logFile.print(buffer[i].ay, 4); logFile.print(',');
    logFile.print(buffer[i].az, 4); logFile.print(',');
    logFile.print(buffer[i].gx, 3); logFile.print(',');
    logFile.print(buffer[i].gy, 3); logFile.print(',');
    logFile.print(buffer[i].gz, 3); logFile.print(',');
    logFile.print(buffer[i].roll, 2); logFile.print(',');
    logFile.print(buffer[i].pitch, 2); logFile.print(',');
    logFile.print(buffer[i].p, 2); logFile.print(',');
    logFile.print(buffer[i].q, 2); logFile.print(',');
    logFile.print(buffer[i].r, 2); logFile.print(',');
    logFile.print(buffer[i].roll_i, 3); logFile.print(',');
    logFile.print(buffer[i].pitch_i, 3); logFile.print(',');
    logFile.print(buffer[i].l_cmd, 2); logFile.print(',');
    logFile.print(buffer[i].r_cmd, 2); logFile.print(',');
    logFile.print(buffer[i].l_servo, 2); logFile.print(',');
    logFile.println(buffer[i].r_servo, 2);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000); // Give user time to open monitor
  Serial.println(F("\n\n# --- Samara V2 Flight Controller (v62-Logged) ---"));

  // 1. IMU Initialization
  Serial.print(F("# Init IMU... "));
  if (!IMU.begin()) {
    Serial.println(F("FAILED. Check board."));
    while (true);
  }
  Serial.println(F("OK."));
  setImuHighSpeed();

  // 2. Servo Setup
  Serial.print(F("# Init Servos... "));
  for (int i = 0; i < SERVO_COUNT; i++) servos[i].attach(SERVO_PINS[i]);
  Serial.println(F("OK."));

  // 3. SD Card Setup
  Serial.print(F("# Init SD Card... "));
  SPI.setRX(SD_MISO_PIN); SPI.setTX(SD_MOSI_PIN); SPI.setSCK(SD_SCK_PIN); SPI.begin();
  if (!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(12)))) {
    Serial.println(F("FAILED. (Card missing or pin error)"));
  } else {
    for (int i = 0; i < 1000; i++) {
      sprintf(logFileName, "fl%03d.csv", i);
      if (!sd.exists(logFileName)) break;
    }
    if (logFile.open(logFileName, O_WRONLY | O_CREAT | O_EXCL)) {
      sdAvailable = true;
      logFile.println(F("ms,dt_us,armed,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,roll_est,pitch_est,p_filt,q_filt,r_filt,roll_i,pitch_i,l_cmd,r_cmd,ls,rs"));
      logFile.sync();
      Serial.print(F("OK. Logging to: ")); Serial.println(logFileName);
    } else {
      Serial.println(F("FILE OPEN ERROR."));
    }
  }

  // 4. Calibration
  Serial.println(F("# Calibrating IMU (Keep Still)..."));
  float sumG[3] = {0,0,0};
  uint16_t good = 0;
  for (uint16_t i = 0; i < 500; i++) {
    float ax, ay, az, gx, gy, gz;
    if (readImu(ax, ay, az, gx, gy, gz)) {
      sumG[0] += gx; sumG[1] += gy; sumG[2] += gz;
      float a = sqrtf(ax*ax + ay*ay + az*az);
      if (a >= 0.82f && a <= 1.18f) {
        rollDeg = atan2f(ay, az) * _RAD_TO_DEG;
        pitchDeg = atan2f(-ax, sqrtf(ay*ay + az*az)) * _RAD_TO_DEG;
        good++;
      }
    }
    delay(4);
  }
  for(int i=0; i<3; i++) gyroBiasDps[i] = sumG[i] / 500.0f;
  estimatorReady = (good > 250);
  Serial.println(F("# Ready for Flight."));
}

void loop() {
  // Command Processing
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'a') { armed = true; Serial.println(F("# ARMED")); }
    else if (c == 'd') { armed = false; Serial.println(F("# DISARMED")); }
    else if (c == 'b') { if(sdAvailable) { flushBuffer(); logFile.sync(); } rp2040.rebootToBootloader(); }
  }

  static uint32_t lastLoopUs = micros();
  uint32_t nowUs = micros();

  // 500Hz Loop
  if ((int32_t)(nowUs - lastLoopUs) >= CONTROL_DT_US) {
    uint32_t actual_dt_us = nowUs - lastLoopUs;
    float dt = actual_dt_us * 1e-6f;
    lastLoopUs = nowUs;
    if (dt > 0.05f) dt = 0.002f;

    float ax, ay, az, gx, gy, gz;
    if (readImu(ax, ay, az, gx, gy, gz)) {
      // 1. Estimator
      float p = gx - gyroBiasDps[0], q = gy - gyroBiasDps[1], r = gz - gyroBiasDps[2];
      pFiltDps += RATE_LPF_ALPHA * (p - pFiltDps);
      qFiltDps += RATE_LPF_ALPHA * (q - qFiltDps);
      rFiltDps += RATE_LPF_ALPHA * (r - rFiltDps);
      rollDeg += pFiltDps * dt; pitchDeg += qFiltDps * dt;
      
      float a_norm = sqrtf(ax*ax + ay*ay + az*az);
      if (a_norm >= ACC_GATE_LOW_G && a_norm <= ACC_GATE_HIGH_G) {
        rollDeg = COMP_ALPHA * rollDeg + (1.0f - COMP_ALPHA) * (atan2f(ay, az) * _RAD_TO_DEG);
        pitchDeg = COMP_ALPHA * pitchDeg + (1.0f - COMP_ALPHA) * (atan2f(-ax, sqrtf(ay*ay + az*az)) * _RAD_TO_DEG);
      }

      // 2. PID
      if (armed && estimatorReady) {
        rollI = clampf(rollI + (rollDeg)*dt, -INTEGRATOR_LIMIT_DEG, INTEGRATOR_LIMIT_DEG);
        pitchI = clampf(pitchI + (pitchDeg)*dt, -INTEGRATOR_LIMIT_DEG, INTEGRATOR_LIMIT_DEG);
        float uR = -KP_ROLL * rollDeg - KI_ROLL * rollI - KD_ROLL * pFiltDps;
        float uP = -KP_PITCH * pitchDeg - KI_PITCH * pitchI - KD_PITCH * qFiltDps;
        uR = clampf(uR, -CMD_LIMIT_DEG, CMD_LIMIT_DEG);
        uP = clampf(uP, -CMD_LIMIT_DEG, CMD_LIMIT_DEG);
        leftCmdFiltDeg += CMD_LPF_ALPHA * (clampf(uP + uR, -CMD_LIMIT_DEG, CMD_LIMIT_DEG) - leftCmdFiltDeg);
        rightCmdFiltDeg += CMD_LPF_ALPHA * (clampf(uP - uR, -CMD_LIMIT_DEG, CMD_LIMIT_DEG) - rightCmdFiltDeg);
        
        float l = WING_NEUTRAL_DEG + leftCmdFiltDeg;
        float r = WING_NEUTRAL_DEG + rightCmdFiltDeg;
        const float step = SERVO_RATE_LIMIT_DEG_PER_S * dt;
        servoDeg[0] = clampf(l, servoDeg[0] - step, servoDeg[0] + step);
        servoDeg[1] = clampf(r, servoDeg[1] - step, servoDeg[1] + step);
        servos[0].writeMicroseconds(1000 + (servoDeg[0]/180.0f)*1000);
        servos[1].writeMicroseconds(1000 + (servoDeg[1]/180.0f)*1000);
      } else {
        for(int i=0; i<SERVO_COUNT; i++) servos[i].write(90);
      }

      // 3. Log to Buffer
      if (sdAvailable && bufferIdx < BUFFER_SIZE) {
        LogEntry &e = buffer[bufferIdx];
        e.ms = millis();
        e.dt_us = (uint16_t)actual_dt_us;
        e.armed = armed;
        e.ax = ax; e.ay = ay; e.az = az;
        e.gx = gx; e.gy = gy; e.gz = gz;
        e.roll = rollDeg; e.pitch = pitchDeg;
        e.p = pFiltDps; e.q = qFiltDps; e.r = rFiltDps;
        e.roll_i = rollI; e.pitch_i = pitchI;
        e.l_cmd = leftCmdFiltDeg; e.r_cmd = rightCmdFiltDeg;
        e.l_servo = servoDeg[0]; e.r_servo = servoDeg[1];
        bufferIdx++;
      }
    }
    
    // Auto-flush every 200ms (100 samples)
    if (bufferIdx >= 100) {
      flushBuffer();
    }
  }

  // Periodic Hard Sync (Power loss protection)
  static uint32_t lastSync = 0;
  if (millis() - lastSync > 1000) {
    if (sdAvailable) logFile.sync();
    lastSync = millis();
  }
}
