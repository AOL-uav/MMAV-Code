#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <Servo.h>
#include <math.h>
#include <SPI.h>
#include "SdFat.h"

/*
  Samara V2 - High Speed Buffered Logger (500Hz)
  Purpose: Measure actuator delay by logging at 2ms resolution.
  Uses a RAM buffer to prevent SD card "hiccups" from slowing down the loop.
*/

// --- HARDWARE CONFIG ---
#define SD_CS_PIN 5
#define SD_MOSI_PIN 7
#define SD_MISO_PIN 4
#define SD_SCK_PIN 6

// --- TIMING CONFIG ---
static const float CONTROL_HZ = 500.0f; // 2ms loop
static const uint32_t CONTROL_DT_US = 2000;

// --- BUFFER CONFIG ---
// We'll store data in a struct to keep it compact in RAM
struct LogEntry {
  uint32_t ms;
  float roll;
  float pitch;
  float gx;
  float gy;
  float cmd;
};

#define BUFFER_SIZE 500 // Stores 1 second of 500Hz data in RAM
LogEntry buffer[BUFFER_SIZE];
int bufferIdx = 0;

SdFat sd;
FsFile logFile;
char logFileName[16];
bool sdAvailable = false;
bool recording = true; // Auto-start high-speed logging
bool armed = false;

static const int SERVO_PINS[4] = {2, 3, 4, 5};
static Servo servos[4];
static float _RAD_TO_DEG = 57.2957795f;

void handleCommand(char cmd) {
  if (cmd == 'a') {
    recording = true;
    bufferIdx = 0;
    Serial.println(F("# BURST RESUMED"));
  } else if (cmd == 'd') {
    recording = false;
    Serial.println(F("# PAUSED. Saving..."));
    flushBuffer();
  } else if (cmd == 's') { // STEP TEST
    Serial.println(F("# STEP COMMAND SENT"));
    for(int i=0; i<4; i++) servos[i].write(180);
    // The log will show exactly when this happened relative to IMU
  } else if (cmd == 'n') {
    for(int i=0; i<4; i++) servos[i].write(90);
    Serial.println(F("# NEUTRAL"));
  } else if (cmd == 'b') {
    rp2040.rebootToBootloader();
  }
}

void flushBuffer() {
  if (!sdAvailable) return;
  Serial.print(F("# Writing ")); Serial.print(bufferIdx); Serial.println(F(" lines to SD..."));
  for (int i = 0; i < bufferIdx; i++) {
    logFile.print(buffer[i].ms); logFile.print(',');
    logFile.print(buffer[i].roll, 2); logFile.print(',');
    logFile.print(buffer[i].pitch, 2); logFile.print(',');
    logFile.print(buffer[i].gx, 2); logFile.print(',');
    logFile.print(buffer[i].gy, 2); logFile.print(',');
    logFile.println(buffer[i].cmd, 2);
  }
  logFile.sync();
  Serial.println(F("# Done."));
  bufferIdx = 0;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  if (!IMU.begin()) { Serial.println(F("# IMU FAIL")); while(1); }
  
  for (int i = 0; i < 4; i++) servos[i].attach(SERVO_PINS[i]);

  SPI.setRX(SD_MISO_PIN); SPI.setTX(SD_MOSI_PIN); SPI.setSCK(SD_SCK_PIN); SPI.begin();
  if (sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(12)))) {
    sprintf(logFileName, "burst000.csv");
    for(int i=0; i<100; i++) {
      sprintf(logFileName, "bt%02d.csv", i);
      if(!sd.exists(logFileName)) break;
    }
    if (logFile.open(logFileName, O_WRONLY | O_CREAT | O_EXCL)) {
      sdAvailable = true;
      logFile.println(F("ms,roll,pitch,gx,gy,cmd"));
      Serial.print(F("# Log: ")); Serial.println(logFileName);
    }
  }

  Serial.println(F("# 500Hz Logger Ready. Commands: a=start, d=stop/save, s=step, n=neutral, b=boot"));
}

void loop() {
  if (Serial.available()) handleCommand(Serial.read());

  static uint32_t nextUs = micros();
  if ((int32_t)(micros() - nextUs) >= 0) {
    nextUs += CONTROL_DT_US;

    float ax, ay, az, gx, gy, gz;
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      IMU.readGyroscope(gx, gy, gz);

      if (recording && bufferIdx < BUFFER_SIZE) {
        buffer[bufferIdx].ms = millis();
        buffer[bufferIdx].roll = atan2f(ay, az) * _RAD_TO_DEG;
        buffer[bufferIdx].pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * _RAD_TO_DEG;
        buffer[bufferIdx].gx = gx;
        buffer[bufferIdx].gy = gy;
        buffer[bufferIdx].cmd = (float)servos[0].read(); // Log current command
        bufferIdx++;
        
        // Auto-flush in smaller chunks (0.5 seconds) to ensure data safety
        if (bufferIdx >= 250) { 
          flushBuffer(); 
          logFile.sync(); // Burn to chips immediately
        }
      }
    }
  }
}
