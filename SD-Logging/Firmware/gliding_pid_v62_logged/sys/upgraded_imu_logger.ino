#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <SPI.h>
#include "SdFat.h"
#include <math.h>

/*
  Samara V2 - High-Speed IMU Logger (500Hz)
  Features:
    - Auto-starts recording on power-up
    - High-frequency sampling (500Hz / 2ms resolution)
    - Manual IMU ODR override to 833Hz (ensures fresh 500Hz data)
    - Power-loss resilience: Sync every 1s
*/

// --- HARDWARE CONFIG ---
#define SD_CS_PIN 5
#define SD_MOSI_PIN 7
#define SD_MISO_PIN 4
#define SD_SCK_PIN 6

// --- LOGGING CONFIG ---
static const float SAMPLE_HZ = 500.0f; 
static const uint32_t SAMPLE_DT_US = 2000; // 2ms

struct LogEntry {
  uint32_t ms;
  float ax, ay, az;
  float gx, gy, gz;
};

// 1 second buffer at 500Hz
#define BUFFER_SIZE 500 
LogEntry buffer[BUFFER_SIZE];
int bufferIdx = 0;

SdFat sd;
FsFile logFile;
char logFileName[16];
bool sdAvailable = false;

// Manual ODR override for LSM6DSOX (Arduino library defaults to 104Hz)
void setImuHighSpeed() {
  // CTRL1_XL: 0x80 = 1.66kHz, 0x70 = 833Hz, 0x60 = 416Hz
  // Using 833Hz (0x70) to safely cover our 500Hz request
  // 4g range, bypass mode, LPF1 enabled: 0x7A
  
  // First, find the registers (from LSM6DSOX.cpp)
  // #define LSM6DSOX_CTRL1_XL 0X10
  // #define LSM6DSOX_CTRL2_G  0X11
  
  // We have to use raw writes because the library doesn't expose ODR
  // The library's 'readRegister' is private, but we can just use Wire directly
  // since the Nano RP2040 uses I2C for the onboard IMU.
  
  Wire.beginTransmission(0x6A);
  Wire.write(0x10); // CTRL1_XL
  Wire.write(0x7A); // 833Hz, 4g
  Wire.endTransmission();

  Wire.beginTransmission(0x6A);
  Wire.write(0x11); // CTRL2_G
  Wire.write(0x7C); // 833Hz, 2000dps
  Wire.endTransmission();
}

static bool readImu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return false;
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  return true;
}

void flushBuffer() {
  if (!sdAvailable || bufferIdx == 0) return;
  for (int i = 0; i < bufferIdx; i++) {
    float norm = sqrtf(buffer[i].ax*buffer[i].ax + buffer[i].ay*buffer[i].ay + buffer[i].az*buffer[i].az);
    logFile.print(buffer[i].ms); logFile.print(',');
    logFile.print(buffer[i].ax, 4); logFile.print(',');
    logFile.print(buffer[i].ay, 4); logFile.print(',');
    logFile.print(buffer[i].az, 4); logFile.print(',');
    logFile.print(norm, 4); logFile.print(',');
    logFile.print(buffer[i].gx, 3); logFile.print(',');
    logFile.print(buffer[i].gy, 3); logFile.print(',');
    logFile.println(buffer[i].gz, 3);
  }
  bufferIdx = 0;
}

void setup() {
  Serial.begin(115200);
  
  if (!IMU.begin()) {
    while (true); 
  }
  setImuHighSpeed(); // Boost ODR to handle 500Hz

  SPI.setRX(SD_MISO_PIN);
  SPI.setTX(SD_MOSI_PIN);
  SPI.setSCK(SD_SCK_PIN);
  SPI.begin();

  if (sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(12)))) {
    for (int i = 0; i < 1000; i++) {
      sprintf(logFileName, "hi%03d.csv", i);
      if (!sd.exists(logFileName)) break;
    }
    if (logFile.open(logFileName, O_WRONLY | O_CREAT | O_EXCL)) {
      sdAvailable = true;
      logFile.println(F("ms,ax,ay,az,norm,gx,gy,gz"));
      logFile.sync();
    }
  }
}

void loop() {
  static uint32_t nextSampleUs = micros();
  uint32_t nowUs = micros();

  // 1. Data Collection (500Hz)
  if ((int32_t)(nowUs - nextSampleUs) >= 0) {
    nextSampleUs += SAMPLE_DT_US;
    
    float ax, ay, az, gx, gy, gz;
    if (readImu(ax, ay, az, gx, gy, gz)) {
      if (sdAvailable && bufferIdx < BUFFER_SIZE) {
        buffer[bufferIdx].ms = millis();
        buffer[bufferIdx].ax = ax;
        buffer[bufferIdx].ay = ay;
        buffer[bufferIdx].az = az;
        buffer[bufferIdx].gx = gx;
        buffer[bufferIdx].gy = gy;
        buffer[bufferIdx].gz = gz;
        bufferIdx++;

        // Flush buffer to file cache every 50 samples (100ms)
        if (bufferIdx >= 50) {
          flushBuffer();
        }
      }
    }
  }

  // 2. Power-Loss Resilience (Sync every 1s)
  static uint32_t lastSyncMs = 0;
  if (millis() - lastSyncMs > 1000) {
    if (sdAvailable) {
      logFile.sync(); 
    }
    lastSyncMs = millis();
  }

  if (Serial.available() && Serial.read() == 'b') {
    if(sdAvailable) { flushBuffer(); logFile.sync(); }
    rp2040.rebootToBootloader();
  }
}
