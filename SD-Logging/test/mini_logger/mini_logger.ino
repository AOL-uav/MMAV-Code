#include <Arduino.h>
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

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("--- Minimal Logger Test ---");

  // Initialize SD card
  SPI.setRX(SD_MISO_PIN);
  SPI.setTX(SD_MOSI_PIN);
  SPI.setSCK(SD_SCK_PIN);
  SPI.begin();

  if (!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(12)))) {
    Serial.println(F("SD init failed."));
  } else {
    for (int i = 0; i < 1000; i++) {
      sprintf(logFileName, "test%03d.csv", i);
      if (!sd.exists(logFileName)) break;
    }
    if (logFile.open(logFileName, O_WRONLY | O_CREAT | O_EXCL)) {
      sdAvailable = true;
      logFile.println(F("ms,counter"));
      Serial.print(F("Logging to: ")); Serial.println(logFileName);
    }
  }
}

void loop() {
  static uint32_t counter = 0;
  uint32_t now = millis();
  
  Serial.print(now); Serial.print(","); Serial.println(counter);
  
  if (sdAvailable) {
    logFile.print(now); logFile.print(","); logFile.println(counter);
    static uint32_t lastSync = 0;
    if (now - lastSync > 2000) {
      logFile.sync();
      lastSync = now;
      Serial.println("Syncing...");
    }
  }
  
  counter++;
  delay(10); // 100Hz
}
