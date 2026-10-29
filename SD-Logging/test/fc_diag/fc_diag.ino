#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <SPI.h>
#include "SdFat.h"

// SD Pins for Nano RP2040 Connect on Philhower core
#define SD_CS_PIN 5
#define SD_MOSI_PIN 7
#define SD_MISO_PIN 4
#define SD_SCK_PIN 6

SdFat sd;

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("--- FC Diagnostic ---");

  Serial.println("Initializing IMU...");
  if (IMU.begin()) {
    Serial.println("IMU OK");
  } else {
    Serial.println("IMU FAIL");
  }

  Serial.println("Initializing SPI...");
  SPI.setRX(SD_MISO_PIN);
  SPI.setTX(SD_MOSI_PIN);
  SPI.setSCK(SD_SCK_PIN);
  SPI.begin();
  Serial.println("SPI Started");

  Serial.println("Initializing SD...");
  if (sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(12)))) {
    Serial.println("SD OK");
  } else {
    Serial.println("SD FAIL");
  }
  
  Serial.println("Diagnostic Complete.");
}

void loop() {
  Serial.println("Alive...");
  delay(1000);
}
