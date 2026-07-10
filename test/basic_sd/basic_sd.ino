#include <SPI.h>
#include "SdFat.h"

// In Philhower core:
// D10 = GPIO 5 (CS)
// D11 = GPIO 7 (MOSI)
// D12 = GPIO 4 (MISO)
// D13 = GPIO 6 (SCK)

#define SD_CS_PIN 5
#define SD_MOSI_PIN 7
#define SD_MISO_PIN 4
#define SD_SCK_PIN 6

SdFat sd;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Serial.println("\n--- Nano RP2040 Connect SD Card Test ---");
  
  // Set explicit pins for Philhower core
  SPI.setRX(SD_MISO_PIN);
  SPI.setTX(SD_MOSI_PIN);
  SPI.setSCK(SD_SCK_PIN);
  SPI.begin();
  
  // Optional: Add pull-up to MISO
  pinMode(SD_MISO_PIN, INPUT_PULLUP);

  Serial.println("Initializing SD card at 1MHz...");
  
  if (!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(1)))) {
    Serial.println("Initialization failed!");
    sd.initErrorPrint(&Serial);
  } else {
    Serial.println("Initialization SUCCESS!");
    sd.ls(&Serial, LS_R);
  }
}

void loop() {
  delay(1000);
}


