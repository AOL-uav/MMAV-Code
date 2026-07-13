#include "SdFat.h"
#include <SPI.h>

#define SD_CS_PIN 5
#define SD_MISO_PIN 4
#define SD_MOSI_PIN 7
#define SD_SCK_PIN 6

SdFat sd;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  
  Serial.println("\nTesting SD card on Philhower Core...");
  
  SPI.setRX(SD_MISO_PIN);
  SPI.setTX(SD_MOSI_PIN);
  SPI.setSCK(SD_SCK_PIN);
  SPI.begin();
  
  if (!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(12)))) {
    Serial.println("SD Initialization Failed!");
    if (sd.card()->errorCode()) {
      Serial.print("SD error code: 0x");
      Serial.println(sd.card()->errorCode(), HEX);
      Serial.print("SD error data: 0x");
      Serial.println(sd.card()->errorData(), HEX);
    }
  } else {
    Serial.println("SD Initialization Success!");
    sd.ls(LS_SIZE);
  }
}

void loop() {
  delay(1000);
}
