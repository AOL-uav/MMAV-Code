#include <SPI.h>
#include "SdFat.h"

#define SD_CS_PIN 5
#define SD_MOSI_PIN 7
#define SD_MISO_PIN 4
#define SD_SCK_PIN 6

SdFat sd;
bool initSuccess = false;
bool tried = false;

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (!tried) {
    delay(5000); // Give serial monitor time to connect
    Serial.println("\n--- Nano RP2040 Connect SD Card Test ---");
    SPI.setRX(SD_MISO_PIN);
    SPI.setTX(SD_MOSI_PIN);
    SPI.setSCK(SD_SCK_PIN);
    SPI.begin();
    pinMode(SD_MISO_PIN, INPUT_PULLUP);

    Serial.println("Initializing SD card at 1MHz...");
    if (!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(1)))) {
      Serial.println("Initialization failed!");
      sd.initErrorPrint(&Serial);
    } else {
      Serial.println("Initialization SUCCESS!");
      initSuccess = true;
    }
    tried = true;
  }
  
  if (initSuccess) {
    Serial.println("SD OK!");
  } else {
    Serial.println("SD FAIL!");
  }
  delay(2000);
}



