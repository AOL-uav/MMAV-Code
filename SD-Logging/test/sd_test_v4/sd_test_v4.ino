#include <SPI.h>
#include "SdFat.h"

#define CS_PIN 10

SdFat sd;
bool initialized = false;
bool result = false;

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (!initialized) {
    delay(5000);
    Serial.println("--- SdFat SD card test v4 ---");
    Serial.println("Initializing at 100kHz...");
    result = sd.begin(SdSpiConfig(CS_PIN, SHARED_SPI, SD_SCK_MHZ(0.1)));
    initialized = true;
  }
  
  if (result) {
    Serial.println("SD OK!");
  } else {
    Serial.print("SD FAIL. Error: ");
    sd.initErrorPrint(&Serial);
  }
  
  delay(2000);
}
