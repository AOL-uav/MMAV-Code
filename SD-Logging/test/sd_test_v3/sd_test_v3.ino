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
    Serial.println("--- SdFat SD card test (Explicit loop) ---");
    Serial.println("Initializing...");
    result = sd.begin(SdSpiConfig(CS_PIN, SHARED_SPI, SD_SCK_MHZ(0.1)));
    initialized = true;
    Serial.print("Init finished. Success: ");
    Serial.println(result ? "YES" : "NO");
    if (!result) {
      sd.initErrorPrint(&Serial);
    }
  }
  
  Serial.print("Alive. Result was: ");
  Serial.println(result ? "OK" : "FAIL");
  delay(2000);
}
