#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"

SdFs sd;
bool initSuccess = false;
bool tried = false;

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println("Initializing SD card...");

  if (!sd.begin(10)) {
    Serial.println("initialization failed!");
  } else {
    Serial.println("initialization done.");
  }
  delay(2000);
}
