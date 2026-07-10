#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
  } else {
    Serial.println("initialization done.");
  }
  delay(2000);
}
