#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("--- SD Card CS Scanner ---");

  bool found = false;
  for (int pin = 0; pin < 30; pin++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
  delay(100);

  for (int pin = 0; pin < 30; pin++) {
    Serial.print("Testing pin ");
    Serial.print(pin);
    Serial.print("... ");
    if (SD.begin(pin)) {
      Serial.println("SUCCESS!");
      found = true;
      break;
    } else {
      Serial.println("failed.");
    }
  }

  if (!found) {
    Serial.println("Could not find SD card on any pin.");
  }
}

void loop() {
  delay(1000);
}

