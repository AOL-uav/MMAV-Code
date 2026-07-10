#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("Starting SD brute force test...");
  
  int cs_pins[] = {10, 5, 4, 8, 9, 7};
  bool success = false;
  
  for (int i = 0; i < 6; i++) {
    int cs = cs_pins[i];
    Serial.print("Trying CS = ");
    Serial.println(cs);
    
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    SPI.begin();
    delay(50);
    
    if (SD.begin(1000000, cs)) {
      Serial.println("SUCCESS on CS = " + String(cs));
      success = true;
      break;
    } else {
      Serial.println("Failed on CS = " + String(cs));
    }
    SPI.end();
    delay(50);
  }
  
  if (!success) {
    Serial.println("All CS pins failed.");
  }
}

void loop() {
  delay(1000);
}
