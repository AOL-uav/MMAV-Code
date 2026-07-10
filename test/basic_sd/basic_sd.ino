#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Serial.println("\n--- Pin Mappings ---");
  Serial.print("D10: "); Serial.println(D10);
  Serial.print("D11: "); Serial.println(D11);
  Serial.print("D12: "); Serial.println(D12);
  Serial.print("D13: "); Serial.println(D13);
  Serial.print("MISO: "); Serial.println(MISO);
  Serial.print("MOSI: "); Serial.println(MOSI);
  Serial.print("SCK: "); Serial.println(SCK);
  Serial.print("SS: "); Serial.println(SS);
}

void loop() {
  delay(1000);
}




