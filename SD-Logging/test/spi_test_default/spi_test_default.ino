#include <SPI.h>

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Testing DEFAULT SPI pins...");
  
  Serial.print("MISO: "); Serial.println(MISO);
  Serial.print("MOSI: "); Serial.println(MOSI);
  Serial.print("SCK: "); Serial.println(SCK);
  Serial.print("SS: "); Serial.println(SS);

  Serial.println("Calling SPI.begin()...");
  SPI.begin();
  
  Serial.println("SPI.begin() successful.");
}

void loop() {
  Serial.println("Stable.");
  delay(1000);
}
