#include <SPI.h>

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Testing SPI pin configuration...");
  
  Serial.println("Setting RX/TX/SCK...");
  SPI.setRX(12);
  SPI.setTX(11);
  SPI.setSCK(13);
  
  Serial.println("Calling SPI.begin()...");
  SPI.begin();
  
  Serial.println("SPI.begin() successful.");
}

void loop() {
  Serial.println("Stable.");
  delay(1000);
}
