#include <SPI.h>

void setup() {
  Serial.begin(115200);
  delay(5000); 
  Serial.println("--- SPI Slow Trigger Test ---");
  
  SPI.begin();
  // Set speed to 100kHz
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  Serial.println("SPI.begin() and beginTransaction(100kHz) OK.");
}

void loop() {
  Serial.println("Waiting for 'g'...");
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') {
      Serial.println("Triggering SLOW transfer!");
      byte result = SPI.transfer(0xA5);
      Serial.print("Result: 0x"); Serial.println(result, HEX);
    }
  }
  delay(1000);
}
