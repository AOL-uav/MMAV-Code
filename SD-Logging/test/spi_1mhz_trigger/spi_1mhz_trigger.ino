#include <SPI.h>

void setup() {
  Serial.begin(115200);
  delay(5000); 
  Serial.println("--- SPI 1MHz Trigger Test ---");
  
  SPI.begin();
  // Set speed to 1MHz
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  Serial.println("SPI.begin() and beginTransaction(1MHz) OK.");
}

void loop() {
  Serial.println("Waiting for 'g'...");
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') {
      Serial.println("Triggering 1MHz transfer!");
      byte result = SPI.transfer(0xA5);
      Serial.print("Result: 0x"); Serial.println(result, HEX);
    }
  }
  delay(1000);
}
