#include <SPI.h>

void setup() {
  Serial.begin(115200);
  // Remove while(!Serial) to avoid hang if monitor isn't ready
  delay(5000); 
  Serial.println("--- SPI Trigger Test v2 ---");
  
  SPI.begin();
  Serial.println("SPI.begin() OK.");
}

void loop() {
  Serial.println("Waiting for 'g'...");
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') {
      Serial.println("Triggering transfer!");
      byte result = SPI.transfer(0xA5);
      Serial.print("Result: 0x"); Serial.println(result, HEX);
    }
  }
  delay(1000);
}
