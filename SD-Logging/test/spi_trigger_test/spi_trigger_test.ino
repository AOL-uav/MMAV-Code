#include <SPI.h>

void setup() {
  Serial.begin(115200);
  while(!Serial); 
  delay(2000);
  Serial.println("SPI Ready. Type 'g' to start transfer...");
  
  SPI.begin();
  Serial.println("SPI.begin() OK.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') {
      Serial.println("Starting transfer...");
      byte result = SPI.transfer(0xA5);
      Serial.print("Result: 0x"); Serial.println(result, HEX);
    }
  }
}
