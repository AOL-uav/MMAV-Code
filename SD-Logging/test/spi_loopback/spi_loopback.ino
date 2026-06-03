#include <SPI.h>

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Philhower core specific pin mapping
  SPI.setRX(12);
  SPI.setTX(11);
  SPI.setSCK(13);
  SPI.begin();

  Serial.println("Starting SPI loopback test...");
}

void loop() {
  byte result = SPI.transfer(0xA5);
  Serial.print("Loopback result: 0x");
  Serial.println(result, HEX);
  delay(1000);
}
