#include <SPI.h>

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("--- Simple SPI Loopback (No Pin Remap) ---");
  
  SPI.begin();
  Serial.println("SPI started successfully.");
}

void loop() {
  byte send = 0xA5;
  byte recv = SPI.transfer(send);
  Serial.print("Sent: 0x"); Serial.print(send, HEX);
  Serial.print(" Recv: 0x"); Serial.println(recv, HEX);
  delay(1000);
}
