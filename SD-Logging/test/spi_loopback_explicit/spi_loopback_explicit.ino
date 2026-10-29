#include <SPI.h>

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("SPI Loopback with EXPLICIT GPIOs");
  
  // GPIO 4 = D12 (MISO)
  // GPIO 7 = D11 (MOSI)
  // GPIO 6 = D13 (SCK)
  SPI.setRX(4);
  SPI.setTX(7);
  SPI.setSCK(6);
  
  Serial.println("Calling SPI.begin()...");
  SPI.begin();
  Serial.println("SPI started.");
}

void loop() {
  byte send = 0xA5;
  byte recv = SPI.transfer(send);
  Serial.print("Sent: 0x"); Serial.print(send, HEX);
  Serial.print(" Recv: 0x"); Serial.println(recv, HEX);
  delay(1000);
}
