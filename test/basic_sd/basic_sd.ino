#include <Arduino.h>
#include <SPI.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
void loop() {
  Serial.println("Raw SPI SD CMD0 Test...");
  
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  pinMode(12, INPUT_PULLUP);
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);
  SPI.begin();
  
  // 80 dummy clocks with CS high to initialize SD
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
  for (int i = 0; i < 10; i++) SPI.transfer(0xFF);
  
  // Send CMD0 (GO_IDLE_STATE)
  digitalWrite(10, LOW);
  SPI.transfer(0x40 | 0); // CMD0
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x95);     // CRC for CMD0
  
  // Read response
  uint8_t res = 0xFF;
  for (int i = 0; i < 10; i++) {
    res = SPI.transfer(0xFF);
    if (res != 0xFF) break;
  }
  digitalWrite(10, HIGH);
  SPI.endTransaction();
  
  Serial.print("CMD0 Response: 0x");
  Serial.println(res, HEX);
  
  if (res == 0x01) {
    Serial.println("Hardware SPI is ALIVE and SD card responded!");
  } else {
    Serial.println("SPI failed or SD card did not respond properly.");
  }
  
  delay(2000);
}
