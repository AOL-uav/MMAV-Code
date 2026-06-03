#include <SPI.h>

#define CS_PIN 10
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN 13

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("--- Manual SD CMD0 Test ---");
  
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);

  Serial.println("Sending 80 dummy clocks...");
  digitalWrite(CS_PIN, HIGH);
  for (int i = 0; i < 80; i++) {
    digitalWrite(SCK_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(SCK_PIN, LOW); delayMicroseconds(10);
  }

  Serial.println("Sending CMD0...");
  digitalWrite(CS_PIN, LOW);
  
  // CMD0: 0x40, 0x00, 0x00, 0x00, 0x00, 0x95
  byte cmd0[] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
  for (int i = 0; i < 6; i++) {
    shiftOut(MOSI_PIN, SCK_PIN, MSBFIRST, cmd0[i]);
  }

  Serial.println("Waiting for response (max 100 tries)...");
  byte resp = 0xFF;
  for (int i = 0; i < 100; i++) {
    digitalWrite(SCK_PIN, HIGH); delayMicroseconds(10);
    resp = (resp << 1) | digitalRead(MISO_PIN);
    digitalWrite(SCK_PIN, LOW); delayMicroseconds(10);
    // This is a bit-bang read, simplified
    if (resp != 0xFF) break;
  }
  
  Serial.print("Response: 0x"); Serial.println(resp, HEX);
  digitalWrite(CS_PIN, HIGH);
}

void loop() {
  delay(1000);
}
