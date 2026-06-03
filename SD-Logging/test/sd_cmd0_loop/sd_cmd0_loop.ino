#include <SPI.h>

#define CS_PIN 10
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN 13

void setup() {
  Serial.begin(115200);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT_PULLUP); // Use pullup to see if card pulls it low
}

void loop() {
  Serial.println("--- Manual SD CMD0 Test ---");
  
  digitalWrite(CS_PIN, HIGH);
  for (int i = 0; i < 80; i++) {
    digitalWrite(SCK_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(SCK_PIN, LOW); delayMicroseconds(10);
  }

  digitalWrite(CS_PIN, LOW);
  byte cmd0[] = {0x40, 0x00, 0x00, 0x00, 0x00, 0x95};
  for (int i = 0; i < 6; i++) {
    shiftOut(MOSI_PIN, SCK_PIN, MSBFIRST, cmd0[i]);
  }

  byte resp = 0xFF;
  for (int i = 0; i < 200; i++) {
    digitalWrite(SCK_PIN, HIGH); delayMicroseconds(10);
    resp = (resp << 1) | digitalRead(MISO_PIN);
    digitalWrite(SCK_PIN, LOW); delayMicroseconds(10);
    if ((resp & 0x80) == 0) break; // First bit of response is 0
  }
  
  Serial.print("Response: 0x"); Serial.println(resp, HEX);
  digitalWrite(CS_PIN, HIGH);
  
  delay(5000);
}
