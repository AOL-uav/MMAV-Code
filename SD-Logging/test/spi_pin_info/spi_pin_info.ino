#include <SPI.h>

bool printed = false;

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (!printed) {
    delay(5000); // Give monitor time to connect
    Serial.println("--- SPI Pin Info ---");
    Serial.print("MISO (D12): "); Serial.println(MISO);
    Serial.print("MOSI (D11): "); Serial.println(MOSI);
    Serial.print("SCK (D13): "); Serial.println(SCK);
    Serial.print("SS (D10): "); Serial.println(SS);
    printed = true;
  }
  Serial.println("Heartbeat...");
  delay(2000);
}
