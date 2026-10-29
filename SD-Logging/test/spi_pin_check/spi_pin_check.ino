#include <SPI.h>

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Checking SPI pins (No begin)...");
  Serial.print("MISO: "); Serial.println(MISO);
  Serial.print("MOSI: "); Serial.println(MOSI);
  Serial.print("SCK: "); Serial.println(SCK);
}

void loop() {
  Serial.println("Looping...");
  delay(1000);
}
