#include <SPI.h>
#include <SdFat.h>

SdFat sd;
#define CS 10

void setup() {
  Serial.begin(115200);
  delay(2000);

  SPI.setRX(12);
  SPI.setTX(11);
  SPI.setSCK(13);
  SPI.setCS(10);
  SPI.begin();

  Serial.println("SdFat SD init test...");

  if (sd.begin(SdSpiConfig(CS, SHARED_SPI, SD_SCK_MHZ(4)))) {
    Serial.println("SD init OK");
    uint32_t size = sd.card()->sectorCount();
    Serial.print("Card size: ");
    Serial.print(size / 2048UL);
    Serial.println(" MB");
  } else {
    Serial.println("SD init FAIL:");
    sd.initErrorPrint(&Serial);
  }
}

void loop() {}
