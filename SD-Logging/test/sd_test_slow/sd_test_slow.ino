#include <SPI.h>
#include "SdFat.h"

#define CS_PIN 10

SdFat sd;

void setup() {
  Serial.begin(115200);
  delay(5000); 
  Serial.println("--- SdFat SD card test (Slow Clock) ---");

  Serial.println("Configuring SPI pins...");
  SPI.setRX(12);
  SPI.setTX(11);
  SPI.setSCK(13);
  SPI.begin();
  Serial.println("SPI pins configured.");

  Serial.println("Initializing SD card on CS pin 10 at 1MHz...");
  
  // Use 1MHz for maximum stability
  if (!sd.begin(SdSpiConfig(CS_PIN, SHARED_SPI, SD_SCK_MHZ(1)))) {
    Serial.println("SD initialization failed!");
    sd.initErrorPrint(&Serial);
  } else {
    Serial.println("SD initialization OK!");
    
    uint32_t size = sd.card()->sectorCount();
    if (size == 0) {
      Serial.println("Can't determine the card size.");
    } else {
      uint32_t sizeMB = size / 2048;
      Serial.print("Card size: ");
      Serial.print(sizeMB);
      Serial.println(" MB");
    }
  }
}

void loop() {
  Serial.println("Heartbeat...");
  delay(5000);
}
