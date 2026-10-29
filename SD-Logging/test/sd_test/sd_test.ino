#include <SPI.h>
#include "SdFat.h"

#define CS_PIN 10

SdFat sd;

void setup() {
  Serial.begin(115200);
  delay(5000); // Give plenty of time to connect serial monitor
  Serial.println("--- SdFat SD card test ---");

  // Philhower core pin mapping
  // MOSI: D11, MISO: D12, SCK: D13
  SPI.setRX(12);
  SPI.setTX(11);
  SPI.setSCK(13);
  SPI.begin();

  Serial.println("Initializing SD card on CS pin 10...");
  
  // Use lower speed for first test: 4MHz
  if (!sd.begin(SdSpiConfig(CS_PIN, SHARED_SPI, SD_SCK_MHZ(4)))) {
    Serial.println("SD initialization failed!");
    sd.initErrorPrint(&Serial);
  } else {
    Serial.println("SD initialization OK!");
    
    // Print card info
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
  // Blink LED if possible? Nano RP2040 has an RGB LED but also a power LED.
  // The L LED is on D13 (SCK), so we can't use it without interfering with SPI.
  delay(1000);
}
