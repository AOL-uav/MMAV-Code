#include <SPI.h>
#include "SdFat.h"

// Correct GPIO numbers for Nano RP2040 Connect
#define SD_CS_PIN 5   // D10
#define SD_MOSI_PIN 7 // D11
#define SD_MISO_PIN 4 // D12
#define SD_SCK_PIN 6  // D13

SdFat sd;

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("--- Final SdFat SD Card Test ---");
  
  Serial.println("Configuring SPI with CORRECT GPIOs...");
  SPI.setRX(SD_MISO_PIN);
  SPI.setTX(SD_MOSI_PIN);
  SPI.setSCK(SD_SCK_PIN);
  SPI.begin();
  
  Serial.println("Initializing SD card at 4MHz...");
  if (!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(4)))) {
    Serial.println("SD initialization failed!");
    sd.initErrorPrint(&Serial);
  } else {
    Serial.println("SD initialization SUCCESS!");
    
    uint32_t size = sd.card()->sectorCount();
    if (size == 0) {
      Serial.println("Can't determine the card size.");
    } else {
      uint32_t sizeMB = size / 2048;
      Serial.print("Card size: ");
      Serial.print(sizeMB);
      Serial.println(" MB");
    }
    
    Serial.println("Listing files:");
    sd.ls(&Serial, LS_R);
  }
}

void loop() {
  Serial.println("Heartbeat...");
  delay(5000);
}
