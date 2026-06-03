#include <SPI.h>
#include "SdFat.h"

#define SD_CS_PIN 5   
#define SD_MOSI_PIN 7 
#define SD_MISO_PIN 4 
#define SD_SCK_PIN 6  

SdFat sd;
bool success = false;

void setup() {
  Serial.begin(115200);
}

void loop() {
  static bool started = false;
  if (!started) {
    delay(10000);
    Serial.println("--- Loop Based SD Test ---");
    SPI.setRX(SD_MISO_PIN);
    SPI.setTX(SD_MOSI_PIN);
    SPI.setSCK(SD_SCK_PIN);
    SPI.begin();
    success = sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(4)));
    started = true;
  }
  
  if (success) {
    Serial.println("SD SUCCESS!");
  } else {
    Serial.print("SD FAIL. Error: ");
    sd.initErrorPrint(&Serial);
  }
  delay(2000);
}
