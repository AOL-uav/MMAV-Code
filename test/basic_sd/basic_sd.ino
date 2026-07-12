#include <SPI.h>
#include "SdFat.h"

SdFs sd;
bool initSuccess = false;
bool tried = false;

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (!tried) {
    delay(3000);
    Serial.println("\n--- SD Card Test (SdFs) ---");
    SPI.setRX(4);
    SPI.setTX(7);
    SPI.setSCK(6);
    SPI.begin();
    pinMode(4, INPUT_PULLUP);

    if (!sd.begin(SdSpiConfig(5, SHARED_SPI, SD_SCK_MHZ(1)))) {
      Serial.println("Init failed!");
      sd.initErrorPrint(&Serial);
    } else {
      Serial.println("Init SUCCESS!");
      initSuccess = true;
      sd.ls(&Serial, LS_R);
      
      // Let's test writing
      FsFile file;
      if (file.open("test.txt", O_RDWR | O_CREAT | O_TRUNC)) {
        file.println("Testing SD card writing!");
        file.sync();
        file.close();
        Serial.println("File write successful!");
      } else {
        Serial.println("File write failed!");
      }
    }
    tried = true;
  }
  delay(1000);
}
