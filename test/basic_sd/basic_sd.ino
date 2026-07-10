#include <SPI.h>
#include "RP2040_SD.h"

#define PIN_SD_MOSI       PIN_SPI_MOSI
#define PIN_SD_MISO       PIN_SPI_MISO
#define PIN_SD_SCK        PIN_SPI_SCK

const int chipSelect = PIN_SPI_SS;

Sd2Card card;
SdVolume volume;
SdFile root;

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print("Initializing SD card with SS = ");  Serial.println(PIN_SPI_SS);
  Serial.print("SCK = ");   Serial.println(PIN_SPI_SCK);
  Serial.print("MOSI = ");  Serial.println(PIN_SPI_MOSI);
  Serial.print("MISO = ");  Serial.println(PIN_SPI_MISO);

  if (!card.init(SPI_HALF_SPEED, chipSelect)) 
  {
    Serial.println("initialization failed.");
  } 
  else 
  {
    Serial.println("Wiring is correct and a card is present.");
    if (!SD.begin(PIN_SPI_SS)) {
      Serial.println("SD.begin failed!");
    } else {
      Serial.println("SD.begin OK!");
    }
  }
  delay(2000);
}
