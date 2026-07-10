#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  
  Serial.println("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("Initialization failed!");
    while (1);
  }
  Serial.println("Initialization done.");

  File myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    myFile.close();
    Serial.println("done.");
  } else {
    Serial.println("error opening test.txt");
  }
}

void loop() {
  delay(1000);
  Serial.println("Looping...");
}
