/*
  GPS TinyGPS++ Advanced Test
  Board: Arduino Nano RP2040 Connect
  
  Requires: TinyGPS++ Library
  
  Purpose: 
  Parses raw NMEA data into readable Latitude, Longitude, Altitude, 
  Speed, and Satellite metadata.
*/

#include <TinyGPS++.h>

// The TinyGPS++ object
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // GPS module on Serial1 (Pins 0/1)
  // Default is usually 9600, but some modules (like UBLOX) can be configured higher.
  Serial1.begin(9600);

  Serial.println(F("--- TinyGPS++ Advanced Parsing Test ---"));
  Serial.println(F("LAT, LON, ALT(m), SPEED(km/h), SATS, HDOP"));
}

void loop() {
  // Feed data to TinyGPS++
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      displayInfo();
    }
  }

  // If no data is received for 5 seconds, warn the user
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    delay(5000);
  }
}

void displayInfo() {
  // Location
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(", "));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("0.000000, 0.000000"));
  }

  // Altitude
  Serial.print(F(" | Alt: "));
  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters());
    Serial.print(F("m"));
  } else {
    Serial.print(F("INVALID"));
  }

  // Speed
  Serial.print(F(" | Spd: "));
  if (gps.speed.isValid()) {
    Serial.print(gps.speed.kmph());
    Serial.print(F(" km/h ("));
    Serial.print(gps.speed.knots());
    Serial.print(F(" knots)"));
  } else {
    Serial.print(F("INVALID"));
  }

  // Satellite Metadata (Fix Quality)
  Serial.print(F(" | Sats: "));
  if (gps.satellites.isValid()) {
    Serial.print(gps.satellites.value());
  } else {
    Serial.print(F("0"));
  }

  Serial.print(F(" | HDOP: "));
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.hdop());
  } else {
    Serial.print(F("99.9"));
  }

  // Time
  Serial.print(F(" | Time: "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
  }

  Serial.println();
}
