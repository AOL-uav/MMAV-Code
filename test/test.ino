#include <TinyGPSPlus.h>

// The TinyGPSPlus object
TinyGPSPlus gps;

// The GPS is connected to Serial1
// Connect GPS TX to Nano RP2040 RX (Pin 0)
// Connect GPS RX to Nano RP2040 TX (Pin 1)
// Connect GPS VCC to 3.3V or 5V (depending on your module)
// Connect GPS GND to GND

void setup() {
  Serial.begin(115200);
  
  // We found out the GPS module communicates at 115200 baud!
  Serial1.begin(115200); 
  
  // Wait for serial monitor to open (optional, but good for debugging)
  while (!Serial) {
    delay(10);
  }
  
  Serial.println(F("Nano RP2040 GPS Test"));
  Serial.println(F("Waiting for GPS data..."));
}

void loop() {
  // Read incoming characters from the GPS
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // Print location when it is updated
  if (gps.location.isUpdated()) {
    Serial.print(F("Location: "));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(", "));
    Serial.println(gps.location.lng(), 6);
    
    Serial.print(F("Satellites: "));
    Serial.println(gps.satellites.value());
    
    Serial.print(F("Altitude: "));
    Serial.print(gps.altitude.meters());
    Serial.println(F(" m"));
    Serial.println();
  }
  
  // Warning if no data is coming in
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring!"));
    delay(2000);
  }
}
