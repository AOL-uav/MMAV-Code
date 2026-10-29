/*
  GPS Basic Passthrough Test
  Board: Arduino Nano RP2040 Connect
  
  Connections:
  - GPS TX  ->  Nano Pin D0 (RX1)
  - GPS RX  ->  Nano Pin D1 (TX1)
  - GPS VCC ->  3.3V or 5V (Check your module spec)
  - GPS GND ->  GND
  
  Purpose: 
  This sketch simply listens to the GPS module on Serial1 and echoes 
  everything to the Serial Monitor (Serial). Use this to verify that 
  you are receiving raw NMEA sentences ($GPRMC, $GPGSV, etc.).
*/

void setup() {
  // USB Serial for Monitor
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial Monitor to open
  }
  
  // Hardware Serial for GPS
  // Most GPS modules default to 9600 baud
  Serial1.begin(9600);
  
  Serial.println("--- GPS Passthrough Started ---");
  Serial.println("Listening for NMEA sentences on Serial1 (Pins 0/1)...");
}

void loop() {
  // Read from GPS and send to PC
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.print(c);
  }

  // Allow sending commands back to GPS if needed
  if (Serial.available()) {
    char c = Serial.read();
    Serial1.print(c);
  }
}
