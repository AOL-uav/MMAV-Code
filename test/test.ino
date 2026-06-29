void setup() {
  Serial.begin(115200);
  Serial1.begin(9600); // Try 9600 baud first
  
  while (!Serial) {
    delay(10);
  }
  
  Serial.println(F("Raw GPS Echo Test Started"));
  Serial.println(F("Waiting for NMEA sentences..."));
}

void loop() {
  // Echo everything from GPS to the Serial Monitor
  while (Serial1.available() > 0) {
    Serial.write(Serial1.read());
  }
  
  // Also print a heartbeat every 5 seconds so we know the board isn't frozen
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 5000) {
    Serial.println(F("[Heartbeat] Still waiting..."));
    lastPrint = millis();
  }
}
