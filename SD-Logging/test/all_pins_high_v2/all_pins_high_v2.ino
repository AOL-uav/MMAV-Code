void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Driving all pins HIGH now...");
  for (int i = 0; i < 30; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }
  Serial.println("Done.");
}

void loop() {
  Serial.println("All pins should be HIGH.");
  delay(1000);
}
