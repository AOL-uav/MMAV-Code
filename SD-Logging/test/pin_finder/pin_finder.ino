void setup() {
  Serial.begin(115200);
  delay(5000);
}

void loop() {
  for (int i = 0; i < 30; i++) {
    Serial.print("Driving GPIO "); Serial.print(i); Serial.println(" HIGH...");
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
    delay(2000); // 2 seconds per pin
    digitalWrite(i, LOW);
    pinMode(i, INPUT);
  }
}
