void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30; i++) {
    pinMode(i, INPUT_PULLUP);
  }
}

void loop() {
  Serial.print("GPIO 4 (D12): "); Serial.println(digitalRead(4));
  Serial.print("GPIO 7 (D11): "); Serial.println(digitalRead(7));
  Serial.print("GPIO 6 (D13): "); Serial.println(digitalRead(6));
  Serial.print("GPIO 5 (D10): "); Serial.println(digitalRead(5));
  delay(1000);
}
