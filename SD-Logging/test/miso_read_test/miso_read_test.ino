void setup() {
  Serial.begin(115200);
  pinMode(12, INPUT); // D12 = MISO
}

void loop() {
  int val = digitalRead(12);
  Serial.print("MISO value: ");
  Serial.println(val);
  delay(500);
}
