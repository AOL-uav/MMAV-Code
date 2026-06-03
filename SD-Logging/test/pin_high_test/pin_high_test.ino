void setup() {
  pinMode(10, OUTPUT); digitalWrite(10, HIGH);
  pinMode(11, OUTPUT); digitalWrite(11, HIGH);
  pinMode(12, OUTPUT); digitalWrite(12, HIGH);
  pinMode(13, OUTPUT); digitalWrite(13, HIGH);
  
  Serial.begin(115200);
}

void loop() {
  Serial.println("All SPI pins HIGH. Measure them!");
  delay(1000);
}
