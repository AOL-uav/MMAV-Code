void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("--- GPIO Mapping Check ---");
  Serial.print("D10 (CS) is GPIO: "); Serial.println(D10);
  Serial.print("D11 (MOSI) is GPIO: "); Serial.println(D11);
  Serial.print("D12 (MISO) is GPIO: "); Serial.println(D12);
  Serial.print("D13 (SCK) is GPIO: "); Serial.println(D13);
}

void loop() {
}
