void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("--- SPI Pin Torture Test ---");
  Serial.println("Toggling pins one by one to find the killer.");
}

void loop() {
  Serial.println("Testing D11 (MOSI) - GPIO 7...");
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH); delay(100); digitalWrite(7, LOW); delay(100);
  Serial.println("D11 OK.");

  Serial.println("Testing D13 (SCK) - GPIO 6...");
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH); delay(100); digitalWrite(6, LOW); delay(100);
  Serial.println("D13 OK.");

  Serial.println("Testing D12 (MISO) - GPIO 4...");
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH); delay(100); digitalWrite(4, LOW); delay(100);
  Serial.println("D12 OK.");

  Serial.println("Testing D10 (CS) - GPIO 5...");
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH); delay(100); digitalWrite(5, LOW); delay(100);
  Serial.println("D10 OK.");

  delay(2000);
}
