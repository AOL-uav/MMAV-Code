void setup() {
  Serial.begin(9600);
  
  // Wait up to 5 seconds for Serial Monitor to connect
  uint32_t start = millis();
  while (!Serial && millis() - start < 5000);
  
  Serial.println("\n\n--- HARDWARE DIAGNOSTIC START ---");
  
  // Test 1: Drive D4, read D5
  pinMode(4, OUTPUT);
  pinMode(5, INPUT_PULLDOWN); // Ensure it defaults to LOW
  delay(100);
  
  digitalWrite(4, HIGH);
  delay(50);
  int readD5_High = digitalRead(5);
  
  digitalWrite(4, LOW);
  delay(50);
  int readD5_Low = digitalRead(5);
  
  // Test 2: Drive D5, read D4
  pinMode(5, OUTPUT);
  pinMode(4, INPUT_PULLDOWN); // Ensure it defaults to LOW
  delay(100);
  
  digitalWrite(5, HIGH);
  delay(50);
  int readD4_High = digitalRead(4);
  
  digitalWrite(5, LOW);
  delay(50);
  int readD4_Low = digitalRead(4);
  
  // Put them back to safe inputs
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  
  Serial.println("Results:");
  Serial.print("When D4 is HIGH, D5 reads: "); Serial.println(readD5_High ? "HIGH" : "LOW");
  Serial.print("When D4 is LOW,  D5 reads: "); Serial.println(readD5_Low ? "HIGH" : "LOW");
  Serial.print("When D5 is HIGH, D4 reads: "); Serial.println(readD4_High ? "HIGH" : "LOW");
  Serial.print("When D5 is LOW,  D4 reads: "); Serial.println(readD4_Low ? "HIGH" : "LOW");
  
  Serial.println("\nConclusion:");
  if (readD5_High == HIGH && readD5_Low == LOW && readD4_High == HIGH && readD4_Low == LOW) {
    Serial.println("!!! WARNING: D4 AND D5 ARE PHYSICALLY SHORTED !!!");
    Serial.println("This means a solder bridge or broken trace on the PCB is connecting the two pins.");
    Serial.println("Any signal sent to D4 is physically bleeding over to D5, causing the Sweep servo to identically mimic the AoA servo!");
  } else {
    Serial.println("Pins D4 and D5 are electrically isolated (No short detected).");
  }
  
  Serial.println("--- DIAGNOSTIC END ---");
}

void loop() {
  delay(1000);
}
