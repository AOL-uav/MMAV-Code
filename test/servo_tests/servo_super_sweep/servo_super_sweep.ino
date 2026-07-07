#include <Servo.h>

Servo testServo;

// Nano RP2040 Connect (Earle Philhower core) Pin Mapping
int pins[] = {
  1, 0, 25, 15, 16, 17, 18, 19, // D0 - D7
  20, 21, 5, 7, 4, 6,           // D8 - D13
  26, 27, 28, 29                // A0 - A3
};

const char* labels[] = {
  "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7",
  "D8", "D9", "D10", "D11", "D12", "D13",
  "A0", "A1", "A2", "A3"
};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

// Rapid blinks for tens, slow blinks for units to make counting easier
void signalPin(int index) {
  int displayNum = index; // 0-indexed for counting
  
  // Long pulse to signal start of a new pin test
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  // Blink the number (1-based)
  for (int i = 0; i <= index; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
  }
}

void loop() {
  for (int i = 0; i < 18; i++) {
    int currentPin = pins[i];
    
    // 1. Signal which pin index we are testing
    signalPin(i);
    delay(500);
    
    // 2. Attach and sweep
    Serial.print("Testing: "); Serial.println(labels[i]);
    
    testServo.attach(currentPin);
    testServo.write(130);
    delay(800);
    testServo.write(50);
    delay(800);
    testServo.write(90);
    delay(800);
    testServo.detach();
    
    delay(1500);
  }
  
  // Long pause before restart
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(3000);
}
