#include <Servo.h>

Servo testServo;

// Pins to test (labels on the board)
// In rp2040:rp2040 core, these map to:
// D0 = 1, D1 = 0, D2 = 25, D3 = 15
int pins[] = {1, 0, 25, 15};
const char* labels[] = {"D0", "D1", "D2", "D3"};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void blink(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

void loop() {
  for (int i = 0; i < 4; i++) {
    int currentPin = pins[i];
    
    // 1. Signal which pin we are testing by blinking the LED 'i+1' times
    blink(i + 1);
    delay(1000);
    
    // 2. Attach and move
    Serial.print("Testing Label: "); Serial.print(labels[i]);
    Serial.print(" (GPIO: "); Serial.print(currentPin); Serial.println(")");
    
    testServo.attach(currentPin);
    
    // Move back and forth
    testServo.write(120);
    delay(1000);
    testServo.write(60);
    delay(1000);
    testServo.write(90);
    delay(1000);
    
    testServo.detach();
    delay(2000); // Wait before next pin
  }
  
  // Long pause before restarting the whole sweep
  delay(5000);
}
