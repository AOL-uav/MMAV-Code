#include <Servo.h>

Servo servoL;
Servo servoR;

// FINAL CONFIRMED PINS
// D3 (GPIO 15) -> Moves servo on PCB Pin 1 (Right Servo)
// D4 (GPIO 16) -> Moves servo on other PCB pin (Left Servo)
const int pinL = D4; 
const int pinR = D3; 

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  
  // Attach servos
  servoL.attach(pinL);
  servoR.attach(pinR);
  
  Serial.println("Servo System Online (D3/D4)");
}

void loop() {
  // Pitch up
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Pitch Up");
  servoL.write(110);
  servoR.write(110);
  delay(3000);

  // Pitch down
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Pitch Down");
  servoL.write(70);
  servoR.write(70);
  delay(3000);

  // Neutral
  Serial.println("Neutral");
  servoL.write(90);
  servoR.write(90);
  delay(3000);
}
