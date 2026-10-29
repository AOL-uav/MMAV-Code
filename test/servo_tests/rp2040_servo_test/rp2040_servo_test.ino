#include <107-Arduino-Servo-RP2040.h>

_107_::Servo servoL;
_107_::Servo servoR;

// Using raw GPIO numbers if D2/D3 don't work
// D2 is p25, D3 is p15
const int pinL = 25; 
const int pinR = 15;

void setup() {
  Serial.begin(115200);
  // No while(!Serial) so it runs even if monitor isn't connected
  delay(5000); 
  Serial.println("RP2040 Servo Test Start");
  
  if (servoL.attach(pinL)) Serial.println("L Attached");
  if (servoR.attach(pinR)) Serial.println("R Attached");
}

void loop() {
  Serial.println("Moving to 110 deg (1611us)");
  servoL.writeMicroseconds(1611); // ~110 deg
  servoR.writeMicroseconds(1611);
  delay(3000);

  Serial.println("Moving to 70 deg (1389us)");
  servoL.writeMicroseconds(1389); // ~70 deg
  servoR.writeMicroseconds(1389);
  delay(3000);
}
