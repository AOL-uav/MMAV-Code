#include <Servo.h>

// Pin assignments
const int yawPin = 0;
const int pitchPin = 1;

Servo yawServo;
Servo pitchServo;

// Mode State
bool autoMode = false;
bool relaxed = false;

// Motion Variables (in Microseconds)
float curYaw = 1500, curPitch = 1500;
float targetYaw = 1500, targetPitch = 1500;

// Higher Speed Smoothing (0.15 for faster response)
const float alpha = 0.15; 

// UPDATED RANGE LIMITS (Microseconds)
// Pin 0 (Yaw): Range 60 to 112 degrees (1500 +/- 250us approx)
const int minYawUS = 1167; 
const int maxYawUS = 1744; // Capped at 112 degrees

// Pin 1 (Pitch): +/- 60 degrees from center (1500 +/- 666us)
const int minPitchUS = 834; 
const int maxPitchUS = 2166;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Gimbal v6.2: HIGH SPEED & EXTENDED RANGE");
  
  attachAll();
  centerAll();
}

void attachAll() {
  if (!yawServo.attached()) yawServo.attach(yawPin, 500, 2500);
  if (!pitchServo.attached()) pitchServo.attach(pitchPin, 500, 2500);
  relaxed = false;
  Serial.println("STATUS:ALL_ATTACHED");
}

void detachAll() {
  yawServo.detach();
  pitchServo.detach();
  relaxed = true;
  autoMode = false;
  Serial.println("STATUS:RELAXED_ALL");
}

void centerAll() {
  autoMode = false;
  attachAll();
  targetYaw = 1500; targetPitch = 1500;
  Serial.println("STATUS:CENTERED_ALL");
}

int degToUs(float deg) {
  // Mapping 0-180 degrees to 500-2500us for 25kg servos
  return map(constrain(deg, 60, 120), 0, 180, 500, 2500);
}

void updateServos() {
  if (relaxed) return;
  
  curYaw   = (targetYaw * alpha)   + (curYaw * (1.0 - alpha));
  curPitch = (targetPitch * alpha) + (curPitch * (1.0 - alpha));

  yawServo.writeMicroseconds((int)constrain(curYaw, minYawUS, maxYawUS));
  pitchServo.writeMicroseconds((int)constrain(curPitch, minPitchUS, maxPitchUS));
}

void loop() {
  static String inputString = "";
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      processCommand(inputString);
      inputString = "";
    } else {
      inputString += inChar;
    }
  }

  if (!relaxed) {
    if (autoMode) {
      unsigned long now = millis();
      float phase = now / 1000.0;
      targetYaw   = 1500 + (300 * sin(phase * 0.4)); 
      targetPitch = 1500 + (300 * sin(phase * 0.7 + 1.0));
    }
    updateServos();
  }

  // Telemetry
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 250) {
    if (relaxed) {
      Serial.println("STATE:RELAXED");
    } else {
      Serial.print("POS:"); 
      Serial.print((int)curYaw); Serial.print(",");
      Serial.println((int)curPitch);
    }
    lastLog = millis();
  }

  delay(20); 
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char type = cmd.charAt(0);
  
  if (type == 'S' || type == 's') detachAll();
  else if (type == 'A' || type == 'a') { attachAll(); autoMode = true; }
  else if (type == 'C' || type == 'c') centerAll();
  else if (cmd.length() > 1) {
    int val = cmd.substring(1).toInt();
    float us = degToUs(val);
    autoMode = false; attachAll();
    if (type == 'Y' || type == 'y') targetYaw = us;
    else if (type == 'P' || type == 'p') targetPitch = us;
  }
}
