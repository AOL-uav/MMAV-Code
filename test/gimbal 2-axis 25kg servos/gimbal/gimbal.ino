#include <Servo.h>

const int yawPin = 0;
const int pitchPin = 1;
Servo yawServo;
Servo pitchServo;

float curYaw = 1500, curPitch = 1500;
float targetYaw = 1500, targetPitch = 1500;

// Slew Rate: Max microseconds to move per 20ms frame
float maxStep = 10.0; 

const int minYawUS = 1167; 
const int maxYawUS = 1744; 
const int minPitchUS = 834; 
const int maxPitchUS = 2166;

bool relaxed = false;
bool autoMode = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Gimbal v6.5: SLEW-RATE & SILENT MODE");
  attachAll();
}

void attachAll() {
  if (!yawServo.attached()) yawServo.attach(yawPin, 500, 2500);
  if (!pitchServo.attached()) pitchServo.attach(pitchPin, 500, 2500);
  relaxed = false;
}

void detachAll() {
  yawServo.detach(); pitchServo.detach();
  relaxed = true; autoMode = false;
}

void loop() {
  static String inputString = "";
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  // 1. Check Serial
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      processCommand(inputString);
      inputString = "";
    } else {
      inputString += inChar;
    }
  }

  // 2. Linear Slew Update (50Hz / 20ms)
  if (now - lastUpdate >= 20) {
    lastUpdate = now;
    if (!relaxed) {
      if (autoMode) {
        float phase = now / 1000.0;
        targetYaw = 1500 + (250 * sin(phase * 0.4));
        targetPitch = 1500 + (500 * sin(phase * 0.7));
      }
      updateServos();
    }
  }
}

void updateServos() {
  bool moving = false;

  // Linear Slew for Yaw
  if (abs(targetYaw - curYaw) > maxStep) {
    curYaw += (targetYaw > curYaw) ? maxStep : -maxStep;
    moving = true;
  } else {
    curYaw = targetYaw;
  }

  // Linear Slew for Pitch
  if (abs(targetPitch - curPitch) > maxStep) {
    curPitch += (targetPitch > curPitch) ? maxStep : -maxStep;
    moving = true;
  } else {
    curPitch = targetPitch;
  }

  // Apply to hardware
  yawServo.writeMicroseconds((int)constrain(curYaw, minYawUS, maxYawUS));
  pitchServo.writeMicroseconds((int)constrain(curPitch, minPitchUS, maxPitchUS));

  // 3. SILENT MODE: Only print if we have STOPPED moving
  // This prevents Serial interrupts from jittering the PWM signal during motion
  static bool wasMoving = false;
  if (wasMoving && !moving) {
    Serial.print("STOPPED:"); Serial.print((int)curYaw);
    Serial.print(","); Serial.println((int)curPitch);
  }
  wasMoving = moving;
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char type = cmd.charAt(0);
  
  if (type == 'S' || type == 's') detachAll();
  else if (type == 'A' || type == 'a') { attachAll(); autoMode = true; }
  else if (type == 'C' || type == 'c') { 
    attachAll(); autoMode = false; 
    targetYaw = 1500; targetPitch = 1500; 
  }
  else if (cmd.length() > 1) {
    int val = cmd.substring(1).toInt();
    if (type == 'V' || type == 'v') {
      maxStep = constrain(val / 2.0, 1.0, 50.0); // Map UI slider to Slew Rate
      return;
    }
    if (relaxed) attachAll();
    autoMode = false;
    float us = map(constrain(val, 0, 180), 0, 180, 500, 2500);
    if (type == 'Y' || type == 'y') targetYaw = us;
    else if (type == 'P' || type == 'p') targetPitch = us;
  }
}
