#include <Servo.h>

const int yawPin = 0;
const int pitchPin = 1;
Servo yawServo;
Servo pitchServo;

float curYaw = 1500, curPitch = 1500;
float targetYaw = 1500, targetPitch = 1500;

// Slew Rate: Max microseconds to move per 20ms frame
float maxStep = 10.0; 

// UNIFIED HARDWARE LIMITS
const int minYawUS = 1167; // 60 deg
const int maxYawUS = 1744; // 112 deg (Mechanical Limit)
const int minPitchUS = 1000; // 45 deg
const int maxPitchUS = 2000; // 135 deg (Safety Reduction)

bool relaxed = false;
bool autoMode = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Gimbal v6.6: TIGHTENED LIMITS");
  attachAll();
  centerAll();
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

void centerAll() {
  autoMode = false;
  attachAll();
  targetYaw = 1500; targetPitch = 1500;
}

void loop() {
  static String inputString = "";
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      processCommand(inputString);
      inputString = "";
    } else {
      inputString += inChar;
    }
  }

  if (now - lastUpdate >= 20) {
    lastUpdate = now;
    if (!relaxed) {
      if (autoMode) {
        float phase = now / 1000.0;
        targetYaw = 1500 + (250 * sin(phase * 0.4));
        targetPitch = 1500 + (400 * sin(phase * 0.7));
      }
      updateServos();
    }
  }
}

void updateServos() {
  bool moving = false;

  // Constrain targets to hardware limits before moving
  float safeTargetYaw = constrain(targetYaw, minYawUS, maxYawUS);
  float safeTargetPitch = constrain(targetPitch, minPitchUS, maxPitchUS);

  if (abs(safeTargetYaw - curYaw) > maxStep) {
    curYaw += (safeTargetYaw > curYaw) ? maxStep : -maxStep;
    moving = true;
  } else { curYaw = safeTargetYaw; }

  if (abs(safeTargetPitch - curPitch) > maxStep) {
    curPitch += (safeTargetPitch > curPitch) ? maxStep : -maxStep;
    moving = true;
  } else { curPitch = safeTargetPitch; }

  yawServo.writeMicroseconds((int)curYaw);
  pitchServo.writeMicroseconds((int)curPitch);

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
  else if (type == 'C' || type == 'c') centerAll();
  else if (cmd.length() > 1) {
    int val = cmd.substring(1).toInt();
    if (type == 'V' || type == 'v') {
      maxStep = constrain(val / 2.0, 1.0, 50.0);
      return;
    }
    if (relaxed) attachAll();
    autoMode = false;
    // Map 0-180 UI to 500-2500 range
    float us = map(constrain(val, 0, 180), 0, 180, 500, 2500);
    if (type == 'Y' || type == 'y') targetYaw = us;
    else if (type == 'P' || type == 'p') targetPitch = us;
  }
}
