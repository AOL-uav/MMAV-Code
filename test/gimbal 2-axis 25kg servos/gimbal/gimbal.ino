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

// Smoothing Factor (Alpha)
float alpha = 0.15; 

// RANGE LIMITS (Microseconds)
const int minYawUS = 1167; 
const int maxYawUS = 1744; 
const int minPitchUS = 834; 
const int maxPitchUS = 2166;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Gimbal v6.4: ANTI-VIBRATION EDITION");
  attachAll();
  centerAll();
}

void attachAll() {
  if (!yawServo.attached()) yawServo.attach(yawPin, 500, 2500);
  if (!pitchServo.attached()) pitchServo.attach(pitchPin, 500, 2500);
  relaxed = false;
}

void detachAll() {
  yawServo.detach();
  pitchServo.detach();
  relaxed = true;
  autoMode = false;
}

void centerAll() {
  autoMode = false;
  attachAll();
  targetYaw = 1500; targetPitch = 1500;
}

int degToUs(float deg) {
  return map(constrain(deg, 0, 180), 0, 180, 500, 2500);
}

void updateServos() {
  if (relaxed) return;
  
  float oldYaw = curYaw;
  float oldPitch = curPitch;

  // 1. Move towards target
  curYaw   = (targetYaw * alpha)   + (curYaw * (1.0 - alpha));
  curPitch = (targetPitch * alpha) + (curPitch * (1.0 - alpha));

  // 2. ANTI-VIBRATION LOGIC: 
  // If we are very close to the target, "snap" to it and stop updating
  if (abs(curYaw - targetYaw) < 2.0) curYaw = targetYaw;
  if (abs(curPitch - targetPitch) < 2.0) curPitch = targetPitch;

  // 3. Only write if the position has changed significantly (> 1us)
  if (abs(curYaw - oldYaw) >= 1.0) {
    yawServo.writeMicroseconds((int)constrain(curYaw, minYawUS, maxYawUS));
  }
  if (abs(curPitch - oldPitch) >= 1.0) {
    pitchServo.writeMicroseconds((int)constrain(curPitch, minPitchUS, maxPitchUS));
  }
}

void loop() {
  static String inputString = "";
  static unsigned long lastUpdate = 0;
  
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      processCommand(inputString);
      inputString = "";
    } else {
      inputString += inChar;
    }
  }

  unsigned long now = millis();
  if (now - lastUpdate >= 10) {
    lastUpdate = now;
    if (!relaxed) {
      if (autoMode) {
        float phase = now / 1000.0;
        targetYaw   = 1500 + (250 * sin(phase * 0.4)); 
        targetPitch = 1500 + (500 * sin(phase * 0.7 + 1.0));
      }
      updateServos();
    }
  }

  static unsigned long lastLog = 0;
  if (now - lastLog > 500) { // Reduced telemetry rate to lower noise
    if (!relaxed) {
      Serial.print("POS:"); Serial.print((int)curYaw); 
      Serial.print(","); Serial.println((int)curPitch);
    }
    lastLog = now;
  }
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
      alpha = constrain(val / 100.0, 0.01, 0.5);
      return;
    }
    if (relaxed) attachAll();
    autoMode = false;
    float us = degToUs(val);
    if (type == 'Y' || type == 'y') targetYaw = (int)us;
    else if (type == 'P' || type == 'p') targetPitch = (int)us;
  }
}
