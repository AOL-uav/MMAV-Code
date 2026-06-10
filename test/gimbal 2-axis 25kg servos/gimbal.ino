#include <Servo.h>

// Pin assignments
const int yawPin = 0;
const int pitchPin = 1;
const int rollPin = 2;

Servo yawServo;
Servo pitchServo;
Servo rollServo;

// Mode State
bool autoMode = false;
bool relaxed = false;

// Motion Variables (in Microseconds)
float curYaw = 1500, curPitch = 1500, curRoll = 1500;
float targetYaw = 1500, targetPitch = 1500, targetRoll = 1500;

// Conservative Smoothing Factor (Alpha)
const float alpha = 0.08; 

// STALL PREVENTION: Restricted range (approx 60-120 degrees)
// Updated for 25kg servos (500-2500us range)
const int minSafeUS = 1150; 
const int maxSafeUS = 1850;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Gimbal v6.0: 25kg Servo Upgrade");

  attachAll();
  centerAll();
}

void attachAll() {
  if (!yawServo.attached()) yawServo.attach(yawPin, 500, 2500);
  if (!pitchServo.attached()) pitchServo.attach(pitchPin, 500, 2500);
  if (!rollServo.attached()) rollServo.attach(rollPin, 500, 2500);
  relaxed = false;
  Serial.println("STATUS:ALL_ATTACHED");
}

void detachAll() {
  yawServo.detach();
  pitchServo.detach();
  rollServo.detach();
  relaxed = true;
  autoMode = false;
  Serial.println("STATUS:RELAXED_ALL");
}

void centerAll() {
  autoMode = false;
  attachAll();
  targetYaw = 1500; targetPitch = 1500; targetRoll = 1500;
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
  curRoll  = (targetRoll * alpha)  + (curRoll * (1.0 - alpha));

  yawServo.writeMicroseconds((int)constrain(curYaw, minSafeUS, maxSafeUS));
  pitchServo.writeMicroseconds((int)constrain(curPitch, minSafeUS, maxSafeUS));
  rollServo.writeMicroseconds((int)constrain(curRoll, minSafeUS, maxSafeUS));
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
      targetRoll  = 1500 + (250 * cos(phase * 1.2));
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
      Serial.print((int)curPitch); Serial.print(",");
      Serial.println((int)curRoll);
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
    else if (type == 'R' || type == 'r') targetRoll = us;
  }
}
