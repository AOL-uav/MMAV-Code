#include <Servo.h>


static const int SWEEP_PIN     = 5;   // D5  - POSITIONAL sweep servo
static const int AOA_LEFT_PIN  = 4;   // D4  - left  wing AoA servo
static const int AOA_RIGHT_PIN = 3;   // D3  - right wing AoA servo
static const int PIN_MOUNT     = A0;  // mounting position (to GND)
static const int PIN_UNFOLD    = A1;  // unfolds wings (to GND)
static const int PIN_FOLD      = A2;  // folds wings (to GND)

// We set the absolute minimum 500 as the "mount" position so it goes as far as physically possible.
// We set 700 as the normal "folded" position so it stops a little shy of the absolute limit.
// You can tweak these 3 values until they line up exactly with your physical needs!
static const int SWEEP_UNFOLDED_US = 2200;  // wing OUT (flight) (was 2500)
static const int SWEEP_FOLDED_US   = 500;   // wing FOLDED/stowed (Normal fold)
static const int SWEEP_MOUNT_US    = 700;   // Offset position for mounting

static const int AOA_LEFT_FLAT_US  = 1575;
static const int AOA_RIGHT_FLAT_US = 1500;
// Rotational mode: 90 degrees mirrored
// Assuming ~1000us = 90 degrees.
static const int AOA_LEFT_ROTATIONAL_US  = 575;   // 1575 - 1000
static const int AOA_RIGHT_ROTATIONAL_US = 500;   // 1500 - 1000

Servo sweepServo;
Servo aoaLeft;
Servo aoaRight;

bool isAttached = false;
int currentSweepUs = -1; // Manually track position since mbed readMicroseconds() is buggy
int currentAoaLeftUs = -1;
int currentAoaRightUs = -1;

enum State { UNKNOWN, FOLDED, UNFOLDED, MOUNT };
State currentState = UNKNOWN;

void setup() {
  pinMode(PIN_MOUNT, INPUT_PULLUP);
  pinMode(PIN_UNFOLD, INPUT_PULLUP);
  pinMode(PIN_FOLD, INPUT_PULLUP);
  
  // Start in an unknown state
  currentState = UNKNOWN;
  
  // NOTE: We DO NOT attach() the servos here in setup().
}

// Helper to silently attach servos without jerking backward
void lazyAttach(int sweepTarget, int aoaLeftTarget, int aoaRightTarget) {
  if (!isAttached) {
     currentSweepUs = sweepTarget;
     currentAoaLeftUs = aoaLeftTarget;
     currentAoaRightUs = aoaRightTarget;
     
     sweepServo.writeMicroseconds(currentSweepUs);
     
     // Initialize AoA servos to match the target
     aoaLeft.writeMicroseconds(currentAoaLeftUs);
     aoaRight.writeMicroseconds(currentAoaRightUs);
     
     sweepServo.attach(SWEEP_PIN, 500, 2500);
     aoaLeft.attach(AOA_LEFT_PIN, 500, 2500);
     aoaRight.attach(AOA_RIGHT_PIN, 500, 2500);
     isAttached = true;
  }
}

// Helper to smoothly sweep the main wings over 1.5 seconds
void slowSweepTo(int targetUs) {
  if (currentSweepUs == -1) currentSweepUs = targetUs;
  
  if (currentSweepUs == targetUs) {
    delay(1000);
    return;
  }
  
  int diff = targetUs - currentSweepUs;
  int steps = 75; // 75 steps * 20ms = 1.5s
  float stepSize = (float)diff / steps;
  
  for (int i = 1; i <= steps; i++) {
    sweepServo.writeMicroseconds(currentSweepUs + (int)(stepSize * i));
    delay(20); 
  }
  sweepServo.writeMicroseconds(targetUs);
  currentSweepUs = targetUs;
}

// Helper to smoothly sweep the AoA servos over 1.5 seconds
void slowSweepAoaTo(int targetLeftUs, int targetRightUs) {
  if (currentAoaLeftUs == -1) currentAoaLeftUs = targetLeftUs;
  if (currentAoaRightUs == -1) currentAoaRightUs = targetRightUs;
  
  if (currentAoaLeftUs == targetLeftUs && currentAoaRightUs == targetRightUs) {
    return;
  }
  
  int diffLeft = targetLeftUs - currentAoaLeftUs;
  int diffRight = targetRightUs - currentAoaRightUs;
  int steps = 75; // 75 steps * 20ms = 1.5s
  float stepSizeLeft = (float)diffLeft / steps;
  float stepSizeRight = (float)diffRight / steps;
  
  for (int i = 1; i <= steps; i++) {
    aoaLeft.writeMicroseconds(currentAoaLeftUs + (int)(stepSizeLeft * i));
    aoaRight.writeMicroseconds(currentAoaRightUs + (int)(stepSizeRight * i));
    delay(20); 
  }
  aoaLeft.writeMicroseconds(targetLeftUs);
  aoaRight.writeMicroseconds(targetRightUs);
  currentAoaLeftUs = targetLeftUs;
  currentAoaRightUs = targetRightUs;
}

void loop() {
  // 1. Check if Fold is pressed (Trigger Rotational Mode)
  if (digitalRead(PIN_FOLD) == LOW && currentState != FOLDED) {
    lazyAttach(SWEEP_UNFOLDED_US, AOA_LEFT_FLAT_US, AOA_RIGHT_FLAT_US); 
    
    // First make sure the main wings are swept out before we rotate 90 degrees
    slowSweepTo(SWEEP_UNFOLDED_US);
    
    // Smoothly sweep AoA to 90 degrees mirrored
    slowSweepAoaTo(AOA_LEFT_ROTATIONAL_US, AOA_RIGHT_ROTATIONAL_US);
    
    currentState = FOLDED; // (Reusing the FOLDED state for rotational)
    
    // Wait for the fold signal to be removed so it doesn't instantly repeat!
    while (digitalRead(PIN_FOLD) == LOW) {
      delay(20);
    }
    
  // 2. Check if Unfold is pressed (Reset to Default Flat)
  } else if (digitalRead(PIN_UNFOLD) == LOW && currentState != UNFOLDED) {
    lazyAttach(SWEEP_UNFOLDED_US, AOA_LEFT_FLAT_US, AOA_RIGHT_FLAT_US); 
    
    // Smoothly sweep main wings out over 1.5 seconds (if they weren't already)
    slowSweepTo(SWEEP_UNFOLDED_US);
    
    // Smoothly flatten AoA once the wings are fully swept out
    slowSweepAoaTo(AOA_LEFT_FLAT_US, AOA_RIGHT_FLAT_US);
    
    currentState = UNFOLDED;
  }
  
  delay(20);
}
