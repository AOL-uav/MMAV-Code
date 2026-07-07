#include <Servo.h>


static const int SWEEP_PIN     = 6;   // D5  - POSITIONAL sweep servo
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
static const int AOA_LEFT_FOLDED_US  = 1125;  // ~45 deg offset (1575 - 450)
static const int AOA_RIGHT_FOLDED_US = 1050;  // reversed offset (1500 - 450)

Servo sweepServo;
Servo aoaLeft;
Servo aoaRight;

bool isAttached = false;
int currentSweepUs = -1; // Manually track position since mbed readMicroseconds() is buggy

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
     sweepServo.writeMicroseconds(currentSweepUs);
     
     // Initialize AoA servos to match the target
     aoaLeft.writeMicroseconds(aoaLeftTarget);
     aoaRight.writeMicroseconds(aoaRightTarget);
     
     sweepServo.attach(SWEEP_PIN, 500, 2500);
     aoaLeft.attach(AOA_LEFT_PIN, 500, 2500);
     aoaRight.attach(AOA_RIGHT_PIN, 500, 2500);
     isAttached = true;
  }
}

// Helper to smoothly sweep the main wings over 1 second
void slowSweepTo(int targetUs) {
  if (currentSweepUs == -1) currentSweepUs = targetUs;
  
  if (currentSweepUs == targetUs) {
    delay(1000);
    return;
  }
  
  int diff = targetUs - currentSweepUs;
  int steps = 50; 
  float stepSize = (float)diff / steps;
  
  for (int i = 1; i <= steps; i++) {
    sweepServo.writeMicroseconds(currentSweepUs + (int)(stepSize * i));
    delay(20); // 50 steps * 20ms = 1000ms = 1s sweep time
  }
  sweepServo.writeMicroseconds(targetUs);
  currentSweepUs = targetUs;
}

void loop() {
  // 1. Check if Fold is pressed
  if (digitalRead(PIN_FOLD) == LOW && currentState != FOLDED) {
    lazyAttach(SWEEP_FOLDED_US, AOA_LEFT_FOLDED_US, AOA_RIGHT_FOLDED_US); 
    
    // Instantly fold AoA to clear the path
    aoaLeft.writeMicroseconds(AOA_LEFT_FOLDED_US);
    aoaRight.writeMicroseconds(AOA_RIGHT_FOLDED_US);
    
    // Smoothly sweep main wings in over 1 second
    slowSweepTo(SWEEP_FOLDED_US);
    
    // Wait for 40 seconds
    for (int i = 0; i < 40; i++) {
      delay(1000);
    }
    
    // Automatically unfold after 40 seconds
    slowSweepTo(SWEEP_UNFOLDED_US);
    
    // Instantly flatten AoA once the wings are fully swept out
    aoaLeft.writeMicroseconds(AOA_LEFT_FLAT_US);
    aoaRight.writeMicroseconds(AOA_RIGHT_FLAT_US);
    
    currentState = UNFOLDED;
    
    // Wait for the fold signal to be removed so it doesn't instantly repeat!
    while (digitalRead(PIN_FOLD) == LOW) {
      delay(20);
    }
    
  // 2. Check if Unfold is pressed
  } else if (digitalRead(PIN_UNFOLD) == LOW && currentState != UNFOLDED) {
    lazyAttach(SWEEP_UNFOLDED_US, AOA_LEFT_FLAT_US, AOA_RIGHT_FLAT_US); 
    
    // Smoothly sweep main wings out over 1 second
    slowSweepTo(SWEEP_UNFOLDED_US);
    
    // Instantly flatten AoA once the wings are fully swept out
    aoaLeft.writeMicroseconds(AOA_LEFT_FLAT_US);
    aoaRight.writeMicroseconds(AOA_RIGHT_FLAT_US);
    
    currentState = UNFOLDED;

  // 3. Check if Mount is pressed
  } else if (digitalRead(PIN_MOUNT) == LOW && currentState != MOUNT) {
    lazyAttach(SWEEP_MOUNT_US, AOA_LEFT_FOLDED_US, AOA_RIGHT_FOLDED_US); 
    
    // Instantly fold AoA to clear the path
    aoaLeft.writeMicroseconds(AOA_LEFT_FOLDED_US);
    aoaRight.writeMicroseconds(AOA_RIGHT_FOLDED_US);
    
    // Smoothly sweep main wings to mount position over 1 second
    slowSweepTo(SWEEP_MOUNT_US);
    
    currentState = MOUNT;
  }
  
  delay(20);
}
