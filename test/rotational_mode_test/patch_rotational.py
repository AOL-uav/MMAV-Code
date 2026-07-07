import os

path = r"C:\Users\Kai\OneDrive - purdue.edu\Purdue\AOL\MAV-2026\github stuff\sweep stuff\sweep_rotational_mode\sweep_rotational_mode.ino"

with open(path, 'r') as f:
    content = f.read()

# 1. Replace constants (around line 79)
old_consts = """static const int AOA_LEFT_FLAT_US  = 1575;
static const int AOA_RIGHT_FLAT_US = 1500;
static const int AOA_LEFT_FOLDED_US  = 1125;
static const int AOA_RIGHT_FOLDED_US = 1050;"""

new_consts = """static const int AOA_LEFT_FLAT_US  = 1575;
static const int AOA_RIGHT_FLAT_US = 1500;

// Rotational mode: 90 degrees mirrored
static const int AOA_LEFT_ROTATIONAL_US  = 575;   // 1575 - 1000
static const int AOA_RIGHT_ROTATIONAL_US = 500;   // 1500 - 1000"""

content = content.replace(old_consts, new_consts)

# 2. Add currentAoaLeftUs and currentAoaRightUs variables around line 133 (currentSweepUs)
content = content.replace("float currentSweepUs = -1.0f;", "float currentSweepUs = -1.0f;\nfloat currentAoaLeftUs = -1.0f;\nfloat currentAoaRightUs = -1.0f;")

# 3. Update lazyAttach (around line 1880)
old_lazyAttach = """void lazyAttach(int sweepTarget, int aoaLeftTarget, int aoaRightTarget) {
  if (!servosAttached) {
     currentSweepUs = sweepTarget;
     servoMorph.writeMicroseconds(currentSweepUs);
     
     servoLeft.writeMicroseconds(aoaLeftTarget);
     servoRight.writeMicroseconds(aoaRightTarget);"""

new_lazyAttach = """void lazyAttach(int sweepTarget, int aoaLeftTarget, int aoaRightTarget) {
  if (!servosAttached) {
     currentSweepUs = sweepTarget;
     currentAoaLeftUs = aoaLeftTarget;
     currentAoaRightUs = aoaRightTarget;
     
     servoMorph.writeMicroseconds(currentSweepUs);
     
     servoLeft.writeMicroseconds(currentAoaLeftUs);
     servoRight.writeMicroseconds(currentAoaRightUs);"""

content = content.replace(old_lazyAttach, new_lazyAttach)

# 4. Update slowSweepTo to be 1.5s (steps = 75)
content = content.replace("int steps = 25; \n  float stepSize = (float)diff / steps;", "int steps = 75; // 75 steps * 20ms = 1.5s\n  float stepSize = (float)diff / steps;")
# Or just replace exactly:
content = content.replace("int steps = 25; ", "int steps = 75; ")

# 5. Add slowSweepAoaTo function after slowSweepTo
slowSweepAoaTo_func = """
void slowSweepAoaTo(int targetLeftUs, int targetRightUs) {
  if (currentAoaLeftUs == -1.0f) currentAoaLeftUs = targetLeftUs;
  if (currentAoaRightUs == -1.0f) currentAoaRightUs = targetRightUs;
  
  if ((int)currentAoaLeftUs == targetLeftUs && (int)currentAoaRightUs == targetRightUs) {
    return;
  }
  
  int diffLeft = targetLeftUs - (int)currentAoaLeftUs;
  int diffRight = targetRightUs - (int)currentAoaRightUs;
  int steps = 75; // 1.5s
  float stepSizeLeft = (float)diffLeft / steps;
  float stepSizeRight = (float)diffRight / steps;
  
  for (int i = 1; i <= steps; i++) {
    servoLeft.writeMicroseconds(currentAoaLeftUs + (int)(stepSizeLeft * i));
    servoRight.writeMicroseconds(currentAoaRightUs + (int)(stepSizeRight * i));
    
    morphPwmUs = (int)currentSweepUs;
    leftPwmUs = currentAoaLeftUs + (int)(stepSizeLeft * i);
    rightPwmUs = currentAoaRightUs + (int)(stepSizeRight * i);
    
    uint32_t stepStartMs = millis();
    while (millis() - stepStartMs < 20) {
      pollGps();
      const uint32_t nowUs = micros();
      if ((int32_t)(nowUs - nextControlUs) >= 0) {
        nextControlUs += CONTROL_DT_US;
        const uint32_t controlDtUs = nowUs - lastControlUs;
        lastControlUs = nowUs;
        float dt = controlDtUs * 1.0e-6f;
        if (dt <= 0.0f || dt > 0.1f) dt = DT_NOMINAL;
        const uint32_t controlStartUs = micros();
        if (!imuFaultLocked && readImu(imu)) {
          const uint32_t esekfStartUs = micros();
          predictEsekf(imu, dt);
          updateEsekfWithAccelerometer(imu);
          updateEsekfWithGps();
          const uint32_t esekfUs = micros() - esekfStartUs;
          eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
          const uint32_t controlExecUs = micros() - controlStartUs;
          maybeEnqueueLog(controlDtUs, controlExecUs, esekfUs);
        }
      }
    }
  }
  servoLeft.writeMicroseconds(targetLeftUs);
  servoRight.writeMicroseconds(targetRightUs);
  currentAoaLeftUs = targetLeftUs;
  currentAoaRightUs = targetRightUs;
  leftPwmUs = targetLeftUs;
  rightPwmUs = targetRightUs;
}
"""

content = content.replace("void loop() {", slowSweepAoaTo_func + "\nvoid loop() {")

# 6. Update loop() logic for FOLD and UNFOLD (and remove MOUNT)
old_loop_logic = """  // 1. Check if Fold is pressed
  if (digitalRead(PIN_FOLD) == LOW && sweepMode != SWEEP_FOLDED) {
    lazyAttach(SWEEP_FOLDED_US, AOA_LEFT_FOLDED_US, AOA_RIGHT_FOLDED_US); 
    servoLeft.writeMicroseconds(AOA_LEFT_FOLDED_US);
    servoRight.writeMicroseconds(AOA_RIGHT_FOLDED_US);
    
    slowSweepTo(SWEEP_FOLDED_US);
    sweepMode = SWEEP_FOLDED;
    
  // 2. Check if Unfold is pressed
  } else if (digitalRead(PIN_UNFOLD) == LOW && sweepMode != SWEEP_UNFOLDED) {
    lazyAttach(SWEEP_UNFOLDED_US, AOA_LEFT_FLAT_US, AOA_RIGHT_FLAT_US); 
    slowSweepTo(SWEEP_UNFOLDED_US);
    
    servoLeft.writeMicroseconds(AOA_LEFT_FLAT_US);
    servoRight.writeMicroseconds(AOA_RIGHT_FLAT_US);
    sweepMode = SWEEP_UNFOLDED;

  // 3. Check if Mount is pressed
  } else if (digitalRead(PIN_MOUNT) == LOW && sweepMode != SWEEP_MOUNT) {
    lazyAttach(SWEEP_MOUNT_US, AOA_LEFT_FOLDED_US, AOA_RIGHT_FOLDED_US); 
    servoLeft.writeMicroseconds(AOA_LEFT_FOLDED_US);
    servoRight.writeMicroseconds(AOA_RIGHT_FOLDED_US);
    
    slowSweepTo(SWEEP_MOUNT_US);
    sweepMode = SWEEP_MOUNT;
  }
}"""

new_loop_logic = """  // 1. Check if Fold is pressed (Trigger Rotational Mode)
  if (digitalRead(PIN_FOLD) == LOW && sweepMode != SWEEP_FOLDED) {
    lazyAttach(SWEEP_UNFOLDED_US, AOA_LEFT_FLAT_US, AOA_RIGHT_FLAT_US); 
    slowSweepTo(SWEEP_UNFOLDED_US);
    slowSweepAoaTo(AOA_LEFT_ROTATIONAL_US, AOA_RIGHT_ROTATIONAL_US);
    sweepMode = SWEEP_FOLDED; // Reusing state for rotational
    while(digitalRead(PIN_FOLD) == LOW) delay(20);
    
  // 2. Check if Unfold is pressed (Reset to Default Flat)
  } else if (digitalRead(PIN_UNFOLD) == LOW && sweepMode != SWEEP_UNFOLDED) {
    lazyAttach(SWEEP_UNFOLDED_US, AOA_LEFT_FLAT_US, AOA_RIGHT_FLAT_US); 
    slowSweepTo(SWEEP_UNFOLDED_US);
    slowSweepAoaTo(AOA_LEFT_FLAT_US, AOA_RIGHT_FLAT_US);
    sweepMode = SWEEP_UNFOLDED;
  }
}"""

# Wait, we need to be careful with exact match of old_loop_logic. Let's do it via regex or just replace the whole loop function.
content = content[:content.find("  // 1. Check if Fold is pressed")] + new_loop_logic

with open(path, 'w') as f:
    f.write(content)
print("Done")
