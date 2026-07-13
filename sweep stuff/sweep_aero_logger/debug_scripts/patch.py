import re

with open('wing_sweep_esekf_log.ino', 'r') as f:
    content = f.read()

slow_sweep_func = '''
void lazyAttach(int sweepTarget, int aoaLeftTarget, int aoaRightTarget) {
  if (!servosAttached) {
     currentSweepUs = sweepTarget;
     servoMorph.writeMicroseconds(currentSweepUs);
     
     servoLeft.writeMicroseconds(aoaLeftTarget);
     servoRight.writeMicroseconds(aoaRightTarget);
     
     servoMorph.attach(SERVO_MORPH_PIN, 500, 2500);
     servoLeft.attach(SERVO_LEFT_PIN, 500, 2500);
     servoRight.attach(SERVO_RIGHT_PIN, 500, 2500);
     servosAttached = true;
  }
}

void slowSweepTo(int targetUs) {
  if (currentSweepUs == -1.0f) currentSweepUs = targetUs;
  
  if ((int)currentSweepUs == targetUs) {
    delay(1000);
    return;
  }
  
  int diff = targetUs - (int)currentSweepUs;
  int steps = 50; 
  float stepSize = (float)diff / steps;
  
  for (int i = 1; i <= steps; i++) {
    servoMorph.writeMicroseconds(currentSweepUs + (int)(stepSize * i));
    morphPwmUs = currentSweepUs + (int)(stepSize * i);
    leftPwmUs = (int)currentAoaLeftUs;
    rightPwmUs = (int)currentAoaRightUs;
    
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
  servoMorph.writeMicroseconds(targetUs);
  currentSweepUs = targetUs;
  morphPwmUs = targetUs;
}

void loop() {
'''
content = content.replace("void loop() {", slow_sweep_func)

new_loop_body = '''
  // Run ESEKF/logging for the current tick if not sweeping
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
      
      morphPwmUs = (int)currentSweepUs;
      leftPwmUs = (int)currentAoaLeftUs;
      rightPwmUs = (int)currentAoaRightUs;
      
      const uint32_t controlExecUs = micros() - controlStartUs;
      maybeEnqueueLog(controlDtUs, controlExecUs, esekfUs);
    }
  }

  // 1. Check if Fold is pressed
  if (digitalRead(PIN_FOLD) == LOW && sweepMode != SWEEP_FOLDED) {
    lazyAttach(SWEEP_FOLDED_US, AOA_LEFT_FOLDED_US, AOA_RIGHT_FOLDED_US); 
    servoLeft.writeMicroseconds(AOA_LEFT_FOLDED_US);
    servoRight.writeMicroseconds(AOA_RIGHT_FOLDED_US);
    currentAoaLeftUs = AOA_LEFT_FOLDED_US;
    currentAoaRightUs = AOA_RIGHT_FOLDED_US;
    
    slowSweepTo(SWEEP_FOLDED_US);
    sweepMode = SWEEP_FOLDED;
    
  // 2. Check if Unfold is pressed
  } else if (digitalRead(PIN_UNFOLD) == LOW && sweepMode != SWEEP_UNFOLDED) {
    lazyAttach(SWEEP_UNFOLDED_US, AOA_LEFT_FLAT_US, AOA_RIGHT_FLAT_US); 
    slowSweepTo(SWEEP_UNFOLDED_US);
    
    servoLeft.writeMicroseconds(AOA_LEFT_FLAT_US);
    servoRight.writeMicroseconds(AOA_RIGHT_FLAT_US);
    currentAoaLeftUs = AOA_LEFT_FLAT_US;
    currentAoaRightUs = AOA_RIGHT_FLAT_US;
    sweepMode = SWEEP_UNFOLDED;

  // 3. Check if Mount is pressed
  } else if (digitalRead(PIN_MOUNT) == LOW && sweepMode != SWEEP_MOUNT) {
    lazyAttach(SWEEP_MOUNT_US, AOA_LEFT_FOLDED_US, AOA_RIGHT_FOLDED_US); 
    servoLeft.writeMicroseconds(AOA_LEFT_FOLDED_US);
    servoRight.writeMicroseconds(AOA_RIGHT_FOLDED_US);
    currentAoaLeftUs = AOA_LEFT_FOLDED_US;
    currentAoaRightUs = AOA_RIGHT_FOLDED_US;
    
    slowSweepTo(SWEEP_MOUNT_US);
    sweepMode = SWEEP_MOUNT;
  }
}
'''
content = re.sub(r'void loop\(\) \{.*\}', new_loop_body, content, flags=re.DOTALL)

with open('wing_sweep_esekf_log.ino', 'w') as f:
    f.write(content)
