import re

with open('../sweep_aero_logger/sweep_aero_logger.ino', 'r') as f:
    content = f.read()

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
    
    // Wait for 40 seconds (2000 steps of 20ms) non-blocking for logger
    for (int i = 0; i < 2000; i++) {
        uint32_t stepStartMs = millis();
        while (millis() - stepStartMs < 20) {
            pollGps();
            const uint32_t nowUs2 = micros();
            if ((int32_t)(nowUs2 - nextControlUs) >= 0) {
                nextControlUs += CONTROL_DT_US;
                const uint32_t controlDtUs2 = nowUs2 - lastControlUs;
                lastControlUs = nowUs2;
                float dt2 = controlDtUs2 * 1.0e-6f;
                if (dt2 <= 0.0f || dt2 > 0.1f) dt2 = DT_NOMINAL;
                const uint32_t controlStartUs2 = micros();
                if (!imuFaultLocked && readImu(imu)) {
                    const uint32_t esekfStartUs2 = micros();
                    predictEsekf(imu, dt2);
                    updateEsekfWithAccelerometer(imu);
                    updateEsekfWithGps();
                    const uint32_t esekfUs2 = micros() - esekfStartUs2;
                    eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
                    const uint32_t controlExecUs2 = micros() - controlStartUs2;
                    maybeEnqueueLog(controlDtUs2, controlExecUs2, esekfUs2);
                }
            }
        }
    }
    
    // Automatically unfold after 40 seconds
    slowSweepTo(SWEEP_UNFOLDED_US);
    
    servoLeft.writeMicroseconds(AOA_LEFT_FLAT_US);
    servoRight.writeMicroseconds(AOA_RIGHT_FLAT_US);
    currentAoaLeftUs = AOA_LEFT_FLAT_US;
    currentAoaRightUs = AOA_RIGHT_FLAT_US;
    sweepMode = SWEEP_UNFOLDED;
    
    while(digitalRead(PIN_FOLD) == LOW) {
      delay(20);
    }
    
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

with open('sweep_auto_timer_logged.ino', 'w') as f:
    f.write(content)

print("Created sweep_auto_timer_logged.ino successfully!")
