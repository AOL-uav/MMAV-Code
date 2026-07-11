import re

with open('wing_sweep_esekf_log.ino', 'r') as f:
    content = f.read()

constants = '''
// ========================= Sweep Constants =========================
static const int PIN_MOUNT     = A0;
static const int PIN_UNFOLD    = A1;
static const int PIN_FOLD      = A2;

static const int SWEEP_UNFOLDED_US = 2500;
static const int SWEEP_FOLDED_US   = 500;
static const int SWEEP_MOUNT_US    = 700;

static const int AOA_LEFT_FLAT_US  = 1575;
static const int AOA_RIGHT_FLAT_US = 1500;
static const int AOA_LEFT_FOLDED_US  = 1125;
static const int AOA_RIGHT_FOLDED_US = 1050;

enum SweepMode { SWEEP_UNKNOWN, SWEEP_FOLDED, SWEEP_UNFOLDED, SWEEP_MOUNT };
static SweepMode sweepMode = SWEEP_UNKNOWN;
static bool servosAttached = false;
static float currentSweepUs = -1.0f;
static float currentAoaLeftUs = -1.0f;
static float currentAoaRightUs = -1.0f;
static int targetSweepUs = -1;
'''
content = content.replace('// ========================= Timing and IO =========================', constants + '\n// ========================= Timing and IO =========================')

setup_code = '''  pinMode(PIN_MOUNT, INPUT_PULLUP);
  pinMode(PIN_UNFOLD, INPUT_PULLUP);
  pinMode(PIN_FOLD, INPUT_PULLUP);
  
  sweepMode = SWEEP_UNKNOWN;
'''
content = content.replace('  pinMode(ARM_PIN, INPUT_PULLUP);', setup_code)

new_loop_logic = '''
  // Run ESEKF for logging
  if (esekfHealthy()) {
    eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
  } else {
    resetEsekfFromAccel(imu);
    eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
  }

  // Read sweep buttons
  if (digitalRead(PIN_FOLD) == LOW && sweepMode != SWEEP_FOLDED) {
    targetSweepUs = SWEEP_FOLDED_US;
    currentAoaLeftUs = AOA_LEFT_FOLDED_US;
    currentAoaRightUs = AOA_RIGHT_FOLDED_US;
    sweepMode = SWEEP_FOLDED;
  } else if (digitalRead(PIN_UNFOLD) == LOW && sweepMode != SWEEP_UNFOLDED) {
    targetSweepUs = SWEEP_UNFOLDED_US;
    currentAoaLeftUs = AOA_LEFT_FLAT_US;
    currentAoaRightUs = AOA_RIGHT_FLAT_US;
    sweepMode = SWEEP_UNFOLDED;
  } else if (digitalRead(PIN_MOUNT) == LOW && sweepMode != SWEEP_MOUNT) {
    targetSweepUs = SWEEP_MOUNT_US;
    currentAoaLeftUs = AOA_LEFT_FOLDED_US;
    currentAoaRightUs = AOA_RIGHT_FOLDED_US;
    sweepMode = SWEEP_MOUNT;
  }
  
  if (sweepMode != SWEEP_UNKNOWN) {
    if (!servosAttached) {
      servoMorph.attach(SERVO_MORPH_PIN, 500, 2500);
      servoLeft.attach(SERVO_LEFT_PIN, 500, 2500);
      servoRight.attach(SERVO_RIGHT_PIN, 500, 2500);
      currentSweepUs = targetSweepUs;
      servosAttached = true;
    }
    
    // Slow sweep for Morph servo
    float diff = targetSweepUs - currentSweepUs;
    // Max rate: ~2000 us per second. At 80Hz, that's 25 us per tick.
    float step = 25.0f;
    if (diff > step) currentSweepUs += step;
    else if (diff < -step) currentSweepUs -= step;
    else currentSweepUs = targetSweepUs;
    
    servoMorph.writeMicroseconds((int)currentSweepUs);
    servoLeft.writeMicroseconds((int)currentAoaLeftUs);
    servoRight.writeMicroseconds((int)currentAoaRightUs);
    
    // Record for logger (using the existing PWM fields)
    morphPwmUs = (int)currentSweepUs;
    leftPwmUs = (int)currentAoaLeftUs;
    rightPwmUs = (int)currentAoaRightUs;
  }
'''
content = re.sub(r'  if \(esekfHealthy\(\)\) \{.*?(?=  const uint32_t controlExecUs)', new_loop_logic, content, flags=re.DOTALL)

with open('wing_sweep_esekf_log.ino', 'w') as f:
    f.write(content)
