#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <rtos.h>

// BENCH_MODE 1: use timed arming for restrained bench testing.
// BENCH_MODE 0: require ARM_PIN to be released once, then pulled low to arm.
#define BENCH_MODE 0

// ENABLE_CONTROL 0: estimator/GPS/SD logging only; servos remain neutral.
// ENABLE_CONTROL 1: allow SMC after the selected arming condition is met.
#define ENABLE_CONTROL 1

// ENABLE_ADAPTIVE_B 1: run NLMS online B update.
// ENABLE_ADAPTIVE_B 0: freeze B at initial diagonal; required for first flight.
#define ENABLE_ADAPTIVE_B 0

// USE_TAIL_PITCH_CONTROL 0: tail stays at fixed trim; wings provide pitch.
// USE_TAIL_PITCH_CONTROL 1: tail elevator provides pitch while wing uSym is held near zero.
#define USE_TAIL_PITCH_CONTROL 0

// Experimental future phases are written for logging/planning review only.
// Keep both at 0 while the project is still focused on M3 gliding.
#define ENABLE_GPS_PHASE_TRANSITIONS 0
#define ENABLE_AUTOROTATION_PHASES 0

/*
  626_Mode12_DeployOnly

  Based on the execution architecture used by 69_gliding_pid_only:
    - The flight-control path never writes to the SD card.
    - A non-blocking record queue transfers snapshots to an RTOS logger thread.
    - If logging falls behind, a log sample is dropped instead of delaying control.

  Estimator:
    - 15-state error-state EKF:
      dx = [dtheta(3), dv(3), dp(3), dbg(3), dba(3)]
    - IMU prediction at 80 Hz.
    - Accelerometer roll/pitch correction only when acceleration is near 1 g.
    - Optional u-blox NEO-6M GPS correction using UBX NAV-POSLLH,
      NAV-SOL, and NAV-VELNED.
    - No magnetometer is assumed. Yaw remains weakly observable/unobservable when
      GPS motion does not provide a separate heading measurement.

  Controller:
    - Boundary-layer sliding mode control for roll and pitch.
    - No fixed aircraft A matrix is used.
    - A 2x2 control-effectiveness B matrix is estimated online from measured
      angular acceleration and the previous symmetric/differential wing command.

  GPS wiring for a GY-NEO6MV2 / NEO-6M module:
    GPS GND -> board GND
    GPS VCC -> module breakout rated supply
    GPS TX  -> board D1 / RX
    GPS RX  -> board D0 / TX
    UART    -> 9600 baud

  At startup the code configures the receiver for 5 Hz navigation, disables
  common NMEA messages, and enables the three UBX messages used by the ESEKF.

  IMPORTANT:
    This is an engineering flight-test scaffold. SMC gains, covariance/noise,
    servo direction, and B-estimator bounds require restrained bench/drop tests
    before unrestricted flight.
*/

// ========================= Edit each flight =========================

// Four digits are treated as MMDD and logged as MM_DD_ii.CSV, matching 0620.
static const char LOG_TAG[] = "M12";

// ========================= Timing and IO =========================

static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t SERIAL_WAIT_MS = 1500;
static const uint32_t GPS_BAUD = 9600;
static const uint16_t GPS_MEASUREMENT_PERIOD_MS = 200;

static const float CONTROL_HZ = 80.0f;
static const float DT_NOMINAL = 1.0f / CONTROL_HZ;
static const uint32_t CONTROL_DT_US =
    (uint32_t)(1000000.0f / CONTROL_HZ);
static const float MISSION_HZ = 10.0f;
static const uint32_t MISSION_DT_US =
    (uint32_t)(1000000.0f / MISSION_HZ);

static const float SD_LOG_HZ = 20.0f;
static const uint32_t SD_LOG_DT_US =
    (uint32_t)(1000000.0f / SD_LOG_HZ);
static const float RECORD_START_S = 0.0f;
static const float RECORD_END_S = 3600.0f;
static const int SD_CS_PIN = 10;
static const uint8_t SD_FLUSH_EVERY_N = 10;

static const int SERVO_LEFT_PIN = 4;
static const int SERVO_RIGHT_PIN = 3;
static const int SERVO_MORPH_PIN = 5;
static const int SERVO_TAIL_PIN = 6;
static const int ARM_PIN = 2;
static const bool SERVO_LEFT_REVERSE = true;
static const bool SERVO_RIGHT_REVERSE = false;
static const bool SERVO_MORPH_REVERSE = false;
static const bool SERVO_TAIL_REVERSE = false;
static const float SERVO_LEFT_GAIN = 1.00f;
static const float SERVO_RIGHT_GAIN = 1.00f;
static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2500;
static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 180.0f;
static const float SERVO_NEUTRAL_DEG = 90.0f;
static const float INCIDENCE_TRIM_DEG = 0.0f;
static const float SERVO_LEFT_NEUTRAL_DEG = 95.0f;
static const float SERVO_RIGHT_NEUTRAL_DEG = 95.0f;
static const float SERVO_MORPH_STOWED_DEG = 90.0f;
static const float SERVO_MORPH_DEPLOYED_DEG = 150.0f;
static const float SERVO_TAIL_TRIM_DEG = 90.0f;
static const float SERVO_CMD_LIMIT_DEG = 5.0f;
static const float SERVO_RATE_LIMIT_DEG_PER_S = 60.0f;
static const float SERVO_COMMAND_LPF_ALPHA = 0.20f;
static const float TAIL_CMD_LIMIT_DEG = 8.0f;
static const float TAIL_RATE_LIMIT_DEG_PER_S = 60.0f;
static const float TAIL_COMMAND_LPF_ALPHA = 0.20f;

// Mission manager follows the latest M0-M5 table.
// M0: carried/folded, M1: folded acceleration, M2: deployment transient,
// M3: glide navigation, M4: autorotation transition, M5: autorotation landing.
static const float BETA_FOLDED_DEG = SERVO_MORPH_STOWED_DEG;
static const float BETA_OPEN_DEG = SERVO_MORPH_DEPLOYED_DEG;
static const float T2_OPEN_S = 1.00f;
static const float T2_SETTLE_S = 0.25f;
static const float T2_MARGIN_S = 0.50f;
static const float T1_MIN_S = 0.20f;
static const float V_OPEN_MIN_MPS = 4.0f;
static const float H_OPEN_MIN_M = 20.0f;
static const float H1_LAST_M = 30.0f;
static const float H_FORCE_DEPLOY_M = 7.0f;
static const float AV_12_MPS2 = 0.0f;
static const float VZ_12_BAR_MPS = 6.0f;
static const float ALPHA_V_FOLDED_FALL = 0.65f;
static const float RELEASE_ALTITUDE_ESTIMATE_M = 45.0f;
static const bool USE_ESEKF_ALTITUDE_FOR_MISSION = false;
static const float MISSION_SPEED_MEAS_ALPHA = 0.35f;
static const float MISSION_ALTITUDE_MEAS_ALPHA = 0.20f;
static const float MISSION_MIN_VALID_SPEED_MPS = 0.50f;
static const bool HAS_WING_POSITION_FEEDBACK = false;
static const float LAMBDA_2_MIN = 0.90f;

// Disabled M3->M4 GPS trigger. Coordinates are local NEU meters from GPS origin.
// Fill these from the chosen autorotation entry region before enabling.
static const float AUTOROTATION_ENTRY_N_M = 0.0f;
static const float AUTOROTATION_ENTRY_E_M = 0.0f;
static const float AUTOROTATION_ENTRY_RADIUS_M = 12.0f;
static const uint8_t AUTOROTATION_ENTRY_CONFIRM_CYCLES = 10;

// Disabled M4/M5 descent-rate experiment.
static const uint32_t AUTOROTATION_TRANSITION_MS = 1000;
static const float SERVO_MORPH_AUTOROTATION_BASE_DEG = 90.0f;
static const float SERVO_MORPH_AUTOROTATION_MIN_DEG = 80.0f;
static const float SERVO_MORPH_AUTOROTATION_MAX_DEG = 130.0f;
static const float AUTOROTATION_NORMAL_DESCENT_MPS = 2.5f;
static const float AUTOROTATION_FAST_DESCENT_MPS = 3.5f;
static const float AUTOROTATION_STRONG_WIND_MPS = 4.0f;
static const float AUTOROTATION_DESCENT_SIGN = 1.0f;
static const float AUTOROTATION_DESCENT_GAIN_DEG_PER_MPS = 8.0f;
static const float AUTOROTATION_FILTER_ALPHA_VD = 0.10f;
static const float AUTOROTATION_FILTER_ALPHA_WIND = 0.10f;
static const float AUTOROTATION_FILTER_ALPHA_GYRO = 0.15f;

static const float G_MPS2 = 9.80665f;
static const float DEG_TO_RAD_F = 0.017453292519943295f;
static const float RAD_TO_DEG_F = 57.29577951308232f;

// ========================= ESEKF configuration =========================

// Error-state order:
//   0..2   attitude error, rad
//   3..5   world NEU velocity error, m/s
//   6..8   world NEU position error, m
//   9..11  gyro bias error, rad/s
//   12..14 accelerometer bias error, m/s^2
static const int ESEKF_N = 15;

static const float GYRO_NOISE_RADPS = 0.015f;
static const float ACCEL_NOISE_MPS2 = 0.18f;
static const float GYRO_BIAS_RW_RADPS = 0.0008f;
static const float ACCEL_BIAS_RW_MPS2 = 0.006f;
static const float ACCEL_LEVEL_NOISE_RAD = 3.0f * DEG_TO_RAD_F;
static const float ACCEL_GATE_LOW_MPS2 = 0.82f * G_MPS2;
static const float ACCEL_GATE_HIGH_MPS2 = 1.18f * G_MPS2;
static const float GPS_POSITION_NOISE_FLOOR_M = 1.5f;
static const float GPS_VERTICAL_NOISE_FLOOR_M = 2.5f;
static const float GPS_VELOCITY_NOISE_MPS = 0.35f;
static const uint32_t GPS_FIX_STALE_MS = 1500;
static const uint8_t GPS_ORIGIN_MIN_SATS = 6;
static const float GPS_ORIGIN_MAX_HACC_M = 3.0f;
static const uint8_t GPS_ORIGIN_STABLE_FIXES = 3;

static const uint16_t CAL_SAMPLES = 500;
static const uint16_t CAL_DT_MS = 4;

// ========================= SMC and online B estimate =========================

// These targets are intentionally placeholders. For flight, replace them with
// the stable passive-glide roll and pitch observed with neutral wings/tail.
// GPS is currently used for logging and low-frequency ESEKF correction only;
// target landing guidance needs a wind/airspeed estimate before it is trusted.
static const float ROLL_TARGET_RAD = 0.0f;
static const float PITCH_TARGET_RAD = 0.0f;

// s = rate + lambda * angle_error.
static const float SMC_LAMBDA_ROLL = 0.80f;
static const float SMC_LAMBDA_PITCH = 0.80f;
static const float SMC_K_ROLL_RADPS2 = 0.60f;
static const float SMC_K_PITCH_RADPS2 = 0.60f;
static const float SMC_PHI_ROLL_RADPS = 0.35f;
static const float SMC_PHI_PITCH_RADPS = 0.35f;
static const float SMC_ROLL_DEADBAND_RAD = 5.0f * DEG_TO_RAD_F;
static const float SMC_PITCH_DEADBAND_RAD = 3.0f * DEG_TO_RAD_F;
static const float SMC_RATE_DEADBAND_RADPS = 5.0f * DEG_TO_RAD_F;
static const uint32_t SMC_ARM_DELAY_MS = 750;

// B maps [u_diff, u_sym] in radians to [p_dot, q_dot] in rad/s^2.
// The initial signs follow the existing 64/69 servo mixing convention.
static const float B_ROLL_INITIAL = -6.0f;
static const float B_PITCH_INITIAL = 6.0f;
static const float B_TAIL_PITCH_INITIAL = 6.0f;
static const float B_DIAG_MIN_ABS = 0.75f;
static const float B_DIAG_MAX_ABS = 60.0f;
static const float B_CROSS_MAX_ABS = 15.0f;
static const float B_NLMS_GAIN = 0.025f;
static const float B_EXCITATION_MIN_RAD = 1.5f * DEG_TO_RAD_F;
static const float B_ALPHA_LPF = 0.20f;
static const float DISTURBANCE_LPF = 0.015f;
static const float DISTURBANCE_MAX_RADPS2 = 20.0f;
static const float ANGULAR_ACCEL_MAX_RADPS2 = 100.0f;

// ========================= Data types =========================

struct ImuSample {
  float accel[3];
  float gyro[3];
  bool valid;
};

struct GpsSample {
  bool bytesSeen;
  bool fix;
  bool fresh;
  uint8_t fixType;
  uint8_t satellites;
  uint32_t iTowMs;
  uint32_t positionTowMs;
  uint32_t velocityTowMs;
  uint32_t solutionTowMs;
  uint32_t lastFusedTowMs;
  uint32_t receivedMs;
  double latitudeDeg;
  double longitudeDeg;
  float altitudeM;
  float hAccM;
  float vAccM;
  float velocityNed[3];
};

struct ESEKFState {
  float q[4];
  float v[3];
  float p[3];
  float bg[3];
  float ba[3];
  float P[ESEKF_N][ESEKF_N];
  bool initialized;
};

struct VehicleState {
  float timeS;
  float vxG;
  float vyG;
  float vzG;
  float wx;
  float wy;
  float wz;
  float x;
  float y;
  float h;
  float p;
  float q;
  float r;
  float rollPhi;
  float pitchTheta;
  float headingPsi;
  float beta;
  float airspeed;
};

enum RecordKind : uint8_t {
  RECORD_DATA = 0,
  RECORD_STOP = 1
};

enum FlightMode : uint8_t {
  MODE_CARRIED_FOLDED = 0,
  MODE_FOLDED_ACCELERATION = 1,
  MODE_DEPLOYMENT_TRANSIENT = 2,
  MODE_GLIDE_NAVIGATION = 3,
  MODE_AUTOROTATION_TRANSITION = 4,
  MODE_AUTOROTATION_LANDING = 5,
  MODE_FAILSAFE = 99
};

struct ControlRecord {
  uint8_t kind;
  uint8_t flightMode;
  float modeElapsedS;
  float missionAirspeedMps;
  float missionAltitudeM;
  float missionBetaCmdDeg;
  float missionDeployProgress;
  uint8_t missionD12Normal;
  uint8_t missionD12Last;
  uint8_t missionD12GroundForce;
  uint8_t missionD23;
  uint32_t ms;
  uint32_t controlDtUs;
  uint32_t controlExecUs;
  uint32_t esekfUs;
  uint32_t missedPeriods;
  uint32_t logDropped;
  float accel[3];
  float gyro[3];
  float rollDeg;
  float pitchDeg;
  float yawDeg;
  float velocityNeu[3];
  float positionNeu[3];
  float gyroBias[3];
  float accelBias[3];
  float sRoll;
  float sPitch;
  float uDiffDeg;
  float uSymDeg;
  float uTailDeg;
  float leftIncidenceDeg;
  float rightIncidenceDeg;
  float bMatrix[2][2];
  float leftServoDeg;
  float rightServoDeg;
  float morphServoDeg;
  float tailServoDeg;
  float autoVDownFilt;
  float autoWindSpeedFilt;
  float autoGyroMagFilt;
  float autoMorphTargetDeg;
  uint16_t leftPwmUs;
  uint16_t rightPwmUs;
  uint16_t morphPwmUs;
  uint16_t tailPwmUs;
  bool gpsFix;
  uint8_t gpsSatellites;
  double gpsLatitudeDeg;
  double gpsLongitudeDeg;
  float gpsAltitudeM;
  float gpsVelocityNed[3];
};

// ========================= Non-blocking logger queues =========================

/*
  KEY DESIGN NOTE, inherited from 69:

  The control loop never calls SD.write() or SD.flush(). It only:
    1. takes a free preallocated record slot with timeout 0,
    2. fills the slot,
    3. puts the pointer into filledQueue with timeout 0.

  If no slot is available, the sample is dropped immediately. Flight control
  therefore has priority over data completeness.
*/
static const uint8_t QUEUE_DEPTH = 24;
static ControlRecord recordPool[QUEUE_DEPTH];
static rtos::Queue<ControlRecord, QUEUE_DEPTH> filledQueue;
static rtos::Queue<ControlRecord, QUEUE_DEPTH> freeQueue;
static rtos::Thread loggerThread;
static rtos::Mutex serialMutex;

static void safeSerialPrintln(const __FlashStringHelper *s) {
  serialMutex.lock();
  Serial.println(s);
  serialMutex.unlock();
}

static void safeSerialPrint(const __FlashStringHelper *s) {
  serialMutex.lock();
  Serial.print(s);
  serialMutex.unlock();
}

// ========================= Global state =========================

static Servo servoLeft;
static Servo servoRight;
static Servo servoMorph;
static Servo servoTail;
static ImuSample imu = {};
static GpsSample gps = {};
static ESEKFState esekf = {};

static float gyroCalibration[3] = {};
static bool gpsOriginSet = false;
static double gpsOriginLatDeg = 0.0;
static double gpsOriginLonDeg = 0.0;
static float gpsOriginAltM = 0.0f;
static uint8_t gpsOriginGoodCount = 0;

static float bHat[2][2] = {
  {B_ROLL_INITIAL, 0.0f},
  {0.0f, B_PITCH_INITIAL}
};
static float matchedDisturbance[2] = {};
static float previousRate[2] = {};
static float filteredAngularAccel[2] = {};
static float previousControlRad[2] = {};
static bool bEstimatorReady = false;

static float rollRad = 0.0f;
static float pitchRad = 0.0f;
static float yawRad = 0.0f;
static float sRoll = 0.0f;
static float sPitch = 0.0f;
static float uDiffRad = 0.0f;
static float uSymRad = 0.0f;
static float uTailRad = 0.0f;
static float uDiffFilteredRad = 0.0f;
static float uSymFilteredRad = 0.0f;
static float uTailFilteredRad = 0.0f;
static float leftServoDeg = SERVO_LEFT_NEUTRAL_DEG;
static float rightServoDeg = SERVO_RIGHT_NEUTRAL_DEG;
static float morphServoDeg = SERVO_MORPH_STOWED_DEG;
static float tailServoDeg = SERVO_TAIL_TRIM_DEG;
static uint16_t leftPwmUs = 1500;
static uint16_t rightPwmUs = 1500;
static uint16_t morphPwmUs = 1500;
static uint16_t tailPwmUs = 1500;

static uint32_t nextControlUs = 0;
static uint32_t lastControlUs = 0;
static uint32_t nextLogUs = 0;
static uint32_t logClockStartMs = 0;
static bool logWindowOpen = false;
static bool logWindowDone = false;
static bool logStopQueued = false;

static const uint32_t ARMING_DELAY_MS = 5000;
static const uint8_t ARM_DEBOUNCE_CYCLES = 4;
static bool armed = false;
static uint32_t armedSinceMs = 0;
static bool armInputSeenHigh = false;
static uint8_t armLowCount = 0;
static uint8_t armHighCount = 0;
static bool imuFaultLocked = false;
static bool esekfFaultLocked = false;
static FlightMode flightMode = MODE_CARRIED_FOLDED;
static uint32_t modeStartUs = 0;
static uint32_t nextMissionUs = 0;
static float modeElapsedS = 0.0f;
static float missionBetaCmdDeg = BETA_FOLDED_DEG;
static float missionDeployProgress = 0.0f;
static bool missionD12Normal = false;
static bool missionD12Last = false;
static bool missionD12GroundForce = false;
static bool missionD23 = false;
static VehicleState missionState = {};
static bool missionStateInitialized = false;
static uint32_t missionStateLastUs = 0;
static float missionAirspeedEstMps = 0.0f;
static float missionAltitudeEstM = RELEASE_ALTITUDE_ESTIMATE_M;
static uint32_t autorotationModeStartMs = 0;
static uint8_t autorotationEntryConfirmCount = 0;
static float autorotationVDownFilt = 0.0f;
static float autorotationWindSpeedFilt = 0.0f;
static float autorotationGyroMagFilt = 0.0f;
static float autorotationMorphTargetDeg = SERVO_MORPH_AUTOROTATION_BASE_DEG;

static const uint8_t ESEKF_SETTLE_CYCLES = 16;
static uint8_t esekfSettleRemaining = 0;

static volatile uint32_t controlCount = 0;
static volatile uint32_t missedPeriodCount = 0;
static volatile uint32_t logDroppedCount = 0;
static volatile uint32_t maxControlDtUs = 0;
static volatile uint32_t maxControlExecUs = 0;
static volatile uint32_t maxEsekfUs = 0;
static volatile uint32_t gpsEpochCount = 0;
static volatile uint32_t esekfResetCount = 0;

// Scratch matrices are global to avoid large temporary arrays on the stack.
static float phiMat[ESEKF_N][ESEKF_N];
static float tempMat[ESEKF_N][ESEKF_N];
static float covarianceNext[ESEKF_N][ESEKF_N];

// ========================= Math helpers =========================

static float sqf(float x) {
  return x * x;
}

static float clampfLocal(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float wrapPi(float x) {
  return x - 2.0f * PI * floorf((x + PI) / (2.0f * PI));
}

static float sat(float x) {
  return clampfLocal(x, -1.0f, 1.0f);
}

static float deadbandZero(float x, float band) {
  if (fabsf(x) <= band) return 0.0f;
  return x;
}

static float rateLimit(float target, float current, float maxStep) {
  return current + clampfLocal(target - current, -maxStep, maxStep);
}

static const char *flightModeName(FlightMode mode) {
  switch (mode) {
    case MODE_CARRIED_FOLDED:
      return "M0_carried_folded";
    case MODE_FOLDED_ACCELERATION:
      return "M1_folded_accel";
    case MODE_DEPLOYMENT_TRANSIENT:
      return "M2_deploy_transient";
    case MODE_GLIDE_NAVIGATION:
      return "M3_glide_nav";
    case MODE_AUTOROTATION_TRANSITION:
      return "M4_autorotation_transition";
    case MODE_AUTOROTATION_LANDING:
      return "M5_autorotation_landing";
    case MODE_FAILSAFE:
      return "MODE_FAILSAFE";
    default:
      return "M_unknown";
  }
}

static bool logTagLooksMmdd() {
  if (sizeof(LOG_TAG) != 5) return false;
  return LOG_TAG[0] >= '0' && LOG_TAG[0] <= '9' &&
         LOG_TAG[1] >= '0' && LOG_TAG[1] <= '9' &&
         LOG_TAG[2] >= '0' && LOG_TAG[2] <= '9' &&
         LOG_TAG[3] >= '0' && LOG_TAG[3] <= '9' &&
         LOG_TAG[4] == '\0';
}

static void makeLogFilename(char *filename, size_t filenameSize, int index) {
  if (logTagLooksMmdd()) {
    snprintf(
        filename,
        filenameSize,
        "%c%c_%c%c_%02d.CSV",
        LOG_TAG[0],
        LOG_TAG[1],
        LOG_TAG[2],
        LOG_TAG[3],
        index);
  } else {
    snprintf(filename, filenameSize, "%s_%02d.CSV", LOG_TAG, index);
  }
}

static void clearControlState() {
  sRoll = 0.0f;
  sPitch = 0.0f;
  uDiffRad = 0.0f;
  uSymRad = 0.0f;
  uTailRad = 0.0f;
  uDiffFilteredRad = 0.0f;
  uSymFilteredRad = 0.0f;
  uTailFilteredRad = 0.0f;
  leftServoDeg = SERVO_LEFT_NEUTRAL_DEG;
  rightServoDeg = SERVO_RIGHT_NEUTRAL_DEG;
  tailServoDeg = SERVO_TAIL_TRIM_DEG;
  previousControlRad[0] = 0.0f;
  previousControlRad[1] = 0.0f;
  matchedDisturbance[0] = 0.0f;
  matchedDisturbance[1] = 0.0f;
  filteredAngularAccel[0] = 0.0f;
  filteredAngularAccel[1] = 0.0f;
  previousRate[0] = 0.0f;
  previousRate[1] = 0.0f;
  bHat[0][0] = B_ROLL_INITIAL;
  bHat[0][1] = 0.0f;
  bHat[1][0] = 0.0f;
  bHat[1][1] = B_PITCH_INITIAL;
  autorotationVDownFilt = 0.0f;
  autorotationWindSpeedFilt = 0.0f;
  autorotationGyroMagFilt = 0.0f;
  autorotationMorphTargetDeg = SERVO_MORPH_AUTOROTATION_BASE_DEG;
  bEstimatorReady = false;
}

static uint32_t recordStartMs() {
  return (uint32_t)(RECORD_START_S * 1000.0f + 0.5f);
}

static uint32_t recordEndMs() {
  return (uint32_t)(RECORD_END_S * 1000.0f + 0.5f);
}

static void quatNormalize(float q[4]) {
  const float n =
      sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1.0e-8f) {
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
    return;
  }
  for (int i = 0; i < 4; i++) q[i] /= n;
}

static void quatMultiply(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0] * b[0] - a[1] * b[1] -
           a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] +
           a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] +
           a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] -
           a[2] * b[1] + a[3] * b[0];
}

static void integrateQuaternion(float q[4], const float omega[3], float dt) {
  const float wn = sqrtf(
      omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]);
  float dq[4];
  if (wn < 1.0e-8f) {
    dq[0] = 1.0f;
    dq[1] = 0.5f * omega[0] * dt;
    dq[2] = 0.5f * omega[1] * dt;
    dq[3] = 0.5f * omega[2] * dt;
  } else {
    const float halfAngle = 0.5f * wn * dt;
    const float scale = sinf(halfAngle) / wn;
    dq[0] = cosf(halfAngle);
    dq[1] = scale * omega[0];
    dq[2] = scale * omega[1];
    dq[3] = scale * omega[2];
  }
  float qNext[4];
  quatMultiply(q, dq, qNext);
  for (int i = 0; i < 4; i++) q[i] = qNext[i];
  quatNormalize(q);
}

static void quaternionToRotation(const float q[4], float r[3][3]) {
  const float w = q[0];
  const float x = q[1];
  const float y = q[2];
  const float z = q[3];
  r[0][0] = 1.0f - 2.0f * (y * y + z * z);
  r[0][1] = 2.0f * (x * y - z * w);
  r[0][2] = 2.0f * (x * z + y * w);
  r[1][0] = 2.0f * (x * y + z * w);
  r[1][1] = 1.0f - 2.0f * (x * x + z * z);
  r[1][2] = 2.0f * (y * z - x * w);
  r[2][0] = 2.0f * (x * z - y * w);
  r[2][1] = 2.0f * (y * z + x * w);
  r[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

static void eulerFromQuaternion(
    const float q[4], float &roll, float &pitch, float &yaw) {
  const float w = q[0];
  const float x = q[1];
  const float y = q[2];
  const float z = q[3];
  roll = atan2f(
      2.0f * (w * x + y * z),
      1.0f - 2.0f * (x * x + y * y));
  pitch = asinf(clampfLocal(2.0f * (w * y - z * x), -1.0f, 1.0f));
  yaw = atan2f(
      2.0f * (w * z + x * y),
      1.0f - 2.0f * (y * y + z * z));
}

static void quaternionFromRollPitch(float roll, float pitch, float q[4]) {
  const float cr = cosf(0.5f * roll);
  const float sr = sinf(0.5f * roll);
  const float cp = cosf(0.5f * pitch);
  const float sp = sinf(0.5f * pitch);
  q[0] = cp * cr;
  q[1] = cp * sr;
  q[2] = sp * cr;
  q[3] = -sp * sr;
  quatNormalize(q);
}

static void injectSmallAngle(float q[4], const float dtheta[3]) {
  float dq[4] = {
    1.0f,
    0.5f * dtheta[0],
    0.5f * dtheta[1],
    0.5f * dtheta[2]
  };
  quatNormalize(dq);
  float qNext[4];
  quatMultiply(q, dq, qNext);
  for (int i = 0; i < 4; i++) q[i] = qNext[i];
  quatNormalize(q);
}

// ========================= GPS NEO-6M UBX parser =========================

/*
  KEY GPS NOTE:

  GPS is optional. Serial1 is polled without waiting. No characters, a stale
  packet, or a packet without a valid 3D fix simply leaves gps.fix false.
  The ESEKF then continues as an IMU-only estimator and no error is printed.

  NEO-6 firmware does not provide the newer 92-byte NAV-PVT packet. The same
  information is assembled from three messages sharing the same GPS iTOW:
    NAV-POSLLH (0x01 0x02): latitude, longitude, altitude, position accuracy
    NAV-SOL    (0x01 0x06): fix validity and satellite count
    NAV-VELNED (0x01 0x12): north/east/down velocity
*/
enum UbxState : uint8_t {
  UBX_SYNC1,
  UBX_SYNC2,
  UBX_CLASS,
  UBX_ID,
  UBX_LEN1,
  UBX_LEN2,
  UBX_PAYLOAD,
  UBX_CK_A,
  UBX_CK_B
};

static UbxState ubxState = UBX_SYNC1;
static uint8_t ubxClass = 0;
static uint8_t ubxId = 0;
static uint16_t ubxLength = 0;
static uint16_t ubxIndex = 0;
static uint8_t ubxChecksumA = 0;
static uint8_t ubxChecksumB = 0;
static uint8_t ubxPayload[100];

static void ubxChecksumAdd(uint8_t value) {
  ubxChecksumA += value;
  ubxChecksumB += ubxChecksumA;
}

static uint16_t readU16Le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t readU32Le(const uint8_t *p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static int32_t readI32Le(const uint8_t *p) {
  return (int32_t)readU32Le(p);
}

static void refreshGpsEpoch() {
  const bool sameEpoch =
      gps.positionTowMs == gps.solutionTowMs &&
      gps.velocityTowMs == gps.solutionTowMs;
  if (!gps.fix || !sameEpoch || gps.solutionTowMs == gps.lastFusedTowMs) return;

  gps.iTowMs = gps.solutionTowMs;
  gps.lastFusedTowMs = gps.solutionTowMs;
  gps.receivedMs = millis();
  gps.fresh = true;
  gpsEpochCount++;
}

static void handleNavPosllh() {
  if (ubxLength != 28) return;

  gps.positionTowMs = readU32Le(&ubxPayload[0]);
  gps.longitudeDeg = readI32Le(&ubxPayload[4]) * 1.0e-7;
  gps.latitudeDeg = readI32Le(&ubxPayload[8]) * 1.0e-7;
  gps.altitudeM = readI32Le(&ubxPayload[16]) * 1.0e-3f;
  gps.hAccM = readU32Le(&ubxPayload[20]) * 1.0e-3f;
  gps.vAccM = readU32Le(&ubxPayload[24]) * 1.0e-3f;
  refreshGpsEpoch();
}

static void handleNavSol() {
  if (ubxLength != 52) return;

  gps.solutionTowMs = readU32Le(&ubxPayload[0]);
  gps.fixType = ubxPayload[10];
  const uint8_t flags = ubxPayload[11];
  gps.satellites = ubxPayload[47];
  gps.fix = ((flags & 0x01U) != 0U) && gps.fixType >= 3;
  if (!gps.fix) gps.fresh = false;
  refreshGpsEpoch();
}

static void handleNavVelned() {
  if (ubxLength != 36) return;

  gps.velocityTowMs = readU32Le(&ubxPayload[0]);
  // NEO-6 NAV-VELNED velocities are signed cm/s.
  gps.velocityNed[0] = readI32Le(&ubxPayload[4]) * 0.01f;
  gps.velocityNed[1] = readI32Le(&ubxPayload[8]) * 0.01f;
  gps.velocityNed[2] = readI32Le(&ubxPayload[12]) * 0.01f;
  refreshGpsEpoch();
}

static void handleUbxPacket() {
  if (ubxClass != 0x01) return;
  if (ubxId == 0x02) {
    handleNavPosllh();
  } else if (ubxId == 0x06) {
    handleNavSol();
  } else if (ubxId == 0x12) {
    handleNavVelned();
  }
}

static void parseUbxByte(uint8_t value) {
  switch (ubxState) {
    case UBX_SYNC1:
      if (value == 0xB5) ubxState = UBX_SYNC2;
      break;
    case UBX_SYNC2:
      ubxState = (value == 0x62) ? UBX_CLASS : UBX_SYNC1;
      break;
    case UBX_CLASS:
      ubxClass = value;
      ubxChecksumA = 0;
      ubxChecksumB = 0;
      ubxChecksumAdd(value);
      ubxState = UBX_ID;
      break;
    case UBX_ID:
      ubxId = value;
      ubxChecksumAdd(value);
      ubxState = UBX_LEN1;
      break;
    case UBX_LEN1:
      ubxLength = value;
      ubxChecksumAdd(value);
      ubxState = UBX_LEN2;
      break;
    case UBX_LEN2:
      ubxLength |= ((uint16_t)value << 8);
      ubxChecksumAdd(value);
      ubxIndex = 0;
      if (ubxLength > sizeof(ubxPayload)) {
        ubxState = UBX_SYNC1;
      } else {
        ubxState = (ubxLength == 0) ? UBX_CK_A : UBX_PAYLOAD;
      }
      break;
    case UBX_PAYLOAD:
      ubxPayload[ubxIndex++] = value;
      ubxChecksumAdd(value);
      if (ubxIndex >= ubxLength) ubxState = UBX_CK_A;
      break;
    case UBX_CK_A:
      ubxState = (value == ubxChecksumA) ? UBX_CK_B : UBX_SYNC1;
      break;
    case UBX_CK_B:
      if (value == ubxChecksumB) handleUbxPacket();
      ubxState = UBX_SYNC1;
      break;
  }
}

static void sendUbx(
    uint8_t messageClass,
    uint8_t messageId,
    const uint8_t *payload,
    uint16_t payloadLength) {
  uint8_t checksumA = 0;
  uint8_t checksumB = 0;

  Serial1.write(0xB5);
  Serial1.write(0x62);

  const uint8_t header[4] = {
    messageClass,
    messageId,
    (uint8_t)(payloadLength & 0xFFU),
    (uint8_t)(payloadLength >> 8)
  };
  for (uint8_t i = 0; i < sizeof(header); i++) {
    Serial1.write(header[i]);
    checksumA += header[i];
    checksumB += checksumA;
  }
  for (uint16_t i = 0; i < payloadLength; i++) {
    Serial1.write(payload[i]);
    checksumA += payload[i];
    checksumB += checksumA;
  }
  Serial1.write(checksumA);
  Serial1.write(checksumB);
  Serial1.flush();
}

static void configureUbxMessage(
    uint8_t targetClass,
    uint8_t targetId,
    uint8_t rate) {
  const uint8_t payload[3] = {targetClass, targetId, rate};
  sendUbx(0x06, 0x01, payload, sizeof(payload));  // UBX-CFG-MSG
  delay(15);
}

static void configureNeo6m() {
  // Configure a 200 ms measurement period: 5 Hz measurement and navigation.
  const uint8_t ratePayload[6] = {
    (uint8_t)(GPS_MEASUREMENT_PERIOD_MS & 0xFFU),
    (uint8_t)(GPS_MEASUREMENT_PERIOD_MS >> 8),
    0x01, 0x00,  // navRate = 1
    0x01, 0x00   // timeRef = GPS time
  };
  sendUbx(0x06, 0x08, ratePayload, sizeof(ratePayload));  // UBX-CFG-RATE
  delay(30);

  // Disable common default NMEA sentences to preserve the 9600-baud budget.
  for (uint8_t nmeaId = 0x00; nmeaId <= 0x05; nmeaId++) {
    configureUbxMessage(0xF0, nmeaId, 0);
  }

  // Enable the NEO-6 messages required to assemble one ESEKF GPS update.
  configureUbxMessage(0x01, 0x02, 1);  // NAV-POSLLH
  configureUbxMessage(0x01, 0x06, 1);  // NAV-SOL
  configureUbxMessage(0x01, 0x12, 1);  // NAV-VELNED
}

static void pollGps() {
  while (Serial1.available() > 0) {
    gps.bytesSeen = true;
    parseUbxByte((uint8_t)Serial1.read());
  }

  if (gps.fix &&
      (uint32_t)(millis() - gps.receivedMs) > GPS_FIX_STALE_MS) {
    gps.fix = false;
    gps.fresh = false;
  }
}

// ========================= IMU =========================

static bool readImu(ImuSample &sample) {
  const uint32_t startUs = micros();
  while (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
    if ((uint32_t)(micros() - startUs) > 8000UL) {
      sample.valid = false;
      return false;
    }
  }

  float axG, ayG, azG;
  float gxDps, gyDps, gzDps;
  IMU.readAcceleration(axG, ayG, azG);
  IMU.readGyroscope(gxDps, gyDps, gzDps);
  sample.accel[0] = axG * G_MPS2;
  sample.accel[1] = ayG * G_MPS2;
  sample.accel[2] = azG * G_MPS2;
  sample.gyro[0] = gxDps * DEG_TO_RAD_F;
  sample.gyro[1] = gyDps * DEG_TO_RAD_F;
  sample.gyro[2] = gzDps * DEG_TO_RAD_F;
  sample.valid = true;
  return true;
}

static bool calibrateGyro() {
  float sum[3] = {};
  float sumSq[3] = {};
  uint16_t good = 0;
  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    ImuSample sample = {};
    if (readImu(sample)) {
      const float aN = sqrtf(sample.accel[0] * sample.accel[0] +
                             sample.accel[1] * sample.accel[1] +
                             sample.accel[2] * sample.accel[2]);
      if (aN < ACCEL_GATE_LOW_MPS2 || aN > ACCEL_GATE_HIGH_MPS2) return false;
      for (int axis = 0; axis < 3; axis++) {
        sum[axis] += sample.gyro[axis];
        sumSq[axis] += sample.gyro[axis] * sample.gyro[axis];
      }
      good++;
    }
    delay(CAL_DT_MS);
  }
  if (good < CAL_SAMPLES / 2) return false;
  for (int axis = 0; axis < 3; axis++) {
    gyroCalibration[axis] = sum[axis] / (float)good;
    const float mean = gyroCalibration[axis];
    const float variance = sumSq[axis] / (float)good - mean * mean;
    if (fabsf(mean) > 5.0f * GYRO_NOISE_RADPS) return false;
    if (variance > sqf(3.0f * GYRO_NOISE_RADPS)) return false;
  }
  return true;
}

// ========================= ESEKF =========================

static void resetEsekfFromAccel(const ImuSample &sample) {
  const float ax = sample.accel[0];
  const float ay = sample.accel[1];
  const float az = sample.accel[2];
  const float initialRoll = atan2f(ay, az);
  const float initialPitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  quaternionFromRollPitch(initialRoll, initialPitch, esekf.q);

  for (int i = 0; i < 3; i++) {
    esekf.v[i] = 0.0f;
    esekf.p[i] = 0.0f;
    esekf.bg[i] = gyroCalibration[i];
    esekf.ba[i] = 0.0f;
  }

  for (int r = 0; r < ESEKF_N; r++) {
    for (int c = 0; c < ESEKF_N; c++) esekf.P[r][c] = 0.0f;
  }
  for (int i = 0; i < 3; i++) {
    esekf.P[i][i] = sqf(8.0f * DEG_TO_RAD_F);
    esekf.P[i + 3][i + 3] = sqf(1.0f);
    esekf.P[i + 6][i + 6] = sqf(3.0f);
    esekf.P[i + 9][i + 9] = sqf(0.04f);
    esekf.P[i + 12][i + 12] = sqf(0.30f);
  }
  esekf.initialized = true;
}

static void applyErrorState(const float dx[ESEKF_N]) {
  injectSmallAngle(esekf.q, dx);
  for (int i = 0; i < 3; i++) {
    esekf.v[i] += dx[i + 3];
    esekf.p[i] += dx[i + 6];
    esekf.bg[i] += dx[i + 9];
    esekf.ba[i] += dx[i + 12];
  }
}

static void symmetrizeAndFloorCovariance() {
  for (int r = 0; r < ESEKF_N; r++) {
    for (int c = r + 1; c < ESEKF_N; c++) {
      const float average = 0.5f * (esekf.P[r][c] + esekf.P[c][r]);
      esekf.P[r][c] = average;
      esekf.P[c][r] = average;
    }
    if (esekf.P[r][r] < 1.0e-10f) esekf.P[r][r] = 1.0e-10f;
  }
}

static bool esekfHealthy() {
  float quaternionNorm = 0.0f;
  for (int i = 0; i < 4; i++) {
    if (!isfinite(esekf.q[i])) return false;
    quaternionNorm += esekf.q[i] * esekf.q[i];
  }
  if (quaternionNorm < 0.5f || quaternionNorm > 1.5f) return false;

  for (int i = 0; i < 3; i++) {
    if (!isfinite(esekf.v[i]) ||
        !isfinite(esekf.p[i]) ||
        !isfinite(esekf.bg[i]) ||
        !isfinite(esekf.ba[i])) {
      return false;
    }
  }
  for (int i = 0; i < ESEKF_N; i++) {
    if (!isfinite(esekf.P[i][i]) || esekf.P[i][i] <= 0.0f) return false;
  }
  return true;
}

/*
  KEY ESEKF NOTE:

  The nominal state is propagated nonlinearly with the quaternion and IMU.
  The covariance uses Phi = I + F*dt for the 15-state error dynamics:

    dtheta_dot = -skew(w)*dtheta - dbg
    dv_dot     = -R*skew(a)*dtheta - R*dba
    dp_dot     = dv

  This is more expensive than the complementary filter, so the control rate is
  intentionally reduced from 100 Hz to 80 Hz. The SD writer remains outside
  this path, preserving a deterministic control-time budget.
*/
static void predictEsekf(const ImuSample &sample, float dt) {
  float omega[3];
  float accelBody[3];
  for (int i = 0; i < 3; i++) {
    omega[i] = sample.gyro[i] - esekf.bg[i];
    accelBody[i] = sample.accel[i] - esekf.ba[i];
  }

  integrateQuaternion(esekf.q, omega, dt);

  float rotation[3][3];
  quaternionToRotation(esekf.q, rotation);
  float accelWorld[3] = {
    rotation[0][0] * accelBody[0] +
        rotation[0][1] * accelBody[1] +
        rotation[0][2] * accelBody[2],
    rotation[1][0] * accelBody[0] +
        rotation[1][1] * accelBody[1] +
        rotation[1][2] * accelBody[2],
    rotation[2][0] * accelBody[0] +
        rotation[2][1] * accelBody[1] +
        rotation[2][2] * accelBody[2] - G_MPS2
  };

  for (int i = 0; i < 3; i++) {
    esekf.p[i] += esekf.v[i] * dt + 0.5f * accelWorld[i] * dt * dt;
    esekf.v[i] += accelWorld[i] * dt;
  }

  for (int r = 0; r < ESEKF_N; r++) {
    for (int c = 0; c < ESEKF_N; c++) {
      phiMat[r][c] = (r == c) ? 1.0f : 0.0f;
    }
  }

  const float skewW[3][3] = {
    {0.0f, -omega[2], omega[1]},
    {omega[2], 0.0f, -omega[0]},
    {-omega[1], omega[0], 0.0f}
  };
  const float skewA[3][3] = {
    {0.0f, -accelBody[2], accelBody[1]},
    {accelBody[2], 0.0f, -accelBody[0]},
    {-accelBody[1], accelBody[0], 0.0f}
  };

  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      phiMat[r][c] += -skewW[r][c] * dt;

      float rSkewA = 0.0f;
      for (int k = 0; k < 3; k++) {
        rSkewA += rotation[r][k] * skewA[k][c];
      }
      phiMat[r + 3][c] += -rSkewA * dt;
      phiMat[r + 3][c + 12] += -rotation[r][c] * dt;
    }
    phiMat[r][r + 9] += -dt;
    phiMat[r + 6][r + 3] += dt;
  }

  for (int r = 0; r < ESEKF_N; r++) {
    for (int c = 0; c < ESEKF_N; c++) {
      float value = 0.0f;
      for (int k = 0; k < ESEKF_N; k++) {
        value += phiMat[r][k] * esekf.P[k][c];
      }
      tempMat[r][c] = value;
    }
  }

  for (int r = 0; r < ESEKF_N; r++) {
    for (int c = 0; c < ESEKF_N; c++) {
      float value = 0.0f;
      for (int k = 0; k < ESEKF_N; k++) {
        value += tempMat[r][k] * phiMat[c][k];
      }
      covarianceNext[r][c] = value;
    }
  }

  const float qTheta = sqf(GYRO_NOISE_RADPS) * dt;
  const float qVelocity = sqf(ACCEL_NOISE_MPS2) * dt;
  const float qPosition = qVelocity * dt * dt;
  const float qGyroBias = sqf(GYRO_BIAS_RW_RADPS) * dt;
  const float qAccelBias = sqf(ACCEL_BIAS_RW_MPS2) * dt;
  for (int i = 0; i < 3; i++) {
    covarianceNext[i][i] += qTheta;
    covarianceNext[i + 3][i + 3] += qVelocity;
    covarianceNext[i + 6][i + 6] += qPosition;
    covarianceNext[i + 9][i + 9] += qGyroBias;
    covarianceNext[i + 12][i + 12] += qAccelBias;
  }

  for (int r = 0; r < ESEKF_N; r++) {
    for (int c = 0; c < ESEKF_N; c++) {
      esekf.P[r][c] = covarianceNext[r][c];
    }
  }
  symmetrizeAndFloorCovariance();
}

static bool scalarStateUpdate(
    int observedIndex,
    float residual,
    float measurementVariance,
    float residualLimit) {
  if (!isfinite(residual) ||
      !isfinite(measurementVariance) ||
      fabsf(residual) > residualLimit) {
    return false;
  }

  const float innovationVariance =
      esekf.P[observedIndex][observedIndex] + measurementVariance;
  if (innovationVariance < 1.0e-9f) return false;

  float kalmanGain[ESEKF_N];
  float observedRow[ESEKF_N];
  float dx[ESEKF_N];
  for (int i = 0; i < ESEKF_N; i++) {
    kalmanGain[i] = esekf.P[i][observedIndex] / innovationVariance;
    observedRow[i] = esekf.P[observedIndex][i];
    dx[i] = kalmanGain[i] * residual;
  }

  applyErrorState(dx);

  // Sequential scalar covariance update: P = P - K*(H*P).
  for (int r = 0; r < ESEKF_N; r++) {
    for (int c = 0; c < ESEKF_N; c++) {
      esekf.P[r][c] -= kalmanGain[r] * observedRow[c];
    }
  }
  symmetrizeAndFloorCovariance();
  return true;
}

static void updateEsekfWithAccelerometer(const ImuSample &sample) {
  const float ax = sample.accel[0];
  const float ay = sample.accel[1];
  const float az = sample.accel[2];
  const float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm < ACCEL_GATE_LOW_MPS2 || norm > ACCEL_GATE_HIGH_MPS2) return;

  const float measuredRoll = atan2f(ay, az);
  const float measuredPitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);

  scalarStateUpdate(
      0,
      wrapPi(measuredRoll - rollRad),
      sqf(ACCEL_LEVEL_NOISE_RAD),
      25.0f * DEG_TO_RAD_F);
  eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
  scalarStateUpdate(
      1,
      wrapPi(measuredPitch - pitchRad),
      sqf(ACCEL_LEVEL_NOISE_RAD),
      25.0f * DEG_TO_RAD_F);
}

static void gpsToLocalNeu(float positionNeu[3]) {
  static const double EARTH_RADIUS_M = 6378137.0;
  static const double DEG_TO_RAD_D = 0.017453292519943295;
  const double latDeltaRad =
      (gps.latitudeDeg - gpsOriginLatDeg) * DEG_TO_RAD_D;
  const double lonDeltaRad =
      (gps.longitudeDeg - gpsOriginLonDeg) * DEG_TO_RAD_D;
  const double originLatRad = gpsOriginLatDeg * DEG_TO_RAD_D;
  positionNeu[0] = (float)(EARTH_RADIUS_M * latDeltaRad);
  positionNeu[1] =
      (float)(EARTH_RADIUS_M * cos(originLatRad) * lonDeltaRad);
  positionNeu[2] = gps.altitudeM - gpsOriginAltM;
}

static void updateEsekfWithGps() {
  if (!gps.fix || !gps.fresh) return;
  gps.fresh = false;

  if (!gpsOriginSet) {
    if (gps.fixType >= 3 &&
        gps.satellites >= GPS_ORIGIN_MIN_SATS &&
        gps.hAccM <= GPS_ORIGIN_MAX_HACC_M) {
      gpsOriginGoodCount++;
      if (gpsOriginGoodCount >= GPS_ORIGIN_STABLE_FIXES) {
        gpsOriginLatDeg = gps.latitudeDeg;
        gpsOriginLonDeg = gps.longitudeDeg;
        gpsOriginAltM = gps.altitudeM;
        gpsOriginSet = true;
      }
    } else {
      gpsOriginGoodCount = 0;
    }
    return;
  }

  float gpsPosition[3];
  gpsToLocalNeu(gpsPosition);

  const float horizontalStd =
      fmaxf(gps.hAccM, GPS_POSITION_NOISE_FLOOR_M);
  const float verticalStd =
      fmaxf(gps.vAccM, GPS_VERTICAL_NOISE_FLOOR_M);

  const float gpsVelocityNeu[3] = {
    gps.velocityNed[0],
    gps.velocityNed[1],
    -gps.velocityNed[2]
  };
  for (int axis = 0; axis < 3; axis++) {
    scalarStateUpdate(
        axis + 3,
        gpsVelocityNeu[axis] - esekf.v[axis],
        sqf(GPS_VELOCITY_NOISE_MPS),
        25.0f);
  }

  scalarStateUpdate(
      6,
      gpsPosition[0] - esekf.p[0],
      sqf(horizontalStd),
      100.0f);
  scalarStateUpdate(
      7,
      gpsPosition[1] - esekf.p[1],
      sqf(horizontalStd),
      100.0f);
  scalarStateUpdate(
      8,
      gpsPosition[2] - esekf.p[2],
      sqf(verticalStd),
      100.0f);
}

// ========================= Online control-effectiveness estimate =========================

/*
  KEY B-ESTIMATOR NOTE:

  The estimator observes:

    [p_dot]   [B00 B01] [u_diff]
    [q_dot] = [B10 B11] [u_sym ] + matched disturbance

  A normalized LMS update is used only when the previous command provides
  sufficient excitation. Bounds preserve the expected diagonal control signs
  and prevent a noisy estimate from making B singular.

  This is online system identification, not a proof that B is fully observable
  during every maneuver. If both commands remain near zero, B intentionally
  freezes at its last estimate.
*/
static void updateControlEffectiveness(float dt) {
  const float rate[2] = {
    imu.gyro[0] - esekf.bg[0],
    imu.gyro[1] - esekf.bg[1]
  };

  if (!bEstimatorReady || dt <= 0.0f) {
    previousRate[0] = rate[0];
    previousRate[1] = rate[1];
    bEstimatorReady = true;
    return;
  }

  for (int axis = 0; axis < 2; axis++) {
    const float measuredAlpha = clampfLocal(
        (rate[axis] - previousRate[axis]) / dt,
        -ANGULAR_ACCEL_MAX_RADPS2,
        ANGULAR_ACCEL_MAX_RADPS2);
    filteredAngularAccel[axis] +=
        B_ALPHA_LPF * (measuredAlpha - filteredAngularAccel[axis]);
    previousRate[axis] = rate[axis];
  }

  const float excitation =
      previousControlRad[0] * previousControlRad[0] +
      previousControlRad[1] * previousControlRad[1];
  const float minimumExcitation =
      B_EXCITATION_MIN_RAD * B_EXCITATION_MIN_RAD;

  for (int output = 0; output < 2; output++) {
    const float predicted =
        bHat[output][0] * previousControlRad[0] +
        bHat[output][1] * previousControlRad[1] +
        matchedDisturbance[output];
    const float residual = filteredAngularAccel[output] - predicted;

    // Slow matched-disturbance tracking reduces the amount of natural aircraft
    // dynamics incorrectly absorbed into B.
    matchedDisturbance[output] = clampfLocal(
        matchedDisturbance[output] + DISTURBANCE_LPF * residual,
        -DISTURBANCE_MAX_RADPS2,
        DISTURBANCE_MAX_RADPS2);

    if (excitation > minimumExcitation) {
      const float denominator = 1.0e-4f + excitation;
      for (int input = 0; input < 2; input++) {
        bHat[output][input] +=
            B_NLMS_GAIN * residual * previousControlRad[input] / denominator;
      }
    }
  }

  bHat[0][0] = clampfLocal(
      bHat[0][0], -B_DIAG_MAX_ABS, -B_DIAG_MIN_ABS);
  bHat[1][1] = clampfLocal(
      bHat[1][1], B_DIAG_MIN_ABS, B_DIAG_MAX_ABS);
  bHat[0][1] = clampfLocal(
      bHat[0][1], -B_CROSS_MAX_ABS, B_CROSS_MAX_ABS);
  bHat[1][0] = clampfLocal(
      bHat[1][0], -B_CROSS_MAX_ABS, B_CROSS_MAX_ABS);

  if (!isfinite(bHat[0][0]) ||
      !isfinite(bHat[0][1]) ||
      !isfinite(bHat[1][0]) ||
      !isfinite(bHat[1][1])) {
    bHat[0][0] = B_ROLL_INITIAL;
    bHat[0][1] = 0.0f;
    bHat[1][0] = 0.0f;
    bHat[1][1] = B_PITCH_INITIAL;
    matchedDisturbance[0] = 0.0f;
    matchedDisturbance[1] = 0.0f;
    bEstimatorReady = false;
  }
}

// ========================= Sliding mode control =========================

static void updateSmc() {
  eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
  const float pRate = imu.gyro[0] - esekf.bg[0];
  const float qRate = imu.gyro[1] - esekf.bg[1];
  const float rollError =
      deadbandZero(wrapPi(rollRad - ROLL_TARGET_RAD), SMC_ROLL_DEADBAND_RAD);
  const float pitchError =
      deadbandZero(wrapPi(pitchRad - PITCH_TARGET_RAD), SMC_PITCH_DEADBAND_RAD);
  const float pControlRate = deadbandZero(pRate, SMC_RATE_DEADBAND_RADPS);
  const float qControlRate = deadbandZero(qRate, SMC_RATE_DEADBAND_RADPS);

  sRoll = pControlRate + SMC_LAMBDA_ROLL * rollError;
  sPitch = qControlRate + SMC_LAMBDA_PITCH * pitchError;

  // Soft SMC: within the deadband the airplane is allowed to keep its natural
  // glide, matching the 0615 tests where neutral wings flew best.
  const float desiredAlpha[2] = {
    -SMC_LAMBDA_ROLL * pControlRate
        - SMC_K_ROLL_RADPS2 * sat(sRoll / SMC_PHI_ROLL_RADPS)
        - matchedDisturbance[0],
    -SMC_LAMBDA_PITCH * qControlRate
        - SMC_K_PITCH_RADPS2 * sat(sPitch / SMC_PHI_PITCH_RADPS)
        - matchedDisturbance[1]
  };

#if USE_TAIL_PITCH_CONTROL
  // Optional future allocation: wings handle roll, tail handles pitch.
  // Keep disabled until tail pitch sign and effectiveness are measured.
  uDiffRad = desiredAlpha[0] / B_ROLL_INITIAL;
  uSymRad = 0.0f;
  uTailRad = desiredAlpha[1] / B_TAIL_PITCH_INITIAL;
#else
  const float determinant =
      bHat[0][0] * bHat[1][1] - bHat[0][1] * bHat[1][0];
  const float diagProduct = bHat[0][0] * bHat[1][1];
  if (determinant < -0.25f * fabsf(diagProduct)) {
    uDiffRad =
        (bHat[1][1] * desiredAlpha[0] -
         bHat[0][1] * desiredAlpha[1]) / determinant;
    uSymRad =
        (-bHat[1][0] * desiredAlpha[0] +
          bHat[0][0] * desiredAlpha[1]) / determinant;
  } else {
    // Diagonal fallback is deliberately finite and sign-preserving.
    uDiffRad = desiredAlpha[0] / B_ROLL_INITIAL;
    uSymRad = desiredAlpha[1] / B_PITCH_INITIAL;
  }
  uTailRad = 0.0f;
#endif

  const float limitRad = SERVO_CMD_LIMIT_DEG * DEG_TO_RAD_F;
  uDiffRad = clampfLocal(uDiffRad, -limitRad, limitRad);
  uSymRad = clampfLocal(uSymRad, -limitRad, limitRad);
  uTailRad = clampfLocal(
      uTailRad,
      -TAIL_CMD_LIMIT_DEG * DEG_TO_RAD_F,
      TAIL_CMD_LIMIT_DEG * DEG_TO_RAD_F);

  uDiffFilteredRad +=
      SERVO_COMMAND_LPF_ALPHA * (uDiffRad - uDiffFilteredRad);
  uSymFilteredRad +=
      SERVO_COMMAND_LPF_ALPHA * (uSymRad - uSymFilteredRad);
  uTailFilteredRad +=
      TAIL_COMMAND_LPF_ALPHA * (uTailRad - uTailFilteredRad);

}

// ========================= Servo output =========================

static int pwmFromServoDegrees(float degrees, bool reverse) {
  degrees = clampfLocal(degrees, SERVO_MIN_DEG, SERVO_MAX_DEG);
  if (reverse) {
    degrees = SERVO_MIN_DEG + SERVO_MAX_DEG - degrees;
  }
  const float fraction =
      (degrees - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
  return (int)lroundf(
      SERVO_MIN_US + fraction * (SERVO_MAX_US - SERVO_MIN_US));
}

static int servoReverseSign(bool reverse) {
  return reverse ? -1 : 1;
}

static float wingServoFromIncidence(
    float incidenceDeg,
    float neutralDeg,
    float gain,
    bool reverse) {
  return clampfLocal(
      neutralDeg + servoReverseSign(reverse) * gain * incidenceDeg,
      SERVO_MIN_DEG,
      SERVO_MAX_DEG);
}

static float incidenceFromWingServo(
    float servoDeg,
    float neutralDeg,
    float gain,
    bool reverse) {
  if (gain <= 0.0f) return 0.0f;
  return servoReverseSign(reverse) * (servoDeg - neutralDeg) / gain;
}

static void writeMorphServo(float degrees) {
  morphServoDeg = clampfLocal(degrees, SERVO_MIN_DEG, SERVO_MAX_DEG);
  morphPwmUs =
      (uint16_t)pwmFromServoDegrees(morphServoDeg, SERVO_MORPH_REVERSE);
  servoMorph.writeMicroseconds(morphPwmUs);
}

static void writeTailServo(float degrees) {
  tailServoDeg = clampfLocal(degrees, SERVO_MIN_DEG, SERVO_MAX_DEG);
  tailPwmUs =
      (uint16_t)pwmFromServoDegrees(tailServoDeg, SERVO_TAIL_REVERSE);
  servoTail.writeMicroseconds(tailPwmUs);
}

static float clip01(float x) {
  return clampfLocal(x, 0.0f, 1.0f);
}

static float rawAirspeedMeasurement(const VehicleState &state) {
  const float vxAir = state.vxG - state.wx;
  const float vyAir = state.vyG - state.wy;
  const float vzAir = state.vzG - state.wz;
  return sqrtf(vxAir * vxAir + vyAir * vyAir + vzAir * vzAir);
}

static void resetMissionStateEstimator(uint32_t nowUs) {
  missionStateInitialized = true;
  missionStateLastUs = nowUs;
  missionAirspeedEstMps = 0.0f;
  missionAltitudeEstM = RELEASE_ALTITUDE_ESTIMATE_M;
}

static void updateMissionStateEstimator(
    uint32_t nowUs,
    const VehicleState &rawState) {
  if (!missionStateInitialized) {
    resetMissionStateEstimator(nowUs);
  }

  float dt = (nowUs - missionStateLastUs) * 1.0e-6f;
  if (dt <= 0.0f || dt > 0.5f) dt = 1.0f / MISSION_HZ;
  missionStateLastUs = nowUs;

  const float accelModel =
      (flightMode == MODE_FOLDED_ACCELERATION)
          ? ALPHA_V_FOLDED_FALL * G_MPS2
          : 0.0f;
  const float vPrev = missionAirspeedEstMps;
  missionAirspeedEstMps =
      fmaxf(0.0f, missionAirspeedEstMps + accelModel * dt);

  const float descentRateModel =
      (flightMode == MODE_DEPLOYMENT_TRANSIENT)
          ? VZ_12_BAR_MPS
          : vPrev;
  missionAltitudeEstM -=
      fmaxf(0.0f, descentRateModel) * dt +
      0.5f * accelModel * dt * dt;
  missionAltitudeEstM = fmaxf(missionAltitudeEstM, 0.0f);

  const float measuredSpeed = rawAirspeedMeasurement(rawState);
  if (measuredSpeed >= MISSION_MIN_VALID_SPEED_MPS) {
    missionAirspeedEstMps +=
        MISSION_SPEED_MEAS_ALPHA *
        (measuredSpeed - missionAirspeedEstMps);
  }

  if (USE_ESEKF_ALTITUDE_FOR_MISSION && gpsOriginSet) {
    const float measuredAltitude = fmaxf(rawState.h, 0.0f);
    missionAltitudeEstM +=
        MISSION_ALTITUDE_MEAS_ALPHA *
        (measuredAltitude - missionAltitudeEstM);
  }
}

static VehicleState buildVehicleState(float currentTimeS, uint32_t nowUs) {
  VehicleState state = {};
  state.timeS = currentTimeS;
  state.vxG = esekf.v[0];
  state.vyG = esekf.v[1];
  state.vzG = esekf.v[2];
  state.wx = 0.0f;
  state.wy = 0.0f;
  state.wz = 0.0f;
  state.x = esekf.p[0];
  state.y = esekf.p[1];
  state.h = fmaxf(esekf.p[2], 0.0f);
  state.p = imu.gyro[0] - esekf.bg[0];
  state.q = imu.gyro[1] - esekf.bg[1];
  state.r = imu.gyro[2] - esekf.bg[2];
  eulerFromQuaternion(
      esekf.q, state.rollPhi, state.pitchTheta, state.headingPsi);
  state.beta = morphServoDeg;
  updateMissionStateEstimator(nowUs, state);
  state.airspeed = missionAirspeedEstMps;
  state.h = missionAltitudeEstM;
  return state;
}

static void setFlightMode(FlightMode nextMode, uint32_t nowUs) {
  if (nextMode == flightMode) return;
  if (nextMode < flightMode && armed) return;

  flightMode = nextMode;
  modeStartUs = nowUs;
  modeElapsedS = 0.0f;

  if (nextMode == MODE_FOLDED_ACCELERATION) {
    resetMissionStateEstimator(nowUs);
  }

  if (nextMode == MODE_DEPLOYMENT_TRANSIENT ||
      nextMode == MODE_GLIDE_NAVIGATION ||
      nextMode == MODE_AUTOROTATION_TRANSITION) {
    clearControlState();
  }
}

static bool insideAutorotationEntryRegion() {
#if ENABLE_GPS_PHASE_TRANSITIONS
  if (!gps.fix || !gpsOriginSet) return false;
  const float dn = esekf.p[0] - AUTOROTATION_ENTRY_N_M;
  const float de = esekf.p[1] - AUTOROTATION_ENTRY_E_M;
  return (dn * dn + de * de) <=
         (AUTOROTATION_ENTRY_RADIUS_M * AUTOROTATION_ENTRY_RADIUS_M);
#else
  return false;
#endif
}

static void updateAutorotationFilters() {
  const float gyroMag = sqrtf(
      imu.gyro[0] * imu.gyro[0] +
      imu.gyro[1] * imu.gyro[1] +
      imu.gyro[2] * imu.gyro[2]);
  autorotationGyroMagFilt +=
      AUTOROTATION_FILTER_ALPHA_GYRO *
      (gyroMag - autorotationGyroMagFilt);

  if (!gps.fix) return;

  // NEO-6M VELNED uses positive-down vertical speed.
  const float vDown = gps.velocityNed[2];
  const float windApprox = sqrtf(
      gps.velocityNed[0] * gps.velocityNed[0] +
      gps.velocityNed[1] * gps.velocityNed[1]);

  autorotationVDownFilt +=
      AUTOROTATION_FILTER_ALPHA_VD * (vDown - autorotationVDownFilt);
  autorotationWindSpeedFilt +=
      AUTOROTATION_FILTER_ALPHA_WIND *
      (windApprox - autorotationWindSpeedFilt);
}

static float autorotationServo3TargetDeg() {
  const float targetDescent =
      (autorotationWindSpeedFilt > AUTOROTATION_STRONG_WIND_MPS)
          ? AUTOROTATION_FAST_DESCENT_MPS
          : AUTOROTATION_NORMAL_DESCENT_MPS;
  const float descentError = autorotationVDownFilt - targetDescent;

  // Sign must be measured: +1 means increasing servo3 angle should increase
  // descent rate, -1 means increasing angle should slow descent.
  return clampfLocal(
      SERVO_MORPH_AUTOROTATION_BASE_DEG +
          AUTOROTATION_DESCENT_SIGN *
              AUTOROTATION_DESCENT_GAIN_DEG_PER_MPS * descentError,
      SERVO_MORPH_AUTOROTATION_MIN_DEG,
      SERVO_MORPH_AUTOROTATION_MAX_DEG);
}

static void writeAutorotationServos(float dt) {
  // M4/M5 first version: no direction control. Keep both blades parallel and
  // use servo3 only to test descent-rate authority.
  leftServoDeg = rateLimit(
      SERVO_NEUTRAL_DEG,
      leftServoDeg,
      SERVO_RATE_LIMIT_DEG_PER_S * dt);
  rightServoDeg = rateLimit(
      SERVO_NEUTRAL_DEG,
      rightServoDeg,
      SERVO_RATE_LIMIT_DEG_PER_S * dt);
  leftPwmUs = (uint16_t)pwmFromServoDegrees(leftServoDeg, false);
  rightPwmUs = (uint16_t)pwmFromServoDegrees(rightServoDeg, false);
  servoLeft.writeMicroseconds(leftPwmUs);
  servoRight.writeMicroseconds(rightPwmUs);
  writeTailServo(SERVO_TAIL_TRIM_DEG);

  if (flightMode == MODE_AUTOROTATION_TRANSITION) {
    autorotationMorphTargetDeg = SERVO_MORPH_AUTOROTATION_BASE_DEG;
  } else {
    autorotationMorphTargetDeg = autorotationServo3TargetDeg();
  }
  const float morphStep = SERVO_RATE_LIMIT_DEG_PER_S * dt;
  morphServoDeg = rateLimit(
      autorotationMorphTargetDeg,
      morphServoDeg,
      morphStep);
  morphPwmUs =
      (uint16_t)pwmFromServoDegrees(morphServoDeg, SERVO_MORPH_REVERSE);
  servoMorph.writeMicroseconds(morphPwmUs);
}

static void updateFlightMode(uint32_t nowUs) {
  const float currentTimeS = nowUs * 1.0e-6f;

  if (!armed) {
    flightMode = MODE_CARRIED_FOLDED;
    modeStartUs = 0;
    modeElapsedS = 0.0f;
    missionBetaCmdDeg = BETA_FOLDED_DEG;
    missionDeployProgress = 0.0f;
    missionD12Normal = false;
    missionD12Last = false;
    missionD12GroundForce = false;
    missionD23 = false;
    missionStateInitialized = false;
    missionStateLastUs = 0;
    missionAirspeedEstMps = 0.0f;
    missionAltitudeEstM = RELEASE_ALTITUDE_ESTIMATE_M;
    autorotationModeStartMs = 0;
    autorotationEntryConfirmCount = 0;
    autorotationMorphTargetDeg = SERVO_MORPH_AUTOROTATION_BASE_DEG;
    writeMorphServo(BETA_FOLDED_DEG);
    return;
  }

  if (flightMode == MODE_CARRIED_FOLDED || modeStartUs == 0) {
    setFlightMode(MODE_FOLDED_ACCELERATION, nowUs);
  }

  modeElapsedS = (nowUs - modeStartUs) * 1.0e-6f;
  missionState = buildVehicleState(currentTimeS, nowUs);

  switch (flightMode) {
    case MODE_FOLDED_ACCELERATION: {
      // M1 reduced decision state: s1 = {t1, V1, h1}.
      missionD23 = false;
      missionBetaCmdDeg = BETA_FOLDED_DEG;
      missionDeployProgress = 0.0f;
      writeMorphServo(missionBetaCmdDeg);
      writeTailServo(SERVO_TAIL_TRIM_DEG);

      const float t1 = modeElapsedS;
      const float V1 = missionState.airspeed;
      const float h1 = missionState.h;
      const float T12 = T2_OPEN_S + T2_SETTLE_S;
      const float V1Min = V_OPEN_MIN_MPS - AV_12_MPS2 * T12;
      const float h1Min = H_OPEN_MIN_M + VZ_12_BAR_MPS * T12;

      missionD12Normal =
          (t1 >= T1_MIN_S) &&
          (V1 >= V1Min) &&
          (h1 >= h1Min);
      missionD12Last =
          (t1 >= T1_MIN_S) &&
          (h1 <= H1_LAST_M);
      missionD12GroundForce = (h1 <= H_FORCE_DEPLOY_M);

      if (missionD12Normal || missionD12Last || missionD12GroundForce) {
        setFlightMode(MODE_DEPLOYMENT_TRANSIENT, nowUs);
      }
      break;
    }

    case MODE_DEPLOYMENT_TRANSIENT: {
      // M2 time-scheduled deployment. No wing-position feedback is assumed.
      missionD12Normal = false;
      missionD12Last = false;
      missionD12GroundForce = false;
      missionDeployProgress = clip01(modeElapsedS / T2_OPEN_S);
      missionBetaCmdDeg =
          BETA_FOLDED_DEG +
          missionDeployProgress * (BETA_OPEN_DEG - BETA_FOLDED_DEG);
      writeMorphServo(missionBetaCmdDeg);
      writeTailServo(SERVO_TAIL_TRIM_DEG);

      const float t2Min = T2_OPEN_S + T2_SETTLE_S;
      if (HAS_WING_POSITION_FEEDBACK) {
        const float lambda2 =
            clip01((missionState.beta - BETA_FOLDED_DEG) /
                   (BETA_OPEN_DEG - BETA_FOLDED_DEG));
        missionD23 =
            (modeElapsedS >= t2Min) &&
            (lambda2 >= LAMBDA_2_MIN);
      } else {
        missionD23 = (modeElapsedS >= t2Min);
      }

      if (missionD23) {
        missionBetaCmdDeg = BETA_OPEN_DEG;
        missionDeployProgress = 1.0f;
        writeMorphServo(missionBetaCmdDeg);
        // Mode12 test stops here: keep Mode 2 open and do not enter M3.
      }
      break;
    }

    case MODE_GLIDE_NAVIGATION: {
      missionD12Normal = false;
      missionD12Last = false;
      missionD12GroundForce = false;
      missionD23 = false;
      missionBetaCmdDeg = BETA_OPEN_DEG;
      missionDeployProgress = 1.0f;
      writeMorphServo(missionBetaCmdDeg);

#if ENABLE_AUTOROTATION_PHASES && ENABLE_GPS_PHASE_TRANSITIONS
      // Disabled future path: M3->M4 is intentionally not active for the
      // current gliding-focused implementation.
      if (insideAutorotationEntryRegion()) {
        if (autorotationEntryConfirmCount <
            AUTOROTATION_ENTRY_CONFIRM_CYCLES) {
          autorotationEntryConfirmCount++;
        }
      } else {
        autorotationEntryConfirmCount = 0;
      }

      if (autorotationEntryConfirmCount >=
          AUTOROTATION_ENTRY_CONFIRM_CYCLES) {
        setFlightMode(MODE_AUTOROTATION_TRANSITION, nowUs);
        autorotationModeStartMs = millis();
      }
#endif
      break;
    }

#if ENABLE_AUTOROTATION_PHASES
    case MODE_AUTOROTATION_TRANSITION:
      writeMorphServo(SERVO_MORPH_AUTOROTATION_BASE_DEG);
      if ((uint32_t)(millis() - autorotationModeStartMs) >=
          AUTOROTATION_TRANSITION_MS) {
        setFlightMode(MODE_AUTOROTATION_LANDING, nowUs);
        autorotationModeStartMs = millis();
      }
      break;

    case MODE_AUTOROTATION_LANDING:
      break;
#endif

    default:
      break;
  }
}

static bool flightModeAllowsSmc() {
  return flightMode == MODE_GLIDE_NAVIGATION;
}

static void writeServos(float dt) {
#if ENABLE_AUTOROTATION_PHASES
  if (flightMode == MODE_AUTOROTATION_TRANSITION ||
      flightMode == MODE_AUTOROTATION_LANDING) {
    updateAutorotationFilters();
    writeAutorotationServos(dt);
    previousControlRad[0] = 0.0f;
    previousControlRad[1] = 0.0f;
    return;
  }
#endif

  const float desiredLeftIncidence =
      INCIDENCE_TRIM_DEG +
      (uSymFilteredRad + uDiffFilteredRad) * RAD_TO_DEG_F;
  const float desiredRightIncidence =
      INCIDENCE_TRIM_DEG +
      (uSymFilteredRad - uDiffFilteredRad) * RAD_TO_DEG_F;
  const float desiredLeft = wingServoFromIncidence(
      desiredLeftIncidence,
      SERVO_LEFT_NEUTRAL_DEG,
      SERVO_LEFT_GAIN,
      SERVO_LEFT_REVERSE);
  const float desiredRight = wingServoFromIncidence(
      desiredRightIncidence,
      SERVO_RIGHT_NEUTRAL_DEG,
      SERVO_RIGHT_GAIN,
      SERVO_RIGHT_REVERSE);
  const float maxStep = SERVO_RATE_LIMIT_DEG_PER_S * dt;

  leftServoDeg = rateLimit(
      clampfLocal(desiredLeft, SERVO_MIN_DEG, SERVO_MAX_DEG),
      leftServoDeg,
      maxStep);
  rightServoDeg = rateLimit(
      clampfLocal(desiredRight, SERVO_MIN_DEG, SERVO_MAX_DEG),
      rightServoDeg,
      maxStep);

  leftPwmUs = (uint16_t)pwmFromServoDegrees(leftServoDeg, false);
  rightPwmUs = (uint16_t)pwmFromServoDegrees(rightServoDeg, false);
  servoLeft.writeMicroseconds(leftPwmUs);
  servoRight.writeMicroseconds(rightPwmUs);

#if USE_TAIL_PITCH_CONTROL
  const float desiredTail =
      SERVO_TAIL_TRIM_DEG + uTailFilteredRad * RAD_TO_DEG_F;
  tailServoDeg = rateLimit(
      clampfLocal(desiredTail, SERVO_MIN_DEG, SERVO_MAX_DEG),
      tailServoDeg,
      TAIL_RATE_LIMIT_DEG_PER_S * dt);
  tailPwmUs = (uint16_t)pwmFromServoDegrees(tailServoDeg, SERVO_TAIL_REVERSE);
  servoTail.writeMicroseconds(tailPwmUs);
#else
  // Fixed-tail default: the tail changes the passive airframe trim only, not
  // the active SMC loop. This keeps v3 comparable to the neutral-wing tests.
  writeTailServo(SERVO_TAIL_TRIM_DEG);
#endif

  // Feed the B estimator the command that actually passed the output rate
  // limiter, not the unconstrained command requested by the SMC.
  const float leftIncidence = incidenceFromWingServo(
      leftServoDeg, SERVO_LEFT_NEUTRAL_DEG, SERVO_LEFT_GAIN,
      SERVO_LEFT_REVERSE);
  const float rightIncidence = incidenceFromWingServo(
      rightServoDeg, SERVO_RIGHT_NEUTRAL_DEG, SERVO_RIGHT_GAIN,
      SERVO_RIGHT_REVERSE);
  previousControlRad[0] =
      0.5f * (leftIncidence - rightIncidence) * DEG_TO_RAD_F;
  previousControlRad[1] =
      0.5f * (leftIncidence + rightIncidence) * DEG_TO_RAD_F;
}

// ========================= Logger thread =========================

static void writeCsvHeader(Print &out) {
  out.println(F(
      "ms,mode_id,mode_name,control_dt_us,control_exec_us,esekf_us,"
      "missed,log_dropped,"
      "mode_elapsed_s,mission_airspeed_mps,mission_h_m,"
      "mission_beta_cmd_deg,mission_deploy_progress,"
      "mission_d12_normal,mission_d12_last,mission_d12_ground_force,mission_d23,"
      "ax,ay,az,gx,gy,gz,roll_deg,pitch_deg,yaw_deg,"
      "vn,ve,vu,pn,pe,pu,bgx,bgy,bgz,bax,bay,baz,"
      "s_roll,s_pitch,u_diff_deg,u_sym_deg,u_tail_deg,"
      "left_inc_deg,right_inc_deg,"
      "b00,b01,b10,b11,left_deg,right_deg,morph_deg,tail_deg,"
      "auto_vdown_filt,auto_wind_filt,auto_gyro_mag_filt,auto_morph_target_deg,"
      "left_pwm,right_pwm,morph_pwm,tail_pwm,"
      "gps_fix,gps_sats,gps_lat_deg,gps_lon_deg,gps_alt_m,"
      "gps_vn,gps_ve,gps_vd"));
}

static void writeRecord(Print &out, const ControlRecord &r) {
  out.print(r.ms); out.print(',');
  out.print(r.flightMode); out.print(',');
  out.print(flightModeName((FlightMode)r.flightMode)); out.print(',');
  out.print(r.controlDtUs); out.print(',');
  out.print(r.controlExecUs); out.print(',');
  out.print(r.esekfUs); out.print(',');
  out.print(r.missedPeriods); out.print(',');
  out.print(r.logDropped); out.print(',');
  out.print(r.modeElapsedS, 3); out.print(',');
  out.print(r.missionAirspeedMps, 3); out.print(',');
  out.print(r.missionAltitudeM, 3); out.print(',');
  out.print(r.missionBetaCmdDeg, 2); out.print(',');
  out.print(r.missionDeployProgress, 3); out.print(',');
  out.print(r.missionD12Normal); out.print(',');
  out.print(r.missionD12Last); out.print(',');
  out.print(r.missionD12GroundForce); out.print(',');
  out.print(r.missionD23); out.print(',');

  for (int i = 0; i < 3; i++) {
    out.print(r.accel[i], 5); out.print(',');
  }
  for (int i = 0; i < 3; i++) {
    out.print(r.gyro[i], 6); out.print(',');
  }
  out.print(r.rollDeg, 3); out.print(',');
  out.print(r.pitchDeg, 3); out.print(',');
  out.print(r.yawDeg, 3); out.print(',');

  for (int i = 0; i < 3; i++) {
    out.print(r.velocityNeu[i], 4); out.print(',');
  }
  for (int i = 0; i < 3; i++) {
    out.print(r.positionNeu[i], 4); out.print(',');
  }
  for (int i = 0; i < 3; i++) {
    out.print(r.gyroBias[i], 7); out.print(',');
  }
  for (int i = 0; i < 3; i++) {
    out.print(r.accelBias[i], 6); out.print(',');
  }

  out.print(r.sRoll, 5); out.print(',');
  out.print(r.sPitch, 5); out.print(',');
  out.print(r.uDiffDeg, 3); out.print(',');
  out.print(r.uSymDeg, 3); out.print(',');
  out.print(r.uTailDeg, 3); out.print(',');
  out.print(r.leftIncidenceDeg, 3); out.print(',');
  out.print(r.rightIncidenceDeg, 3); out.print(',');
  out.print(r.bMatrix[0][0], 4); out.print(',');
  out.print(r.bMatrix[0][1], 4); out.print(',');
  out.print(r.bMatrix[1][0], 4); out.print(',');
  out.print(r.bMatrix[1][1], 4); out.print(',');
  out.print(r.leftServoDeg, 2); out.print(',');
  out.print(r.rightServoDeg, 2); out.print(',');
  out.print(r.morphServoDeg, 2); out.print(',');
  out.print(r.tailServoDeg, 2); out.print(',');
  out.print(r.autoVDownFilt, 3); out.print(',');
  out.print(r.autoWindSpeedFilt, 3); out.print(',');
  out.print(r.autoGyroMagFilt, 3); out.print(',');
  out.print(r.autoMorphTargetDeg, 2); out.print(',');
  out.print(r.leftPwmUs); out.print(',');
  out.print(r.rightPwmUs); out.print(',');
  out.print(r.morphPwmUs); out.print(',');
  out.print(r.tailPwmUs); out.print(',');
  out.print(r.gpsFix ? 1 : 0); out.print(',');

  if (r.gpsFix) {
    out.print(r.gpsSatellites); out.print(',');
    out.print(r.gpsLatitudeDeg, 7); out.print(',');
    out.print(r.gpsLongitudeDeg, 7); out.print(',');
    out.print(r.gpsAltitudeM, 3); out.print(',');
    out.print(r.gpsVelocityNed[0], 3); out.print(',');
    out.print(r.gpsVelocityNed[1], 3); out.print(',');
    out.println(r.gpsVelocityNed[2], 3);
  } else {
    // No GPS data is represented by empty CSV fields, not fake zeros.
    out.println(F(",,,,,,"));
  }
}

static void printFrequencyStatus(
    uint32_t elapsedMs,
    uint32_t controlDelta,
    uint32_t lastMissed,
    uint32_t lastDropped) {
  if (elapsedMs == 0) return;
  const float hz = controlDelta * 1000.0f / elapsedMs;

  serialMutex.lock();
  Serial.print(F("FREQ,control_hz="));
  Serial.print(hz, 2);
  Serial.print(F(",dt_max_us="));
  Serial.print(maxControlDtUs);
  Serial.print(F(",exec_max_us="));
  Serial.print(maxControlExecUs);
  Serial.print(F(",esekf_max_us="));
  Serial.print(maxEsekfUs);
  Serial.print(F(",missed_delta="));
  Serial.print(missedPeriodCount - lastMissed);
  Serial.print(F(",log_drop_delta="));
  Serial.print(logDroppedCount - lastDropped);
  Serial.print(F(",gps_epochs="));
  Serial.print(gpsEpochCount);
  Serial.print(F(",gps_bytes="));
  Serial.print(gps.bytesSeen ? 1 : 0);
  Serial.print(F(",gps_fix="));
  Serial.print(gps.fix ? 1 : 0);
  Serial.print(F(",esekf_resets="));
  Serial.print(esekfResetCount);
  Serial.print(F(",armed="));
  Serial.print(armed ? 1 : 0);
  Serial.print(F(",mode="));
  Serial.print((uint8_t)flightMode);
  Serial.print(F(":"));
  Serial.print(flightModeName(flightMode));
  Serial.print(F(",imu_fault="));
  Serial.print(imuFaultLocked ? 1 : 0);
  Serial.print(F(",esekf_fault="));
  Serial.println(esekfFaultLocked ? 1 : 0);
  serialMutex.unlock();
}

static void loggerTask() {
  for (uint8_t i = 0; i < QUEUE_DEPTH; i++) {
    freeQueue.put(&recordPool[i], osWaitForever);
  }

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  SPI.begin();
  delay(100);

  bool sdReady = SD.begin(SD_CS_PIN);
  File logFile;
  if (!sdReady) {
    safeSerialPrintln(F("[Core 1] ERROR: SD.begin failed - logging disabled."));
  } else {
    char filename[16];
    for (int i = 0; i < 100; i++) {
      makeLogFilename(filename, sizeof(filename), i);
      if (!SD.exists(filename)) {
        logFile = SD.open(filename, FILE_WRITE);
        break;
      }
    }
    if (logFile) {
      writeCsvHeader(logFile);
      logFile.flush();
      serialMutex.lock();
      Serial.print(F("[Core 1] Logging to: "));
      Serial.println(logFile.name());
      serialMutex.unlock();
    } else {
      sdReady = false;
      safeSerialPrintln(F("[Core 1] ERROR: cannot create log file."));
    }
  }

  uint32_t rowCount = 0;
  uint32_t lastStatusMs = millis();
  uint32_t lastControlCount = controlCount;
  uint32_t lastMissed = missedPeriodCount;
  uint32_t lastDropped = logDroppedCount;

  while (true) {
    osEvent event = filledQueue.get(100);
    if (event.status == osEventMessage) {
      ControlRecord *record =
          reinterpret_cast<ControlRecord *>(event.value.p);

      if (record->kind == RECORD_STOP) {
        if (sdReady && logFile) {
          logFile.print(F("# stop=time_end\n# records="));
          logFile.println(rowCount);
          logFile.flush();
          logFile.close();
          safeSerialPrintln(F("[Core 1] Log file closed (time_end)."));
        }
        freeQueue.put(record, osWaitForever);
        sdReady = false;
      } else {
        if (sdReady && logFile) {
          writeRecord(logFile, *record);
          rowCount++;
          if ((rowCount % SD_FLUSH_EVERY_N) == 0) logFile.flush();
          if (logFile.getWriteError()) {
            logFile.print(F("# stop=sd_write_error\n# records="));
            logFile.println(rowCount);
            logFile.flush();
            logFile.close();
            sdReady = false;
            safeSerialPrintln(F("[Core 1] ERROR: SD write error - logging stopped."));
          }
        }
        freeQueue.put(record, osWaitForever);
      }
    }

    const uint32_t nowMs = millis();
    const uint32_t statusElapsedMs = nowMs - lastStatusMs;
    if (statusElapsedMs >= 1000) {
      const uint32_t currentControlCount = controlCount;
      printFrequencyStatus(
          statusElapsedMs,
          currentControlCount - lastControlCount,
          lastMissed,
          lastDropped);
      lastStatusMs = nowMs;
      lastControlCount = currentControlCount;
      lastMissed = missedPeriodCount;
      lastDropped = logDroppedCount;
    }
  }
}

static ControlRecord makeRecord(
    uint32_t logMs,
    uint32_t controlDtUs,
    uint32_t controlExecUs,
    uint32_t esekfUs) {
  ControlRecord record = {};
  record.kind = RECORD_DATA;
  record.flightMode = (uint8_t)flightMode;
  record.modeElapsedS = modeElapsedS;
  record.missionAirspeedMps = missionState.airspeed;
  record.missionAltitudeM = missionState.h;
  record.missionBetaCmdDeg = missionBetaCmdDeg;
  record.missionDeployProgress = missionDeployProgress;
  record.missionD12Normal = missionD12Normal ? 1 : 0;
  record.missionD12Last = missionD12Last ? 1 : 0;
  record.missionD12GroundForce = missionD12GroundForce ? 1 : 0;
  record.missionD23 = missionD23 ? 1 : 0;
  record.ms = logMs;
  record.controlDtUs = controlDtUs;
  record.controlExecUs = controlExecUs;
  record.esekfUs = esekfUs;
  record.missedPeriods = missedPeriodCount;
  record.logDropped = logDroppedCount;

  for (int i = 0; i < 3; i++) {
    record.accel[i] = imu.accel[i];
    record.gyro[i] = imu.gyro[i];
    record.velocityNeu[i] = esekf.v[i];
    record.positionNeu[i] = esekf.p[i];
    record.gyroBias[i] = esekf.bg[i];
    record.accelBias[i] = esekf.ba[i];
    record.gpsVelocityNed[i] = gps.velocityNed[i];
  }

  record.rollDeg = rollRad * RAD_TO_DEG_F;
  record.pitchDeg = pitchRad * RAD_TO_DEG_F;
  record.yawDeg = yawRad * RAD_TO_DEG_F;
  record.sRoll = sRoll;
  record.sPitch = sPitch;
  record.uDiffDeg = uDiffFilteredRad * RAD_TO_DEG_F;
  record.uSymDeg = uSymFilteredRad * RAD_TO_DEG_F;
  record.uTailDeg = uTailFilteredRad * RAD_TO_DEG_F;
  record.leftIncidenceDeg = incidenceFromWingServo(
      leftServoDeg, SERVO_LEFT_NEUTRAL_DEG, SERVO_LEFT_GAIN,
      SERVO_LEFT_REVERSE);
  record.rightIncidenceDeg = incidenceFromWingServo(
      rightServoDeg, SERVO_RIGHT_NEUTRAL_DEG, SERVO_RIGHT_GAIN,
      SERVO_RIGHT_REVERSE);
  for (int r = 0; r < 2; r++) {
    for (int c = 0; c < 2; c++) record.bMatrix[r][c] = bHat[r][c];
  }
  record.leftServoDeg = leftServoDeg;
  record.rightServoDeg = rightServoDeg;
  record.morphServoDeg = morphServoDeg;
  record.tailServoDeg = tailServoDeg;
  record.autoVDownFilt = autorotationVDownFilt;
  record.autoWindSpeedFilt = autorotationWindSpeedFilt;
  record.autoGyroMagFilt = autorotationGyroMagFilt;
  record.autoMorphTargetDeg = autorotationMorphTargetDeg;
  record.leftPwmUs = leftPwmUs;
  record.rightPwmUs = rightPwmUs;
  record.morphPwmUs = morphPwmUs;
  record.tailPwmUs = tailPwmUs;
  record.gpsFix = gps.fix;
  record.gpsSatellites = gps.satellites;
  record.gpsLatitudeDeg = gps.latitudeDeg;
  record.gpsLongitudeDeg = gps.longitudeDeg;
  record.gpsAltitudeM = gps.altitudeM;
  return record;
}

static bool takeFreeRecord(ControlRecord *&slot) {
  osEvent event = freeQueue.get(0);
  if (event.status != osEventMessage) return false;
  slot = reinterpret_cast<ControlRecord *>(event.value.p);
  return true;
}

static void maybeEnqueueLog(
    uint32_t controlDtUs,
    uint32_t controlExecUs,
    uint32_t esekfUs) {
  if (logWindowDone) {
    if (!logStopQueued) {
      ControlRecord *stopRecord = nullptr;
      if (takeFreeRecord(stopRecord)) {
        *stopRecord = {};
        stopRecord->kind = RECORD_STOP;
        if (filledQueue.put(stopRecord, 0) == osOK) {
          logStopQueued = true;
        } else {
          freeQueue.put(stopRecord, osWaitForever);
        }
      }
    }
    return;
  }

  const uint32_t elapsedMs = millis() - logClockStartMs;
  if (elapsedMs < recordStartMs()) return;
  if (!logWindowOpen) {
    logWindowOpen = true;
    nextLogUs = micros();
  }
  if (elapsedMs >= recordEndMs()) {
    logWindowDone = true;
    safeSerialPrintln(F("[Core 0] Log window closed (time_end)."));
    return;
  }

  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextLogUs) < 0) return;
  while ((int32_t)(nowUs - nextLogUs) >= 0) nextLogUs += SD_LOG_DT_US;

  ControlRecord *slot = nullptr;
  if (!takeFreeRecord(slot)) {
    logDroppedCount++;
    return;
  }

  *slot = makeRecord(
      elapsedMs - recordStartMs(),
      controlDtUs,
      controlExecUs,
      esekfUs);
  if (filledQueue.put(slot, 0) != osOK) {
    logDroppedCount++;
    freeQueue.put(slot, osWaitForever);
  }
}

// ========================= Setup and loop =========================

void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t serialStartMs = millis();
  while (!Serial &&
         (uint32_t)(millis() - serialStartMs) < SERIAL_WAIT_MS) {
    delay(10);
  }

  // Optional GPS: configure a default GY-NEO6MV2 without requiring u-center.
  // If the module is absent, these UART writes are harmless and boot continues.
  Serial1.begin(GPS_BAUD);
  gps.lastFusedTowMs = 0xFFFFFFFFUL;
  delay(300);
  configureNeo6m();

  serialMutex.lock();
  Serial.print(F("[Core 0] 626 Mode12 DeployOnly boot, log tag="));
  Serial.println(LOG_TAG);
  Serial.println(F("[Core 0] CSV naming: MM_DD_ii.CSV when LOG_TAG is MMDD"));
  Serial.print(F("[Core 0] control_hz="));
  Serial.print(CONTROL_HZ, 1);
  Serial.print(F(" sd_log_hz="));
  Serial.print(SD_LOG_HZ, 1);
  Serial.print(F(" adaptive_b="));
  Serial.println(ENABLE_ADAPTIVE_B ? 1 : 0);
  Serial.print(F("[Core 0] soft_smc deadband_deg="));
  Serial.print(SMC_ROLL_DEADBAND_RAD * RAD_TO_DEG_F, 1);
  Serial.print(',');
  Serial.print(SMC_PITCH_DEADBAND_RAD * RAD_TO_DEG_F, 1);
  Serial.print(F(" cmd_limit_deg="));
  Serial.println(SERVO_CMD_LIMIT_DEG, 1);
  Serial.print(F("[Core 0] tail_pitch_control="));
  Serial.println(USE_TAIL_PITCH_CONTROL ? 1 : 0);
  Serial.print(F("[Core 0] mission_hz="));
  Serial.print(MISSION_HZ, 1);
  Serial.print(F(" T2_open_s="));
  Serial.print(T2_OPEN_S, 2);
  Serial.print(F(" T2_settle_s="));
  Serial.println(T2_SETTLE_S, 2);
  Serial.print(F("[Core 0] M1 deploy thresholds t_min,V_open,h_open,h_last,h_force="));
  Serial.print(T1_MIN_S, 2);
  Serial.print(F(","));
  Serial.print(V_OPEN_MIN_MPS, 2);
  Serial.print(F(","));
  Serial.print(H_OPEN_MIN_M, 2);
  Serial.print(F(","));
  Serial.print(H1_LAST_M, 2);
  Serial.print(F(","));
  Serial.println(H_FORCE_DEPLOY_M, 2);
  Serial.print(F("[Core 0] gps_phase_transitions="));
  Serial.print(ENABLE_GPS_PHASE_TRANSITIONS ? 1 : 0);
  Serial.print(F(" autorotation_phases="));
  Serial.println(ENABLE_AUTOROTATION_PHASES ? 1 : 0);
  Serial.println(F(
      "[Core 0] GPS optional: NEO-6M UBX POSLLH/SOL/VELNED, Serial1 9600"));
  Serial.println(F("[Core 0] GPS guidance: log/ESEKF only, no landing target loop"));
  serialMutex.unlock();

  if (!IMU.begin()) {
    safeSerialPrintln(F("[Core 0] FATAL: onboard LSM6DSOX IMU not detected."));
    while (true) delay(1000);
  }

#if !BENCH_MODE
  pinMode(ARM_PIN, INPUT_PULLUP);
#endif
  servoLeft.attach(SERVO_LEFT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoRight.attach(SERVO_RIGHT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoMorph.attach(SERVO_MORPH_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoTail.attach(SERVO_TAIL_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoLeft.writeMicroseconds(
      pwmFromServoDegrees(SERVO_LEFT_NEUTRAL_DEG, false));
  servoRight.writeMicroseconds(
      pwmFromServoDegrees(SERVO_RIGHT_NEUTRAL_DEG, false));
  writeTailServo(SERVO_TAIL_TRIM_DEG);
  writeMorphServo(SERVO_MORPH_STOWED_DEG);

  loggerThread.start(loggerTask);
  delay(200);

  safeSerialPrintln(F("[Core 0] Keep board still: gyro calibration."));
  if (!calibrateGyro()) {
    safeSerialPrintln(F("[Core 0] FATAL: IMU gyro calibration failed."));
    while (true) delay(1000);
  }
  serialMutex.lock();
  Serial.print(F("[Core 0] gyro_bias_radps="));
  Serial.print(gyroCalibration[0], 7);
  Serial.print(',');
  Serial.print(gyroCalibration[1], 7);
  Serial.print(',');
  Serial.println(gyroCalibration[2], 7);
  serialMutex.unlock();

  ImuSample initialSample = {};
  while (!readImu(initialSample)) delay(10);
  resetEsekfFromAccel(initialSample);

  logClockStartMs = millis();
  lastControlUs = micros();
  nextControlUs = lastControlUs + CONTROL_DT_US;
  nextMissionUs = lastControlUs;
  nextLogUs = lastControlUs;
}

void loop() {
  // GPS polling is outside the scheduled control block and never waits.
  pollGps();

  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextControlUs) < 0) return;

  uint32_t elapsedPeriods = 0;
  while ((int32_t)(nowUs - nextControlUs) >= 0) {
    nextControlUs += CONTROL_DT_US;
    elapsedPeriods++;
  }
  if (elapsedPeriods > 1) missedPeriodCount += elapsedPeriods - 1;

  const uint32_t controlDtUs = nowUs - lastControlUs;
  lastControlUs = nowUs;
  if (controlDtUs > maxControlDtUs) maxControlDtUs = controlDtUs;
  float dt = controlDtUs * 1.0e-6f;
  if (dt <= 0.0f || dt > 0.1f) dt = DT_NOMINAL;

  // IMU hard fault: require power cycle.
  if (imuFaultLocked) {
    servoLeft.writeMicroseconds(
        pwmFromServoDegrees(SERVO_LEFT_NEUTRAL_DEG, false));
    servoRight.writeMicroseconds(
        pwmFromServoDegrees(SERVO_RIGHT_NEUTRAL_DEG, false));
    writeTailServo(SERVO_TAIL_TRIM_DEG);
    return;
  }

  // IMU always runs; fault latches permanently.
  const uint32_t controlStartUs = micros();
  if (!readImu(imu)) {
    imuFaultLocked = true;
    armed = false;
    armedSinceMs = 0;
    clearControlState();
    servoLeft.writeMicroseconds(
        pwmFromServoDegrees(SERVO_LEFT_NEUTRAL_DEG, false));
    servoRight.writeMicroseconds(
        pwmFromServoDegrees(SERVO_RIGHT_NEUTRAL_DEG, false));
    writeTailServo(SERVO_TAIL_TRIM_DEG);
    return;
  }
  if (!esekf.initialized) resetEsekfFromAccel(imu);

  // ESEKF always runs regardless of armed state so attitude is fresh at arming.
  const uint32_t esekfStartUs = micros();
  predictEsekf(imu, dt);
  updateEsekfWithAccelerometer(imu);
  updateEsekfWithGps();
  const uint32_t esekfUs = micros() - esekfStartUs;
  if (esekfUs > maxEsekfUs) maxEsekfUs = esekfUs;

  if (esekfHealthy()) {
    if (esekfSettleRemaining > 0) {
      esekfSettleRemaining--;
      eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
      uDiffRad = 0.0f;
      uSymRad = 0.0f;
      uTailRad = 0.0f;
      uDiffFilteredRad = 0.0f;
      uSymFilteredRad = 0.0f;
      uTailFilteredRad = 0.0f;
    } else if (armed &&
               !esekfFaultLocked &&
               flightModeAllowsSmc() &&
               (uint32_t)(millis() - armedSinceMs) >= SMC_ARM_DELAY_MS) {
#if ENABLE_ADAPTIVE_B
      updateControlEffectiveness(dt);
#endif
      updateSmc();
    } else {
      // Disarmed or ESEKF fault locked: track attitude for logging, no control.
      eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
      uDiffRad = 0.0f;
      uSymRad = 0.0f;
      uTailRad = 0.0f;
      uDiffFilteredRad = 0.0f;
      uSymFilteredRad = 0.0f;
      uTailFilteredRad = 0.0f;
      previousControlRad[0] = 0.0f;
      previousControlRad[1] = 0.0f;
    }
  } else {
    esekfFaultLocked = true;
    armed = false;
    armedSinceMs = 0;
    resetEsekfFromAccel(imu);
    eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
    clearControlState();
    esekfSettleRemaining = ESEKF_SETTLE_CYCLES;
    esekfResetCount++;
  }

  // Update arming state; sensing, estimation, and logging continue throughout.
#if BENCH_MODE
  const bool armRequested =
      ENABLE_CONTROL &&
      !imuFaultLocked &&
      !esekfFaultLocked &&
      (uint32_t)(millis() - logClockStartMs) >= ARMING_DELAY_MS;
#else
  const bool armPinLow = digitalRead(ARM_PIN) == LOW;
  if (armPinLow) {
    if (armLowCount < ARM_DEBOUNCE_CYCLES) armLowCount++;
    armHighCount = 0;
  } else {
    if (armHighCount < ARM_DEBOUNCE_CYCLES) armHighCount++;
    armLowCount = 0;
  }

  const bool armPinStableLow = armLowCount >= ARM_DEBOUNCE_CYCLES;
  const bool armPinStableHigh = armHighCount >= ARM_DEBOUNCE_CYCLES;
  if (armPinStableHigh) armInputSeenHigh = true;

  const bool armRequested =
      ENABLE_CONTROL &&
      armInputSeenHigh &&
      armPinStableLow &&
      !imuFaultLocked &&
      !esekfFaultLocked;
#endif

  if (armRequested && !armed) {
    clearControlState();
    armed = true;
    armedSinceMs = millis();
  }
#if BENCH_MODE
  else if (!armRequested && armed) {
#else
  else if (armPinStableHigh && armed) {
#endif
    armed = false;
    armedSinceMs = 0;
    clearControlState();
  }

  const uint32_t missionNowUs = micros();
  if ((int32_t)(missionNowUs - nextMissionUs) >= 0) {
    do {
      nextMissionUs += MISSION_DT_US;
    } while ((int32_t)(missionNowUs - nextMissionUs) >= 0);
    updateFlightMode(missionNowUs);
  }

  // Servo output only when armed and ESEKF is not latched faulty.
  if (armed && !esekfFaultLocked) {
    writeServos(dt);
  } else {
    leftServoDeg = SERVO_LEFT_NEUTRAL_DEG;
    rightServoDeg = SERVO_RIGHT_NEUTRAL_DEG;
    servoLeft.writeMicroseconds(
        pwmFromServoDegrees(SERVO_LEFT_NEUTRAL_DEG, false));
    servoRight.writeMicroseconds(
        pwmFromServoDegrees(SERVO_RIGHT_NEUTRAL_DEG, false));
    writeTailServo(SERVO_TAIL_TRIM_DEG);
  }

  const uint32_t controlExecUs = micros() - controlStartUs;
  if (controlExecUs > maxControlExecUs) maxControlExecUs = controlExecUs;
  controlCount++;

  maybeEnqueueLog(controlDtUs, controlExecUs, esekfUs);
}
