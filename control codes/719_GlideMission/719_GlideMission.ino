t#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <rtos.h>

// Set 1 to enable M4 (GPS approach region) and M5 (helicopter descent).
// Fill APPROACH_TARGET_N/E_M from mission plan before enabling.
#define ENABLE_APPROACH_PHASES 0

// ========================= Edit each flight =========================
// These are the only values you should need to change between flights.

// Servo positions measured with sweep_aero_logger (deg = (us-500)*180/2000).
// Flat (glide) configuration:
static const float WING_FLAT_LEFT_DEG  = 96.75f;  // 1575 us
static const float WING_FLAT_RIGHT_DEG = 90.00f;  // 1500 us
// Fold configuration: wings biased ±30° from flat to prevent collision
// during morph sweep. Verify clearance on the bench before first flight.
static const float WING_FOLD_LEFT_DEG  = 66.75f;  // flat - 30°
static const float WING_FOLD_RIGHT_DEG = 120.00f; // flat + 30°
// Morph: 0° = folded (500 us), 180° = fully deployed (2500 us).
static const float MORPH_FOLD_DEG    = 0.0f;
static const float MORPH_OPEN_DEG    = 180.0f;
// Tail neutral.
static const float TAIL_HOLD_DEG     = 90.0f;     // 1500 us
// Helicopter/autorotation configuration (M5): wings rotated ~90° in
// opposite directions relative to flat. VERIFY directions on bench.
static const float HELI_LEFT_DEG  = 6.75f;    // flat - 90° → 575 us
static const float HELI_RIGHT_DEG = 180.0f;   // flat + 90° → 2500 us

// M3 PID targets (level glide).
static const float ROLL_TARGET_DEG  = 0.0f;
static const float PITCH_TARGET_DEG = 0.0f;

// M3→M4: target landing point in local NEU metres from GPS origin.
// Leave at 0/0 while ENABLE_APPROACH_PHASES is 0.
static const float APPROACH_TARGET_N_M = 0.0f;
static const float APPROACH_TARGET_E_M = 0.0f;

// Log window: starts at t=0 (end of calibration), closes after RECORD_END_S.
// NOTE: ground wait + drone carry count against this window. 600 s proved
// risky (log could close before release on a slow launch cycle) — 1200 s
// costs ~10 MB of SD and removes the risk.
static const float RECORD_START_S = 0.0f;
static const float RECORD_END_S   = 1200.0f;  // 20-minute cap

// ========================= Mission parameters =========================

// --- M0 → M1: free-fall detection ---
//
// THRESHOLD: In free fall, gravity is cancelled; the accelerometer reads ≈0 m/s².
// 0.3 g (2.94 m/s²) sits above typical drone vibration (< 0.15 g) and below
// any real attitude where gravity contributes at least cos(angle)·g.
// Raise if the carrier drone vibrates more than expected.
static const float FREE_FALL_THRESHOLD_MPS2 = 0.30f * 9.80665f; // 2.94 m/s²

// CONFIRM_FRAMES: 10 frames @ 80 Hz = 125 ms. Long enough to reject a single
// motor pulse; short enough to lose < 8 cm of altitude before detection.
static const uint8_t FREE_FALL_CONFIRM_FRAMES = 10;

// --- M1 → M2 (wing deployment): speed + altitude gates ---
//
// V_OPEN_MIN: minimum ESEKF speed before wings deploy.
// Too low → stall on opening (Reynolds number insufficient for large light wing).
// Raise from first drop tests if the glider stalls.
static const float V_OPEN_MIN_MPS = 8.0f;

// H_OPEN_MIN: minimum AGL altitude at the moment wings open.
// Budget: H_OPEN_MIN > H_FORCE_OPEN_M + VZ_max · T2_OPEN_S ≈ 25 + 15·0.8 = 37 m.
static const float H_OPEN_MIN_M = 40.0f;

// H_FORCE_OPEN: emergency floor — open regardless of speed if never triggered.
// Minimum: 0.5·g·T2_OPEN_S² ≈ 3 m; 25 m gives large safety margin.
static const float H_FORCE_OPEN_M = 25.0f;

// --- M2: wing deployment transient ---
// Wings open in under 1 second; morph is ramped linearly over T2_OPEN_S.
static const float T2_OPEN_S = 0.8f;

// --- M3/M4: GPS navigation (bank-to-turn guidance) ---
//
// Steers toward APPROACH_TARGET_N/E_M by commanding a roll target from the
// heading error (desired bearing - current ground track). Default target is
// 0/0 = the GPS origin, i.e. the power-on location — the glider flies back
// over its launch point unless other coordinates are set.
// Set 0 to disable and hold wings level (straight glide) as before.
#define ENABLE_GPS_NAVIGATION 1
// Roll command per degree of heading error. 0.4 → 40° error saturates the
// bank limit. If the aircraft turns AWAY from the target in flight, the
// servo direction convention is inverted — negate this gain.
static const float NAV_KP_ROLL_PER_HDG_DEG  = 0.4f;
// Bank limit for navigation turns. Conservative for a light glider; raising
// it tightens the turn radius but increases sink rate and stall risk.
static const float NAV_MAX_BANK_DEG         = 15.0f;
// Below this ground speed the track angle from velocity is unreliable
// (atan2 of noise) — hold wings level instead of steering.
static const float NAV_MIN_GROUND_SPEED_MPS = 3.0f;
// Inside this radius stop steering (bearing changes too fast when close);
// hold wings level and let M4 approach logic take over.
static const float NAV_ARRIVAL_RADIUS_M     = 20.0f;

// --- Wind guard (anti-circling in strong headwind) ---
//
// Failure mode without this guard: wind stronger than airspeed blows the
// aircraft backward. Ground track then points AWAY from the target, the
// nav loop sees ~180° heading error and commands a maximum-bank turn.
// But in overwhelming wind no heading makes progress, so the turn never
// closes — the aircraft circles endlessly, sinking faster (banked flight)
// while drifting downwind. Strictly worse than doing nothing.
//
// Guard: if a saturated bank command persists for NAV_STUCK_TURN_MS while
// closing speed on the target stays negative, declare the turn futile and
// hold wings level (minimum sink). Resume navigation automatically if the
// closing speed turns positive (wind eased, or drift rotated us favourably).
static const uint32_t NAV_STUCK_TURN_MS       = 5000;
// Closing speed must exceed this (m/s) to leave the wings-level hold.
// Nonzero so GPS velocity noise cannot flap the state.
static const float    NAV_RESUME_CLOSE_MPS    = 0.5f;

// --- Headwind penetration mode (engaged by the wind guard) ---
//
// When the stuck-turn detector concludes the wind beats our normal glide
// airspeed, pull the morph partially back instead of giving up. Partial
// sweep does two things at once: less wing area raises the trim airspeed,
// and the aft-shifted neutral point pitches the nose down naturally —
// both push airspeed above the wind so ground progress (and steering
// authority over the ground track) come back.
#define ENABLE_PENETRATION_MODE 1
// Servo-space target for the partial sweep, 40° back from fully open.
// PLACEHOLDER GEOMETRY: the servo→wing-sweep mapping depends on the morph
// linkage. Bench-verify that this value gives the intended "tens of
// degrees" of sweep AND that the wing stays stable at that setting.
static const float    PENETRATION_MORPH_DEG     = 140.0f;
// Do not engage below this AGL: penetration raises sink rate, and low down
// the remaining altitude is better spent in minimum-sink level hold.
static const float    PENETRATION_MIN_AGL_M     = 25.0f;
// If after this long in penetration the closing speed is still negative,
// even the raised airspeed cannot beat the wind — fall back to level hold.
static const uint32_t PENETRATION_GIVEUP_MS     = 8000;
// Leave penetration once closing speed is healthy. Deliberately higher than
// NAV_RESUME_CLOSE_MPS: reopening the wings drops airspeed, so exit only
// with margin, or the two states would flap.
static const float    PENETRATION_EXIT_CLOSE_MPS = 2.0f;

// --- M3 → M4: approach zone (only active if ENABLE_APPROACH_PHASES) ---
static const float   APPROACH_RADIUS_M        = 15.0f;
static const uint8_t APPROACH_CONFIRM_CYCLES  = 10;
// Altitude floor for M4: force the approach/heli transition at this AGL
// even if the target zone was never reached — mirrors H_FORCE_OPEN_M in M1.
// Rationale: below ~15 m there is not enough altitude to keep gliding
// toward the target anyway; landing in heli configuration (low vertical
// speed, no horizontal speed) is always safer than gliding into the ground.
// Requires descending (v[2] < 0), same guard as the M1 triggers.
static const float   H_FORCE_APPROACH_M       = 15.0f;

// --- M5: helicopter descent ---
// Morph servo modulates descent rate based on measured vertical speed.
// Positive wind → increase morph angle (more blade area → slower descent).
static const float HELI_MORPH_BASE_DEG     = 90.0f;
static const float HELI_MORPH_MIN_DEG      = 70.0f;
static const float HELI_MORPH_MAX_DEG      = 130.0f;
static const float HELI_NORMAL_DESCENT_MPS = 2.5f;
static const float HELI_FAST_DESCENT_MPS   = 3.5f;
static const float HELI_GAIN_DEG_PER_MPS   = 8.0f;
static const float HELI_VD_FILTER_ALPHA    = 0.10f;

// ========================= Timing and IO =========================

static const uint32_t SERIAL_BAUD            = 115200;
static const uint32_t SERIAL_WAIT_MS         = 1500;
static const uint32_t GPS_BAUD               = 115200;
static const uint16_t GPS_MEASUREMENT_PERIOD_MS = 100; // 10 Hz

static const float    LOOP_HZ    = 80.0f;
static const float    DT_NOMINAL = 1.0f / LOOP_HZ;
static const uint32_t LOOP_DT_US = (uint32_t)(1000000.0f / LOOP_HZ);

static const float    SD_LOG_HZ     = 20.0f;
static const uint32_t SD_LOG_DT_US  = (uint32_t)(1000000.0f / SD_LOG_HZ);
static const int      SD_CS_PIN     = 10;
static const uint8_t  SD_FLUSH_EVERY_N = 10;

static const int   SERVO_LEFT_PIN  = 4;
static const int   SERVO_RIGHT_PIN = 3;
static const int   SERVO_MORPH_PIN = 5;
static const int   SERVO_TAIL_PIN  = 6;
static const int   SERVO_MIN_US    = 500;
static const int   SERVO_MAX_US    = 2500;
static const float SERVO_MIN_DEG   = 0.0f;
static const float SERVO_MAX_DEG   = 180.0f;
// Servo command slew limit. Two purposes:
// (1) mode transitions become controlled ramps instead of step loads on the
//     light airframe; (2) staggers stall-current draw through the TPS61088
//     5 V boost rail — several servos stepping at once caused brownouts on
//     earlier builds (see 708 SD-init notes).
// Must stay above the M2 morph sweep rate (180° / 0.8 s = 225°/s).
static const float SERVO_SLEW_LIMIT_DPS = 300.0f;

static const float G_MPS2       = 9.80665f;
static const float DEG_TO_RAD_F = 0.017453292519943295f;
static const float RAD_TO_DEG_F = 57.29577951308232f;

// ========================= ESEKF configuration =========================
//
// Error-state vector: dx = [dθ(3) dv(3) dp(3) dbg(3) dba(3)]
//   0..2   attitude error (rad)
//   3..5   world NEU velocity error (m/s)
//   6..8   world NEU position error (m)
//   9..11  gyro bias error (rad/s)
//   12..14 accelerometer bias error (m/s²)
static const int ESEKF_N = 15;

static const float GYRO_NOISE_RADPS      = 0.015f;
static const float ACCEL_NOISE_MPS2      = 0.18f;
static const float GYRO_BIAS_RW_RADPS    = 0.0008f;
static const float ACCEL_BIAS_RW_MPS2    = 0.006f;
static const float ACCEL_LEVEL_NOISE_RAD = 3.0f * DEG_TO_RAD_F;

// Accelerometer magnitude gate for attitude correction.
// Outside [0.82g, 1.18g] the aircraft is accelerating and the reading
// no longer represents gravity alone. Free fall and deployment loads
// both fall outside this window automatically.
static const float ACCEL_GATE_LOW_MPS2  = 0.82f * G_MPS2;
static const float ACCEL_GATE_HIGH_MPS2 = 1.18f * G_MPS2;

// Angular rate gate for accel-based attitude leveling.
// Skip when body rotates faster than this — centripetal acceleration
// would corrupt the roll/pitch estimate. 0.25 rad/s ≈ 14°/s.
// Source: Kai Keller version, synthetic validation (roll RMSE 14.9→3.0 deg).
static const float ESEKF_LEVEL_RATE_GATE_RADPS = 0.25f;

static const float    GPS_POSITION_NOISE_FLOOR_M = 1.5f;
static const float    GPS_VERTICAL_NOISE_FLOOR_M  = 2.5f;
// Floor under the M10's reported sAcc (speed accuracy estimate, mm/s).
// Raw sAcc can reach 0.05 m/s in open sky; 0.25 m/s prevents over-weighting.
// During poor visibility sAcc rises automatically, loosening fusion.
static const float    GPS_VELOCITY_NOISE_MPS      = 0.25f;
static const uint32_t GPS_FIX_STALE_MS           = 1500;
static const uint8_t  GPS_ORIGIN_MIN_SATS         = 6;
static const float    GPS_ORIGIN_MAX_HACC_M        = 3.0f;
static const uint8_t  GPS_ORIGIN_STABLE_FIXES      = 3;

static const uint16_t CAL_SAMPLES = 500;
static const uint16_t CAL_DT_MS   = 4;

// ========================= Cascade PID configuration (M3) =========================
//
// Outer loop: angle error (deg) → desired angular rate (deg/s).
// Inner loop: rate error (deg/s) → servo deflection (deg).
// Tuning order: Kp_outer first (roll stiffness), then Kd_inner (rate damping).
static const float KP_OUTER_ROLL  = 1.2f;    // deg/s per deg error
static const float KP_OUTER_PITCH = 1.0f;
// Clamp on the outer-loop rate command. Without this a large angle error
// (e.g. upset) would request an unbounded rate; 60°/s keeps recovery brisk
// but well inside what the inner loop and servos can actually track.
static const float RATE_TARGET_LIMIT_DPS = 60.0f;
static const float KP_INNER_ROLL  = 0.08f;   // servo deg per deg/s error
static const float KP_INNER_PITCH = 0.08f;
static const float KI_INNER_ROLL  = 0.01f;   // integrator: correct steady trim
static const float KI_INNER_PITCH = 0.01f;
// Integrator state limit. The integral accumulates rate error (deg/s) over
// time, so its unit is degrees. Max trim authority = KI_INNER x limit
// = 0.01 x 400 = 4 deg — enough to correct steady trim offsets without
// giving the integrator enough authority to fight the pilot loops.
static const float INTEGRATOR_LIMIT_DEG   = 400.0f;
static const float ANGLE_DEADBAND_DEG     = 2.0f;   // ignore noise below this
static const float RATE_DEADBAND_DPS      = 1.5f;
// Wing servo neutral for PID mixing. Both servos centre at 90° in software;
// mechanical trim offsets (WING_FLAT_*) are used only for fold/deploy logic.
static const float PID_NEUTRAL_DEG        = 90.0f;
// Max deflection from PID_NEUTRAL_DEG each way: range is [60°, 120°].
// 30° gives ~50% more travel than the previous 20° while staying clear of
// mechanical limits (fold positions are ±30° from flat ≈ 67°/120°).
static const float CMD_LIMIT_DEG          = 30.0f;

// ========================= Data types =========================

struct ImuSample {
  float accel[3]; // m/s²
  float gyro[3];  // rad/s
  bool  valid;
};

struct GpsSample {
  bool     bytesSeen;
  bool     fix;
  bool     fresh;
  uint8_t  fixType;
  uint8_t  satellites;
  uint32_t iTowMs;
  uint32_t lastFusedTowMs;
  uint32_t receivedMs;
  double   latitudeDeg;
  double   longitudeDeg;
  float    altitudeM;
  float    hAccM;
  float    vAccM;
  float    sAccMps;      // speed accuracy from NAV-PVT byte 68, adaptive velocity noise
  float    velocityNed[3];
  uint8_t  month;        // 1–12, from NAV-PVT byte 6; 0 until first validDate packet
  uint8_t  day;          // 1–31, from NAV-PVT byte 7; 0 until first validDate packet
};

struct ESEKFState {
  float q[4];
  float v[3];
  float p[3];
  float bg[3];
  float ba[3];
  float P[ESEKF_N][ESEKF_N];
  bool  initialized;
};

// Mission phase identifiers.
enum FlightMode : uint8_t {
  MODE_CARRIED      = 0,  // M0: folded, being carried up by drone
  MODE_FREE_FALL    = 1,  // M1: released, building speed, still folded
  MODE_DEPLOYING    = 2,  // M2: morph sweeping open (< T2_OPEN_S)
  MODE_GLIDE        = 3,  // M3: wings open, cascade PID active
  MODE_APPROACH     = 4,  // M4: near target GPS zone, preparing for heli
  MODE_HELI_DESCENT = 5   // M5: helicopter/autorotation descent
};

enum RecordKind : uint8_t {
  RECORD_DATA = 0,
  RECORD_STOP = 1
};

// Wind-response state machine (see nav constants for the transitions).
enum NavWindState : uint8_t {
  NAV_WIND_NORMAL      = 0,  // normal bank-to-turn navigation
  NAV_WIND_PENETRATION = 1,  // partial sweep engaged, still steering
  NAV_WIND_LEVEL_HOLD  = 2   // turn futile even penetrated: min-sink hold
};

struct ControlRecord {
  uint8_t  kind;
  uint8_t  flightMode;
  uint32_t ms;
  uint32_t loopDtUs;
  uint32_t loopExecUs;
  uint32_t esekfUs;
  uint32_t missedPeriods;
  uint32_t logDropped;
  float    accel[3];
  float    gyro[3];
  float    accelNormMps2;  // raw accelerometer magnitude; verify free-fall detection in post
  float    rollDeg;
  float    pitchDeg;
  float    yawDeg;
  float    velocityNeu[3];
  float    positionNeu[3];
  float    gyroBias[3];
  float    accelBias[3];
  float    rateTargetRoll;    // outer-loop output (deg/s)
  float    rateTargetPitch;
  float    uRollDeg;          // servo command roll
  float    uPitchDeg;         // servo command pitch
  float    navRollCmdDeg;     // nav-commanded roll target (bank-to-turn)
  uint8_t  navWindState;      // 0=normal 1=penetration 2=level-hold
  float    gpsNisPos;         // NIS of last GPS position update (chi2, 3 dof)
  float    gpsNisVel;         // NIS of last GPS velocity update (chi2, 3 dof)
  float    leftServoDeg;
  float    rightServoDeg;
  float    morphServoDeg;
  float    tailServoDeg;
  uint16_t leftPwmUs;
  uint16_t rightPwmUs;
  uint16_t morphPwmUs;
  uint16_t tailPwmUs;
  bool     gpsFix;
  uint8_t  gpsSatellites;
  uint8_t  freeFallCount;  // counter value at log time (debug)
  uint8_t  approachCount;  // M3→M4 confirmation counter
  double   gpsLatitudeDeg;
  double   gpsLongitudeDeg;
  float    gpsAltitudeM;
  float    gpsSAccMps;
  float    gpsVelocityNed[3];
};

// ========================= Non-blocking logger queue =========================

static const uint8_t QUEUE_DEPTH = 24;
static ControlRecord               recordPool[QUEUE_DEPTH];
static rtos::Queue<ControlRecord, QUEUE_DEPTH> filledQueue;
static rtos::Queue<ControlRecord, QUEUE_DEPTH> freeQueue;
static rtos::Thread loggerThread;
static rtos::Mutex  serialMutex;

static void safeSerialPrintln(const __FlashStringHelper *s) {
  serialMutex.lock();
  Serial.println(s);
  serialMutex.unlock();
}

// ========================= Global state =========================

static Servo servoLeft;
static Servo servoRight;
static Servo servoMorph;
static Servo servoTail;

static ImuSample  imu   = {};
static GpsSample  gps   = {};
static ESEKFState esekf = {};

static float   gyroCalibration[3] = {};
static bool    gpsOriginSet       = false;
static double  gpsOriginLatDeg    = 0.0;
static double  gpsOriginLonDeg    = 0.0;
static float   gpsOriginAltM      = 0.0f;
static uint8_t gpsOriginGoodCount = 0;
static bool    gpsSosSent         = false; // true after UBX-UPD-SOS backup is sent

static float rollRad  = 0.0f;
static float pitchRad = 0.0f;
static float yawRad   = 0.0f;

// Mission state
static FlightMode flightMode       = MODE_CARRIED;
static uint32_t   modeStartUs      = 0;
static float      modeElapsedS     = 0.0f;
static uint8_t    freeFallCount    = 0;   // consecutive frames below FREE_FALL_THRESHOLD
static uint8_t    approachCount    = 0;   // consecutive frames inside approach zone
static float      morphDeployStart = MORPH_FOLD_DEG;
static float      heliVdFilt       = 0.0f;

// PID state (M3)
static float pidRateIntRoll     = 0.0f;
static float pidRateIntPitch    = 0.0f;
static float pidRateTargetRoll  = 0.0f;
static float pidRateTargetPitch = 0.0f;
static float uRollDeg           = 0.0f;
static float uPitchDeg          = 0.0f;

// Navigation state exposed for logging (written by navRollTargetDeg()).
// Declared here (not in mission.ino) because logger.ino precedes mission.ino
// in the Arduino build's alphabetical file concatenation.
static float        navRollCmdDeg = 0.0f;
static NavWindState navWindState  = NAV_WIND_NORMAL;

// NIS (Normalized Innovation Squared) of the latest GPS update, written by
// updateEsekfWithGps(). Approximate chi-square with 3 dof (sequential scalar
// updates); consistent filter → mean ≈ 3, 95% of samples below 7.81.
// Sustained values far above that = filter overconfident or GPS degraded.
static float gpsNisPos = 0.0f;
static float gpsNisVel = 0.0f;

// Servo output state
static float    leftServoDeg  = WING_FOLD_LEFT_DEG;
static float    rightServoDeg = WING_FOLD_RIGHT_DEG;
static float    morphServoDeg = MORPH_FOLD_DEG;
static float    tailServoDeg  = TAIL_HOLD_DEG;
static uint16_t leftPwmUs  = 1500;
static uint16_t rightPwmUs = 1500;
static uint16_t morphPwmUs = 500;
static uint16_t tailPwmUs  = 1500;

static bool     imuFaultLocked  = false;
static uint32_t nextLoopUs      = 0;
static uint32_t lastLoopUs      = 0;
static uint32_t nextLogUs       = 0;
static uint32_t logClockStartMs = 0;
static bool     logWindowOpen   = false;
static bool     logWindowDone   = false;
static bool     logStopQueued   = false;

// Performance counters (written by Core 0, readable by Core 1 for status prints).
static volatile uint32_t loopCount         = 0;
static volatile uint32_t missedPeriodCount = 0;
static volatile uint32_t logDroppedCount   = 0;
static volatile uint32_t maxLoopDtUs       = 0;
static volatile uint32_t maxLoopExecUs     = 0;
static volatile uint32_t maxEsekfUs        = 0;
static volatile uint32_t gpsEpochCount     = 0;
static volatile uint32_t esekfResetCount   = 0;

// GPS date shared between Core 0 (writer: handleNavPvt) and Core 1 logger (reader).
// g_gpsDateValid is committed last so Core 1 never reads a half-written month/day pair.
static volatile uint8_t g_gpsMonth     = 0;
static volatile uint8_t g_gpsDay       = 0;
static volatile bool    g_gpsDateValid = false;

// ESEKF work matrices (only used inside predictEsekf; kept global to avoid stack pressure).
static float phiMat[ESEKF_N][ESEKF_N];
static float tempMat[ESEKF_N][ESEKF_N];
static float covarianceNext[ESEKF_N][ESEKF_N];

// ========================= Math helpers =========================

static float sqf(float x) { return x * x; }

static float clampfLocal(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float wrapPi(float x) {
  return x - 2.0f * PI * floorf((x + PI) / (2.0f * PI));
}

static float deadband(float x, float band) {
  if (fabsf(x) <= band) return 0.0f;
  return x > 0.0f ? x - band : x + band;
}

static const char *modeName(FlightMode m) {
  switch (m) {
    case MODE_CARRIED:      return "M0_carried";
    case MODE_FREE_FALL:    return "M1_free_fall";
    case MODE_DEPLOYING:    return "M2_deploying";
    case MODE_GLIDE:        return "M3_glide";
    case MODE_APPROACH:     return "M4_approach";
    case MODE_HELI_DESCENT: return "M5_heli_descent";
    default:                return "unknown";
  }
}

// Filename auto-generated from GPS date (MM_DD_xx.CSV).
// Falls back to NOFIX_xx.CSV if GPS date unavailable within 60 s.
static void makeLogFilename(char *out, size_t sz, int idx) {
  if (g_gpsDateValid) {
    snprintf(out, sz, "%02u_%02u_%02d.CSV",
             (unsigned)g_gpsMonth, (unsigned)g_gpsDay, idx);
  } else {
    snprintf(out, sz, "NOFIX_%02d.CSV", idx);
  }
}

static uint32_t recordStartMs() {
  return (uint32_t)(RECORD_START_S * 1000.0f + 0.5f);
}
static uint32_t recordEndMs() {
  return (uint32_t)(RECORD_END_S * 1000.0f + 0.5f);
}

// ========================= Setup =========================

void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t t0 = millis();
  while (!Serial && (uint32_t)(millis()-t0) < SERIAL_WAIT_MS) delay(10);

  Serial1.begin(GPS_BAUD);
  gps.lastFusedTowMs = 0xFFFFFFFFUL;
  delay(300);
  configureM100();

  serialMutex.lock();
  Serial.println(F("[Core0] 719_GlideMission boot (wind-guard variant)"));
  Serial.println(F("[Core0] SD filename: auto from GPS date (MM_DD_xx.CSV)"));
  Serial.print(F("[Core0] free_fall_threshold_g="));
  Serial.println(FREE_FALL_THRESHOLD_MPS2 / G_MPS2, 2);
  Serial.print(F("[Core0] v_open_min_mps=")); Serial.print(V_OPEN_MIN_MPS, 1);
  Serial.print(F(" h_open_min_m="));          Serial.print(H_OPEN_MIN_M, 1);
  Serial.print(F(" h_force_open_m="));        Serial.println(H_FORCE_OPEN_M, 1);
  Serial.print(F("[Core0] approach_phases=")); Serial.println(ENABLE_APPROACH_PHASES);
  serialMutex.unlock();

  if (!IMU.begin()) {
    safeSerialPrintln(F("[Core0] FATAL: LSM6DSOX not detected."));
    while (true) delay(1000);
  }

  servoLeft.attach(SERVO_LEFT_PIN,  SERVO_MIN_US, SERVO_MAX_US);
  servoRight.attach(SERVO_RIGHT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoMorph.attach(SERVO_MORPH_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoTail.attach(SERVO_TAIL_PIN,  SERVO_MIN_US, SERVO_MAX_US);
  leftServoDeg  = WING_FOLD_LEFT_DEG;
  rightServoDeg = WING_FOLD_RIGHT_DEG;
  morphServoDeg = MORPH_FOLD_DEG;
  tailServoDeg  = TAIL_HOLD_DEG;
  writeAllServos();

  loggerThread.start(loggerTask);
  delay(1500); // let servos settle before still-sensitive gyro calibration

  safeSerialPrintln(F("[Core0] Keep board still: gyro calibration."));
  bool calOk = false;
  for (uint8_t attempt = 0; attempt < 5 && !calOk; attempt++) {
    calOk = calibrateGyro();
    if (!calOk) {
      safeSerialPrintln(F("[Core0] Calibration disturbed — retrying."));
      delay(500);
    }
  }
  if (!calOk) {
    safeSerialPrintln(F("[Core0] FATAL: gyro calibration failed."));
    while (true) delay(1000);
  }

  serialMutex.lock();
  Serial.print(F("[Core0] gyro_bias_radps="));
  Serial.print(gyroCalibration[0], 7); Serial.print(',');
  Serial.print(gyroCalibration[1], 7); Serial.print(',');
  Serial.println(gyroCalibration[2], 7);
  serialMutex.unlock();

  ImuSample init = {};
  while (!readImu(init)) delay(10);
  resetEsekfFromAccel(init);

  flightMode      = MODE_CARRIED;
  modeStartUs     = micros();
  logClockStartMs = millis();
  lastLoopUs      = micros();
  nextLoopUs      = lastLoopUs + LOOP_DT_US;
  nextLogUs       = lastLoopUs;

  safeSerialPrintln(F("[Core0] Ready. Waiting for release (free-fall detection)."));
}

// ========================= Loop =========================

void loop() {
  pollGps();

  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextLoopUs) < 0) return;

  uint32_t elapsed = 0;
  while ((int32_t)(nowUs - nextLoopUs) >= 0) { nextLoopUs += LOOP_DT_US; elapsed++; }
  if (elapsed > 1) missedPeriodCount += elapsed - 1;

  const uint32_t loopDtUs = nowUs - lastLoopUs;
  lastLoopUs = nowUs;
  if (loopDtUs > maxLoopDtUs) maxLoopDtUs = loopDtUs;
  float dt = loopDtUs * 1.0e-6f;
  if (dt <= 0.f || dt > 0.1f) dt = DT_NOMINAL;

  if (imuFaultLocked) {
    // IMU lost: centre wing servos at 90° (symmetric neutral, no active control).
    // Do NOT fold — folding during M3/M4 would immediately crash the aircraft.
    // Morph and tail hold whatever was last commanded before the fault.
    leftServoDeg  = PID_NEUTRAL_DEG;
    rightServoDeg = PID_NEUTRAL_DEG;
    writeAllServos();
    maybeEnqueueLog(0, 0, 0);
    return;
  }

  const uint32_t execStart = micros();
  if (!readImu(imu)) {
    imuFaultLocked = true;
    safeSerialPrintln(F("[Core0] ERROR: IMU read failed."));
    maybeEnqueueLog(0, 0, 0);
    return;
  }

  if (!esekf.initialized) resetEsekfFromAccel(imu);

  const uint32_t esekfStart = micros();
  predictEsekf(imu, dt);
  updateEsekfWithAccelerometer(imu);
  updateEsekfWithGps();
  const uint32_t esekfUs = micros() - esekfStart;
  if (esekfUs > maxEsekfUs) maxEsekfUs = esekfUs;

  if (esekfHealthy()) {
    eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
  } else {
    resetEsekfFromAccel(imu);
    eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
    esekfResetCount++;
  }

  updateMission(dt);

  const uint32_t loopExecUs = micros() - execStart;
  if (loopExecUs > maxLoopExecUs) maxLoopExecUs = loopExecUs;
  loopCount++;

  maybeEnqueueLog(loopDtUs, loopExecUs, esekfUs);
}
