#include <Servo.h>
#include <SD.h>
#include <SPI.h>
#include <rtos.h>

static const int SD_CS_PIN     = 10;
static const int SWEEP_PIN     = 6;   // D5  - POSITIONAL sweep servo
static const int AOA_LEFT_PIN  = 4;   // D4  - left  wing AoA servo
static const int AOA_RIGHT_PIN = 3;   // D3  - right wing AoA servo
static const int PIN_MOUNT     = A0;  // mounting position (to GND)
static const int PIN_UNFOLD    = A1;  // unfolds wings (to GND)
static const int PIN_FOLD      = A2;  // folds wings (to GND)

static const int SWEEP_UNFOLDED_US = 2500;
static const int SWEEP_FOLDED_US   = 500;
static const int SWEEP_MOUNT_US    = 700;

static const int AOA_LEFT_FLAT_US  = 1575;
static const int AOA_RIGHT_FLAT_US = 1500;
static const int AOA_LEFT_FOLDED_US  = 1125;
static const int AOA_RIGHT_FOLDED_US = 1050;

Servo sweepServo;
Servo aoaLeft;
Servo aoaRight;

bool isAttached = false;
int currentSweepUs = -1;
int currentAoaLeftUs = -1;
int currentAoaRightUs = -1;

enum State { UNKNOWN, FOLDED, UNFOLDED, MOUNT };
State currentState = UNKNOWN;

// ========================= SD Logging (RTOS based) =========================

struct LogRecord {
  uint32_t ms;
  uint8_t state;
  int sweepUs;
  int aoaLeftUs;
  int aoaRightUs;
};

static const uint8_t QUEUE_DEPTH = 24;
static LogRecord recordPool[QUEUE_DEPTH];
static rtos::Queue<LogRecord, QUEUE_DEPTH> filledQueue;
static rtos::Queue<LogRecord, QUEUE_DEPTH> freeQueue;
static rtos::Thread loggerThread;

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
  if (sdReady) {
    char filename[16];
    for (int i = 0; i < 100; i++) {
      snprintf(filename, sizeof(filename), "WLOG_%02d.CSV", i);
      if (!SD.exists(filename)) {
        logFile = SD.open(filename, FILE_WRITE);
        break;
      }
    }
    if (logFile) {
      logFile.println("time_ms,state,sweep_us,aoa_left_us,aoa_right_us");
      logFile.flush();
    } else {
      sdReady = false;
    }
  }

  uint32_t rowCount = 0;
  while (true) {
    osEvent event = filledQueue.get(100);
    if (event.status == osEventMessage) {
      LogRecord *record = reinterpret_cast<LogRecord *>(event.value.p);
      
      if (sdReady && logFile) {
        logFile.print(record->ms);
        logFile.print(",");
        logFile.print(record->state);
        logFile.print(",");
        logFile.print(record->sweepUs);
        logFile.print(",");
        logFile.print(record->aoaLeftUs);
        logFile.print(",");
        logFile.println(record->aoaRightUs);
        
        rowCount++;
        // Flush periodically so data isn't lost if powered off
        if ((rowCount % 10) == 0) logFile.flush();
      }
      freeQueue.put(record, osWaitForever);
    }
  }
}

void queueLog() {
  osEvent event = freeQueue.get(0);
  if (event.status == osEventMessage) {
    LogRecord *slot = reinterpret_cast<LogRecord *>(event.value.p);
    slot->ms = millis();
    slot->state = (uint8_t)currentState;
    slot->sweepUs = currentSweepUs;
    
    // Fallbacks if not attached yet
    slot->aoaLeftUs = isAttached ? currentAoaLeftUs : 0;
    slot->aoaRightUs = isAttached ? currentAoaRightUs : 0;
    
    if (filledQueue.put(slot, 0) != osOK) {
      freeQueue.put(slot, 0);
    }
  }
}

// ========================= Setup & Control Logic =========================

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_MOUNT, INPUT_PULLUP);
  pinMode(PIN_UNFOLD, INPUT_PULLUP);
  pinMode(PIN_FOLD, INPUT_PULLUP);
  
  currentState = UNKNOWN;
  
  // Start the background SD logger thread
  loggerThread.start(loggerTask);
}

void lazyAttach(int sweepTarget, int aoaLeftTarget, int aoaRightTarget) {
  if (!isAttached) {
     currentSweepUs = sweepTarget;
     sweepServo.writeMicroseconds(currentSweepUs);
     
     currentAoaLeftUs = aoaLeftTarget;
     currentAoaRightUs = aoaRightTarget;
     aoaLeft.writeMicroseconds(currentAoaLeftUs);
     aoaRight.writeMicroseconds(currentAoaRightUs);
     
     sweepServo.attach(SWEEP_PIN, 500, 2500);
     aoaLeft.attach(AOA_LEFT_PIN, 500, 2500);
     aoaRight.attach(AOA_RIGHT_PIN, 500, 2500);
     isAttached = true;
  }
}

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
    
    // Log position during the sweep!
    queueLog();
    delay(20); 
  }
  sweepServo.writeMicroseconds(targetUs);
  currentSweepUs = targetUs;
}

void loop() {
  // 1. Check if Fold is pressed
  if (digitalRead(PIN_FOLD) == LOW && currentState != FOLDED) {
    lazyAttach(SWEEP_FOLDED_US, AOA_LEFT_FOLDED_US, AOA_RIGHT_FOLDED_US); 
    currentAoaLeftUs = AOA_LEFT_FOLDED_US;
    currentAoaRightUs = AOA_RIGHT_FOLDED_US;
    aoaLeft.writeMicroseconds(currentAoaLeftUs);
    aoaRight.writeMicroseconds(currentAoaRightUs);
    
    slowSweepTo(SWEEP_FOLDED_US);
    currentState = FOLDED;
    
  // 2. Check if Unfold is pressed
  } else if (digitalRead(PIN_UNFOLD) == LOW && currentState != UNFOLDED) {
    lazyAttach(SWEEP_UNFOLDED_US, AOA_LEFT_FLAT_US, AOA_RIGHT_FLAT_US); 
    slowSweepTo(SWEEP_UNFOLDED_US);
    
    currentAoaLeftUs = AOA_LEFT_FLAT_US;
    currentAoaRightUs = AOA_RIGHT_FLAT_US;
    aoaLeft.writeMicroseconds(currentAoaLeftUs);
    aoaRight.writeMicroseconds(currentAoaRightUs);
    currentState = UNFOLDED;

  // 3. Check if Mount is pressed
  } else if (digitalRead(PIN_MOUNT) == LOW && currentState != MOUNT) {
    lazyAttach(SWEEP_MOUNT_US, AOA_LEFT_FOLDED_US, AOA_RIGHT_FOLDED_US); 
    currentAoaLeftUs = AOA_LEFT_FOLDED_US;
    currentAoaRightUs = AOA_RIGHT_FOLDED_US;
    aoaLeft.writeMicroseconds(currentAoaLeftUs);
    aoaRight.writeMicroseconds(currentAoaRightUs);
    
    slowSweepTo(SWEEP_MOUNT_US);
    currentState = MOUNT;
  }
  
  // Log current stable state
  queueLog();
  delay(20);
}
