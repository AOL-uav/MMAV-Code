import sys

file_path = "/home/kaikeller/Purdue/AOL/MAV/test/rotational_mode/rotational_mode_test/rotational_mode_test.ino"

with open(file_path, 'r') as f:
    content = f.read()

# 1. Replace loggerTask
start_idx_task = content.find("static void loggerTask() {")
end_idx_task = content.find("static ControlRecord makeRecord(", start_idx_task)

new_logger_task = """static void loggerTask() {
  // Populate the free list so Core 0 can start immediately
  for (uint8_t i = 0; i < QUEUE_DEPTH; i++) {
    freeQueue.put(&recordPool[i], osWaitForever);
  }

  // SD initialisation (runs in Core 1 context)
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  SPI.begin();
  delay(100);

  bool sdReady = SD.begin(SD_CS_PIN);
  File logFile;

  if (!sdReady) {
    safeSerialPrintln(F("[Core 1] ERROR: SD.begin failed - logging disabled."));
  } else {
    char name[16];
    for (int i = 0; i < 100; i++) {
      snprintf(name, sizeof(name), "%s_%02d.CSV", LOG_TAG, i);
      if (!SD.exists(name)) {
        logFile = SD.open(name, FILE_WRITE);
        break;
      }
    }

    if (!logFile) {
      sdReady = false;
      safeSerialPrintln(F("[Core 1] ERROR: cannot create log file."));
    } else {
      writeCsvHeader(logFile);
      logFile.flush();
      serialMutex.lock();
      Serial.print(F("[Core 1] Logging to: "));
      Serial.println(logFile.name());
      serialMutex.unlock();
    }
  }

  uint32_t rowCount = 0;
  bool finished = false;

  // Main logging loop
  while (true) {
    // Block until Core 0 enqueues a filled record pointer.
    osEvent evt = filledQueue.get();

    if (evt.status != osEventMessage) {
      rtos::ThisThread::yield();
      continue;
    }

    ControlRecord *rec = reinterpret_cast<ControlRecord *>(evt.value.p);

    if (sdReady && logFile && !finished) {
      writeRecord(logFile, *rec);
      rowCount++;

      if (rowCount % SD_FLUSH_EVERY_N == 0) {
        logFile.flush();
      }

      if (logFile.getWriteError()) {
        logFile.print(F("# stop=sd_write_error\\n# records="));
        logFile.println(rowCount);
        logFile.flush();
        logFile.close();
        finished = true;
        safeSerialPrintln(F("[Core 1] ERROR: SD write error - logging stopped."));
      }
    }

    // Return the slot to Core 0's free list.
    freeQueue.put(rec, osWaitForever);

    rtos::ThisThread::yield();
  }
}

"""

if start_idx_task != -1 and end_idx_task != -1:
    content = content[:start_idx_task] + new_logger_task + content[end_idx_task:]
else:
    print("Could not find loggerTask")
    sys.exit(1)

# 2. Replace takeFreeRecord and maybeEnqueueLog
start_idx_take = content.find("static bool takeFreeRecord(ControlRecord *&slot) {")
end_idx_take = content.find("void setup() {", start_idx_take)

new_enqueue_logic = """static void maybeEnqueueLog(
    uint32_t controlDtUs, uint32_t controlExecUs, uint32_t esekfUs) {
  if (logWindowDone) return;

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

  osEvent slotEvt = freeQueue.get(0);
  if (slotEvt.status != osEventMessage) return;

  ControlRecord *slot = reinterpret_cast<ControlRecord *>(slotEvt.value.p);
  *slot = makeRecord(elapsedMs - recordStartMs(), controlDtUs, controlExecUs, esekfUs);

  osStatus s = filledQueue.put(slot, 0);
  if (s != osOK) freeQueue.put(slot, osWaitForever);
}

// ========================= Setup and loop =========================

"""

if start_idx_take != -1 and end_idx_take != -1:
    content = content[:start_idx_take] + new_enqueue_logic + content[end_idx_take:]
else:
    print("Could not find takeFreeRecord")
    sys.exit(1)

with open(file_path, 'w') as f:
    f.write(content)

print("Patching complete!")
