// ========================= Logger thread (Core 1) =========================
//
// Architecture: Core 0 (main loop) fills ControlRecord structs from a free
// pool and posts them to filledQueue. Core 1 drains filledQueue and writes
// CSV rows to the SD card. freeQueue recycles slots back to Core 0.
//
// File naming: waits up to 60 s for Core 0 to deliver a GPS date (month/day)
// via g_gpsDateValid. If GPS provides a date in time, the file is named
// MM_DD_xx.CSV automatically — no manual LOG_TAG needed. If GPS date is not
// available within 60 s, NOFIX_xx.CSV is used as a fallback.
//
// SD writes are slow (~1 ms/row) so logging runs at SD_LOG_HZ (20 Hz)
// while control runs at LOOP_HZ (80 Hz). The queue absorbs the rate mismatch.
// Flush every SD_FLUSH_EVERY_N rows to reduce data loss on crash.

// ---- CSV header ----

static void writeCsvHeader(Print &out) {
  out.println(F(
    "ms,mode_id,mode_name,loop_dt_us,loop_exec_us,esekf_us,"
    "missed,log_dropped,"
    "ax,ay,az,gx,gy,gz,accel_norm_mps2,"
    "roll_deg,pitch_deg,yaw_deg,"
    "vn,ve,vu,pn,pe,pu,bgx,bgy,bgz,bax,bay,baz,"
    "rate_tgt_roll,rate_tgt_pitch,u_roll_deg,u_pitch_deg,"
    "nav_roll_tgt_deg,nav_wind_hold,gps_nis_pos,gps_nis_vel,"
    "left_deg,right_deg,morph_deg,tail_deg,"
    "left_pwm,right_pwm,morph_pwm,tail_pwm,"
    "free_fall_count,approach_count,"
    "gps_fix,gps_sats,gps_lat_deg,gps_lon_deg,gps_alt_m,gps_sacc_mps,"
    "gps_vn,gps_ve,gps_vd"
  ));
}

// ---- CSV row writer ----

static void writeRecord(Print &out, const ControlRecord &r) {
  out.print(r.ms);                out.print(',');
  out.print(r.flightMode);        out.print(',');
  out.print(modeName((FlightMode)r.flightMode)); out.print(',');
  out.print(r.loopDtUs);          out.print(',');
  out.print(r.loopExecUs);        out.print(',');
  out.print(r.esekfUs);           out.print(',');
  out.print(r.missedPeriods);     out.print(',');
  out.print(r.logDropped);        out.print(',');
  for (int i = 0; i < 3; i++) { out.print(r.accel[i], 5); out.print(','); }
  for (int i = 0; i < 3; i++) { out.print(r.gyro[i],  6); out.print(','); }
  out.print(r.accelNormMps2, 4);  out.print(',');
  out.print(r.rollDeg,  3);       out.print(',');
  out.print(r.pitchDeg, 3);       out.print(',');
  out.print(r.yawDeg,   3);       out.print(',');
  for (int i = 0; i < 3; i++) { out.print(r.velocityNeu[i], 4); out.print(','); }
  for (int i = 0; i < 3; i++) { out.print(r.positionNeu[i], 4); out.print(','); }
  for (int i = 0; i < 3; i++) { out.print(r.gyroBias[i],    7); out.print(','); }
  for (int i = 0; i < 3; i++) { out.print(r.accelBias[i],   6); out.print(','); }
  out.print(r.rateTargetRoll,  3); out.print(',');
  out.print(r.rateTargetPitch, 3); out.print(',');
  out.print(r.uRollDeg,   3);      out.print(',');
  out.print(r.uPitchDeg,  3);      out.print(',');
  out.print(r.navRollCmdDeg, 3);   out.print(',');
  out.print(r.navWindHoldFlag);    out.print(',');
  out.print(r.gpsNisPos, 3);       out.print(',');
  out.print(r.gpsNisVel, 3);       out.print(',');
  out.print(r.leftServoDeg,  2);   out.print(',');
  out.print(r.rightServoDeg, 2);   out.print(',');
  out.print(r.morphServoDeg, 2);   out.print(',');
  out.print(r.tailServoDeg,  2);   out.print(',');
  out.print(r.leftPwmUs);          out.print(',');
  out.print(r.rightPwmUs);         out.print(',');
  out.print(r.morphPwmUs);         out.print(',');
  out.print(r.tailPwmUs);          out.print(',');
  out.print(r.freeFallCount);      out.print(',');
  out.print(r.approachCount);      out.print(',');
  out.print(r.gpsFix ? 1 : 0);    out.print(',');
  if (r.gpsFix) {
    out.print(r.gpsSatellites);         out.print(',');
    out.print(r.gpsLatitudeDeg,  7);    out.print(',');
    out.print(r.gpsLongitudeDeg, 7);    out.print(',');
    out.print(r.gpsAltitudeM,    3);    out.print(',');
    out.print(r.gpsSAccMps,      3);    out.print(',');
    out.print(r.gpsVelocityNed[0], 3);  out.print(',');
    out.print(r.gpsVelocityNed[1], 3);  out.print(',');
    out.println(r.gpsVelocityNed[2], 3);
  } else {
    out.println(F(",,,,,,,"));
  }
}

// ---- Periodic serial status (called from Core 1 every 2 s) ----

static void printStatus() {
  serialMutex.lock();
  Serial.print(F("STAT mode="));
  Serial.print(modeName(flightMode));
  Serial.print(F(" agl="));
  Serial.print(esekf.p[2], 1);
  Serial.print(F("m spd="));
  Serial.print(sqrtf(esekf.v[0]*esekf.v[0] +
                     esekf.v[1]*esekf.v[1] +
                     esekf.v[2]*esekf.v[2]), 1);
  Serial.print(F("m/s gps_fix="));
  Serial.print(gps.fix ? 1 : 0);
  Serial.print(F(" ff_cnt="));
  Serial.print(freeFallCount);
  Serial.print(F(" t="));
  Serial.print((millis() - logClockStartMs) * 0.001f, 1);
  Serial.println('s');
  serialMutex.unlock();
}

// ---- Logger thread (Core 1 entry point) ----

static void loggerTask() {
  // Seed the free pool so Core 0 can start queuing records immediately.
  for (uint8_t i = 0; i < QUEUE_DEPTH; i++) freeQueue.put(&recordPool[i], osWaitForever);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  SPI.begin();
  delay(100);

  bool sdReady = false;
  for (int retry = 0; retry < 5 && !sdReady; retry++) {
    sdReady = SD.begin(SD_CS_PIN);
    if (!sdReady) delay(400);
  }

  File logFile;
  if (!sdReady) {
    safeSerialPrintln(F("[Core1] ERROR: SD.begin failed."));
  } else {
    // Wait up to 60 s for Core 0 to deliver a valid GPS date from NAV-PVT.
    // The aircraft is on the ground being carried up; GPS should lock well before launch.
    // If it times out, NOFIX_xx.CSV is used as a fallback — still useful for debugging.
    {
      const uint32_t dateWaitStart = millis();
      while (!g_gpsDateValid && (millis() - dateWaitStart) < 60000U) {
        osDelay(200);
      }
      if (g_gpsDateValid) {
        serialMutex.lock();
        Serial.print(F("[Core1] GPS date: "));
        Serial.print((unsigned)g_gpsMonth);
        Serial.print('/');
        Serial.println((unsigned)g_gpsDay);
        serialMutex.unlock();
      } else {
        safeSerialPrintln(F("[Core1] GPS date timeout — using NOFIX filename."));
      }
    }

    char fn[16];
    for (int i = 0; i < 100; i++) {
      makeLogFilename(fn, sizeof(fn), i);
      if (!SD.exists(fn)) { logFile = SD.open(fn, FILE_WRITE); break; }
    }
    if (logFile) {
      writeCsvHeader(logFile);
      logFile.flush();
      serialMutex.lock();
      Serial.print(F("[Core1] Logging to: "));
      Serial.println(logFile.name());
      serialMutex.unlock();
    } else {
      sdReady = false;
      safeSerialPrintln(F("[Core1] ERROR: cannot create log file."));
    }
  }

  uint32_t rowCount    = 0;
  uint32_t lastStatusMs = millis();

  while (true) {
    osEvent ev = filledQueue.get(100);
    if (ev.status == osEventMessage) {
      ControlRecord *rec = reinterpret_cast<ControlRecord*>(ev.value.p);
      if (rec->kind == RECORD_STOP) {
        if (sdReady && logFile) {
          logFile.print(F("# stop=time_end\n# records="));
          logFile.println(rowCount);
          logFile.flush();
          logFile.close();
          safeSerialPrintln(F("[Core1] Log closed."));
        }
        freeQueue.put(rec, osWaitForever);
        sdReady = false;
      } else {
        if (sdReady && logFile) {
          writeRecord(logFile, *rec);
          rowCount++;
          if ((rowCount % SD_FLUSH_EVERY_N) == 0) logFile.flush();
          if (logFile.getWriteError()) {
            logFile.print(F("# stop=write_error\n# records="));
            logFile.println(rowCount);
            logFile.flush();
            logFile.close();
            sdReady = false;
            safeSerialPrintln(F("[Core1] ERROR: SD write error."));
          }
        }
        freeQueue.put(rec, osWaitForever);
      }
    }
    if ((uint32_t)(millis() - lastStatusMs) >= 2000) {
      printStatus();
      lastStatusMs = millis();
    }
  }
}

// ---- Record builder (called from Core 0) ----

static ControlRecord makeRecord(
    uint32_t logMs, uint32_t loopDtUs,
    uint32_t loopExecUs, uint32_t esekfUs) {
  ControlRecord r = {};
  r.kind          = RECORD_DATA;
  r.flightMode    = (uint8_t)flightMode;
  r.ms            = logMs;
  r.loopDtUs      = loopDtUs;
  r.loopExecUs    = loopExecUs;
  r.esekfUs       = esekfUs;
  r.missedPeriods = missedPeriodCount;
  r.logDropped    = logDroppedCount;

  for (int i = 0; i < 3; i++) {
    r.accel[i]          = imu.accel[i];
    r.gyro[i]           = imu.gyro[i];
    r.velocityNeu[i]    = esekf.v[i];
    r.positionNeu[i]    = esekf.p[i];
    r.gyroBias[i]       = esekf.bg[i];
    r.accelBias[i]      = esekf.ba[i];
    r.gpsVelocityNed[i] = gps.velocityNed[i];
  }
  r.accelNormMps2   = sqrtf(imu.accel[0]*imu.accel[0] +
                             imu.accel[1]*imu.accel[1] +
                             imu.accel[2]*imu.accel[2]);
  r.rollDeg         = rollRad  * RAD_TO_DEG_F;
  r.pitchDeg        = pitchRad * RAD_TO_DEG_F;
  r.yawDeg          = yawRad   * RAD_TO_DEG_F;
  r.rateTargetRoll  = pidRateTargetRoll;
  r.rateTargetPitch = pidRateTargetPitch;
  r.uRollDeg        = uRollDeg;
  r.uPitchDeg       = uPitchDeg;
  r.navRollCmdDeg   = navRollCmdDeg;
  r.navWindHoldFlag = navWindHold ? 1 : 0;
  r.gpsNisPos       = gpsNisPos;
  r.gpsNisVel       = gpsNisVel;
  r.leftServoDeg    = leftServoDeg;
  r.rightServoDeg   = rightServoDeg;
  r.morphServoDeg   = morphServoDeg;
  r.tailServoDeg    = tailServoDeg;
  r.leftPwmUs       = leftPwmUs;
  r.rightPwmUs      = rightPwmUs;
  r.morphPwmUs      = morphPwmUs;
  r.tailPwmUs       = tailPwmUs;
  r.freeFallCount   = freeFallCount;
  r.approachCount   = approachCount;
  r.gpsFix          = gps.fix;
  r.gpsSatellites   = gps.satellites;
  r.gpsLatitudeDeg  = gps.latitudeDeg;
  r.gpsLongitudeDeg = gps.longitudeDeg;
  r.gpsAltitudeM    = gps.altitudeM;
  r.gpsSAccMps      = gps.sAccMps;
  return r;
}

static bool takeFreeRecord(ControlRecord *&slot) {
  osEvent ev = freeQueue.get(0);
  if (ev.status != osEventMessage) return false;
  slot = reinterpret_cast<ControlRecord*>(ev.value.p);
  return true;
}

static void maybeEnqueueLog(
    uint32_t loopDtUs, uint32_t loopExecUs, uint32_t esekfUs) {
  if (logWindowDone) {
    if (!logStopQueued) {
      ControlRecord *sr = nullptr;
      if (takeFreeRecord(sr)) {
        *sr = {}; sr->kind = RECORD_STOP;
        if (filledQueue.put(sr, 0) == osOK) logStopQueued = true;
        else freeQueue.put(sr, osWaitForever);
      }
    }
    return;
  }
  const uint32_t elapsedMs = millis() - logClockStartMs;
  if (elapsedMs < recordStartMs()) return;
  if (!logWindowOpen) { logWindowOpen = true; nextLogUs = micros(); }
  if (elapsedMs >= recordEndMs()) {
    logWindowDone = true;
    safeSerialPrintln(F("[Core0] Log window closed."));
    return;
  }
  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextLogUs) < 0) return;
  while ((int32_t)(nowUs - nextLogUs) >= 0) nextLogUs += SD_LOG_DT_US;

  ControlRecord *slot = nullptr;
  if (!takeFreeRecord(slot)) { logDroppedCount++; return; }
  *slot = makeRecord(elapsedMs - recordStartMs(), loopDtUs, loopExecUs, esekfUs);
  if (filledQueue.put(slot, 0) != osOK) {
    logDroppedCount++;
    freeQueue.put(slot, osWaitForever);
  }
}
