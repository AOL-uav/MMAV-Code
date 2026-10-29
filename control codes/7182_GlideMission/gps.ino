// ========================= GPS: HGLRC M100 MINI (u-blox M10) =========================
//
// The M10 outputs UBX-NAV-PVT (class 0x01, id 0x07, 92-byte payload) only.
// Configuration via UBX-CFG-VALSET; the older CFG-MSG / CFG-RATE messages are
// not supported on the M10.
//
// Wiring:
//   GPS GND → board GND
//   GPS VCC → 3.3–5 V
//   GPS TX  → D1 (Serial1 RX)
//   GPS RX  → D0 (Serial1 TX)
//
// NAV-PVT byte offsets referenced in this file:
//   [0-3]   iTOW (ms)
//   [4-5]   year (U2 LE)        ← date fields
//   [6]     month (U1)
//   [7]     day   (U1)
//   [11]    valid flags: bit0=validDate, bit1=validTime
//   [20]    fixType
//   [21]    flags (bit0=gnssFixOK)
//   [23]    numSV
//   [24-27] lon (1e-7 deg, I4 LE)
//   [28-31] lat (1e-7 deg, I4 LE)
//   [36-39] hMSL (mm, I4 LE)
//   [40-43] hAcc (mm, U4 LE)
//   [44-47] vAcc (mm, U4 LE)
//   [48-51] velN (mm/s, I4 LE)
//   [52-55] velE (mm/s, I4 LE)
//   [56-59] velD (mm/s, I4 LE)
//   [68-71] sAcc (mm/s, U4 LE)  ← speed accuracy for adaptive noise

enum UbxState : uint8_t {
  UBX_SYNC1, UBX_SYNC2, UBX_CLASS, UBX_ID,
  UBX_LEN1, UBX_LEN2, UBX_PAYLOAD, UBX_CK_A, UBX_CK_B
};

static UbxState ubxState     = UBX_SYNC1;
static uint8_t  ubxClass     = 0;
static uint8_t  ubxId        = 0;
static uint16_t ubxLength    = 0;
static uint16_t ubxIndex     = 0;
static uint8_t  ubxChecksumA = 0;
static uint8_t  ubxChecksumB = 0;
static uint8_t  ubxPayload[100];

// ---- UBX low-level helpers ----

static void ubxChecksumAdd(uint8_t v) {
  ubxChecksumA += v;
  ubxChecksumB += ubxChecksumA;
}

static uint32_t readU32Le(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t readI32Le(const uint8_t *p) {
  return (int32_t)readU32Le(p);
}

// ---- NAV-PVT handler ----

static void handleNavPvt() {
  if (ubxLength != 92) return;

  const uint32_t towMs = readU32Le(&ubxPayload[0]);

  // Parse date validity BEFORE the fix check.
  // validDate (byte 11 bit 0) can be set even without a 3D fix —
  // the M10's internal RTC may hold a valid date from a previous session.
  // g_gpsDateValid is written last so Core 1 (logger) never sees a
  // partial month/day pair.
  if (!g_gpsDateValid && (ubxPayload[11] & 0x01U)) {
    const uint8_t m = ubxPayload[6]; // month 1–12
    const uint8_t d = ubxPayload[7]; // day   1–31
    if (m >= 1 && m <= 12 && d >= 1 && d <= 31) {
      gps.month      = m;
      gps.day        = d;
      g_gpsMonth     = m;
      g_gpsDay       = d;
      g_gpsDateValid = true; // commit flag — written last
    }
  }

  gps.fixType    = ubxPayload[20];
  const uint8_t flags = ubxPayload[21];
  gps.satellites = ubxPayload[23];
  gps.fix = ((flags & 0x01U) != 0U) && gps.fixType >= 3;
  if (!gps.fix) { gps.fresh = false; return; }
  if (towMs == gps.lastFusedTowMs) return;

  gps.longitudeDeg   = readI32Le(&ubxPayload[24]) * 1.0e-7;
  gps.latitudeDeg    = readI32Le(&ubxPayload[28]) * 1.0e-7;
  gps.altitudeM      = readI32Le(&ubxPayload[36]) * 1.0e-3f;  // hMSL
  gps.hAccM          = readU32Le(&ubxPayload[40]) * 1.0e-3f;
  gps.vAccM          = readU32Le(&ubxPayload[44]) * 1.0e-3f;
  gps.velocityNed[0] = readI32Le(&ubxPayload[48]) * 1.0e-3f;
  gps.velocityNed[1] = readI32Le(&ubxPayload[52]) * 1.0e-3f;
  gps.velocityNed[2] = readI32Le(&ubxPayload[56]) * 1.0e-3f;
  // sAcc (byte 68): speed accuracy estimate in mm/s.
  // Used as adaptive velocity noise floor in the ESEKF. Tighter in open sky,
  // looser under obstruction — better than a fixed constant.
  gps.sAccMps = readU32Le(&ubxPayload[68]) * 1.0e-3f;

  gps.iTowMs         = towMs;
  gps.lastFusedTowMs = towMs;
  gps.receivedMs     = millis();
  gps.fresh          = true;
  gpsEpochCount++;
}

static void handleUbxPacket() {
  if (ubxClass == 0x01 && ubxId == 0x07) handleNavPvt();
}

// ---- UBX byte-level state machine ----

static void parseUbxByte(uint8_t v) {
  switch (ubxState) {
    case UBX_SYNC1:
      if (v == 0xB5) ubxState = UBX_SYNC2;
      break;
    case UBX_SYNC2:
      ubxState = (v == 0x62) ? UBX_CLASS : UBX_SYNC1;
      break;
    case UBX_CLASS:
      ubxClass = v; ubxChecksumA = 0; ubxChecksumB = 0;
      ubxChecksumAdd(v); ubxState = UBX_ID;
      break;
    case UBX_ID:
      ubxId = v; ubxChecksumAdd(v); ubxState = UBX_LEN1;
      break;
    case UBX_LEN1:
      ubxLength = v; ubxChecksumAdd(v); ubxState = UBX_LEN2;
      break;
    case UBX_LEN2:
      ubxLength |= ((uint16_t)v << 8); ubxChecksumAdd(v); ubxIndex = 0;
      ubxState = (ubxLength > sizeof(ubxPayload)) ? UBX_SYNC1
               : (ubxLength == 0)                  ? UBX_CK_A
               :                                     UBX_PAYLOAD;
      break;
    case UBX_PAYLOAD:
      ubxPayload[ubxIndex++] = v; ubxChecksumAdd(v);
      if (ubxIndex >= ubxLength) ubxState = UBX_CK_A;
      break;
    case UBX_CK_A:
      ubxState = (v == ubxChecksumA) ? UBX_CK_B : UBX_SYNC1;
      break;
    case UBX_CK_B:
      if (v == ubxChecksumB) handleUbxPacket();
      ubxState = UBX_SYNC1;
      break;
  }
}

// ---- UBX transmit helpers ----

static void sendUbx(uint8_t cls, uint8_t id,
                    const uint8_t *payload, uint16_t len) {
  uint8_t ca = 0, cb = 0;
  Serial1.write(0xB5); Serial1.write(0x62);
  const uint8_t hdr[4] = {cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  for (uint8_t i = 0; i < 4; i++) { Serial1.write(hdr[i]); ca += hdr[i]; cb += ca; }
  for (uint16_t i = 0; i < len; i++) { Serial1.write(payload[i]); ca += payload[i]; cb += ca; }
  Serial1.write(ca); Serial1.write(cb);
  Serial1.flush();
}

static void sendValset(const uint8_t *kv, uint16_t kvLen) {
  uint8_t payload[32];
  payload[0] = 0x00; payload[1] = 0x01; // version=0, layer=RAM
  payload[2] = 0x00; payload[3] = 0x00;
  for (uint16_t i = 0; i < kvLen; i++) payload[4 + i] = kv[i];
  sendUbx(0x06, 0x8A, payload, (uint16_t)(4 + kvLen));
  delay(20);
}

// ---- M10 receiver configuration (called once in setup) ----

static void configureM100() {
  // Enable UBX output on UART1; disable NMEA to reduce serial load.
  const uint8_t protocols[] = {
    0x01, 0x00, 0x74, 0x10, 0x01,  // CFG-UART1OUTPROT-UBX  = 1
    0x02, 0x00, 0x74, 0x10, 0x00   // CFG-UART1OUTPROT-NMEA = 0
  };
  sendValset(protocols, sizeof(protocols));

  // Enable NAV-PVT at 1 message per measurement epoch.
  const uint8_t pvtRate[] = {
    0x07, 0x00, 0x91, 0x20, 0x01   // CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1
  };
  sendValset(pvtRate, sizeof(pvtRate));

  // Set measurement rate to GPS_MEASUREMENT_PERIOD_MS (100 ms = 10 Hz).
  const uint8_t measRate[] = {
    0x01, 0x00, 0x21, 0x30,
    (uint8_t)(GPS_MEASUREMENT_PERIOD_MS & 0xFF),
    (uint8_t)(GPS_MEASUREMENT_PERIOD_MS >> 8)
  };
  sendValset(measRate, sizeof(measRate));
}

// ---- Hot-start backup ----
//
// UBX-UPD-SOS (class 0x09, id 0x14), command 0 = create backup.
// Saves the current position, time, and satellite ephemeris into the M10's
// battery-backed RAM (BBR). On the next cold power-cycle the receiver reads
// this snapshot and hot-starts in 5–15 s instead of 2–5 minutes.
//
// Called once by updateEsekfWithGps() immediately after GPS origin is locked,
// ensuring we have a high-quality fix to snapshot.
static void gpsSaveToBackupRam() {
  const uint8_t payload[4] = {0x00, 0x00, 0x00, 0x00}; // cmd=0
  sendUbx(0x09, 0x14, payload, sizeof(payload));
  safeSerialPrintln(F("[GPS] UBX-UPD-SOS backup sent (hot-start next boot)."));
}

// ---- Main GPS poll (called every loop iteration from Core 0) ----

static void pollGps() {
  while (Serial1.available() > 0) {
    gps.bytesSeen = true;
    parseUbxByte((uint8_t)Serial1.read());
  }
  // Invalidate fix if no packet received within GPS_FIX_STALE_MS.
  if (gps.fix && (uint32_t)(millis() - gps.receivedMs) > GPS_FIX_STALE_MS) {
    gps.fix   = false;
    gps.fresh = false;
  }
}
