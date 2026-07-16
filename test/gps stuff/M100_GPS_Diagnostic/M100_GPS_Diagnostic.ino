/*
  HGLRC M100 Mini GPS standalone diagnostic
  Target: Arduino Nano RP2040 Connect / the same Serial1 wiring used by
          sweepFilterRot_V2.ino

  What it does:
    - Opens the GPS UART at 115200 baud.
    - Sends one UBX cold-start reset at boot.
    - Re-enables useful UBX diagnostic messages after the receiver restarts.
    - Prints valid UBX packet counts, checksum errors, fix type, satellite count,
      horizontal/vertical accuracy, position, and NAV-SAT signal information.
    - Can optionally print incoming NMEA sentences.

  Serial Monitor:
    - 115200 baud
    - Line ending does not matter for single-letter commands.

  Commands:
    c  = cold-start the GPS again
    r  = resend diagnostic configuration
    v  = poll receiver firmware/version
    n  = toggle raw NMEA printing
    s  = print an immediate summary
    h  = print command help

  Important:
    This is a navigation-data cold start, NOT a destructive factory-settings
    erase. It clears saved orbit/time/position aiding data but preserves the
    receiver's configured UART baud rate.
*/

#include <Arduino.h>

// ---------------- User settings ----------------

static const uint32_t USB_BAUD = 115200;
static const uint32_t GPS_BAUD = 115200;

// Set false to observe the receiver without automatically clearing aiding data.
static const bool AUTO_COLD_START = true;

// Wait this long after CFG-RST before sending configuration messages.
static const uint32_t GPS_RESTART_WAIT_MS = 3000;

// Print a general link/status summary at this interval.
static const uint32_t SUMMARY_INTERVAL_MS = 5000;

// ---------------- UBX parser ----------------

enum UbxState : uint8_t {
  UBX_SYNC_1,
  UBX_SYNC_2,
  UBX_CLASS,
  UBX_ID,
  UBX_LENGTH_1,
  UBX_LENGTH_2,
  UBX_PAYLOAD,
  UBX_CHECKSUM_A,
  UBX_CHECKSUM_B
};

static const uint16_t UBX_MAX_PAYLOAD = 512;

static UbxState ubxState = UBX_SYNC_1;
static uint8_t ubxClass = 0;
static uint8_t ubxId = 0;
static uint16_t ubxLength = 0;
static uint16_t ubxIndex = 0;
static uint8_t ubxPayload[UBX_MAX_PAYLOAD];
static bool ubxPayloadTooLong = false;
static uint8_t calculatedCkA = 0;
static uint8_t calculatedCkB = 0;

static uint32_t totalGpsBytes = 0;
static uint32_t validUbxPackets = 0;
static uint32_t checksumErrors = 0;
static uint32_t oversizedPackets = 0;
static uint32_t nmeaSentenceCount = 0;
static uint32_t lastGpsByteMs = 0;
static uint32_t lastValidUbxMs = 0;

// ---------------- Navigation state ----------------

static bool pvtSeen = false;
static uint32_t lastPvtMs = 0;
static uint32_t pvtITowMs = 0;
static uint8_t fixType = 0;
static uint8_t pvtFlags = 0;
static uint8_t satellites = 0;
static int32_t longitudeRaw = 0;
static int32_t latitudeRaw = 0;
static int32_t altitudeMslMm = 0;
static uint32_t horizontalAccuracyMm = 0xFFFFFFFFUL;
static uint32_t verticalAccuracyMm = 0xFFFFFFFFUL;
static uint32_t speedAccuracyMmPerSec = 0xFFFFFFFFUL;

static bool navSatSeen = false;
static uint32_t lastNavSatMs = 0;
static uint8_t navSatReported = 0;
static uint8_t navSatUsed = 0;
static uint8_t navSatWithSignal = 0;
static uint8_t navSatMaxCno = 0;

static bool navStatusSeen = false;
static uint8_t navStatusFixType = 0;
static uint8_t navStatusFlags = 0;

// ---------------- Runtime state ----------------

static bool printNmea = false;
static bool waitingForGpsRestart = false;
static uint32_t configureAfterMs = 0;
static uint32_t lastSummaryMs = 0;

static char nmeaBuffer[128];
static uint8_t nmeaIndex = 0;
static bool collectingNmea = false;

// ---------------- Binary helpers ----------------

static uint16_t readU16Le(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int16_t readI16Le(const uint8_t *p) {
  return (int16_t)readU16Le(p);
}

static uint32_t readU32Le(const uint8_t *p) {
  return (uint32_t)p[0]
       | ((uint32_t)p[1] << 8)
       | ((uint32_t)p[2] << 16)
       | ((uint32_t)p[3] << 24);
}

static int32_t readI32Le(const uint8_t *p) {
  return (int32_t)readU32Le(p);
}

static void addChecksumByte(uint8_t value) {
  calculatedCkA = (uint8_t)(calculatedCkA + value);
  calculatedCkB = (uint8_t)(calculatedCkB + calculatedCkA);
}

// ---------------- Formatting helpers ----------------

static const __FlashStringHelper *fixTypeName(uint8_t type) {
  switch (type) {
    case 0: return F("NO_FIX");
    case 1: return F("DEAD_RECKONING");
    case 2: return F("2D");
    case 3: return F("3D");
    case 4: return F("GNSS+DR");
    case 5: return F("TIME_ONLY");
    default: return F("UNKNOWN");
  }
}

static void printAccuracyMm(uint32_t valueMm) {
  if (valueMm == 0xFFFFFFFFUL) {
    Serial.print(F("INVALID"));
  } else {
    Serial.print(valueMm * 0.001f, 3);
    Serial.print(F(" m"));
  }
}

static void printSpeedAccuracy(uint32_t valueMmPerSec) {
  if (valueMmPerSec == 0xFFFFFFFFUL) {
    Serial.print(F("INVALID"));
  } else {
    Serial.print(valueMmPerSec * 0.001f, 3);
    Serial.print(F(" m/s"));
  }
}

static void printFixedAscii(const uint8_t *data, uint16_t length) {
  for (uint16_t i = 0; i < length; ++i) {
    const char c = (char)data[i];
    if (c == '\0') break;
    if (c >= 32 && c <= 126) Serial.print(c);
  }
}

// ---------------- UBX transmission ----------------

static void sendUbx(uint8_t messageClass,
                    uint8_t messageId,
                    const uint8_t *payload,
                    uint16_t payloadLength) {
  uint8_t ckA = 0;
  uint8_t ckB = 0;

  const uint8_t header[4] = {
    messageClass,
    messageId,
    (uint8_t)(payloadLength & 0xFFU),
    (uint8_t)(payloadLength >> 8)
  };

  Serial1.write(0xB5);
  Serial1.write(0x62);

  for (uint8_t i = 0; i < sizeof(header); ++i) {
    Serial1.write(header[i]);
    ckA = (uint8_t)(ckA + header[i]);
    ckB = (uint8_t)(ckB + ckA);
  }

  for (uint16_t i = 0; i < payloadLength; ++i) {
    Serial1.write(payload[i]);
    ckA = (uint8_t)(ckA + payload[i]);
    ckB = (uint8_t)(ckB + ckA);
  }

  Serial1.write(ckA);
  Serial1.write(ckB);
  Serial1.flush();
}

static void configureUbxMessage(uint8_t messageClass,
                                uint8_t messageId,
                                uint8_t rate) {
  // Legacy UBX-CFG-MSG three-byte form:
  // msgClass, msgID, rate on the active UART.
  const uint8_t payload[3] = {messageClass, messageId, rate};
  sendUbx(0x06, 0x01, payload, sizeof(payload));
  delay(30);
}

static void configureGpsDiagnostics() {
  Serial.println(F("\n[CMD] Configuring GPS diagnostic output..."));

  // UBX-CFG-RATE: 1000 ms measurement period, one navigation solution per
  // measurement, UTC time reference.
  const uint8_t ratePayload[6] = {
    0xE8, 0x03,  // 1000 ms
    0x01, 0x00,  // navRate = 1
    0x00, 0x00   // timeRef = UTC
  };
  sendUbx(0x06, 0x08, ratePayload, sizeof(ratePayload));
  delay(50);

  configureUbxMessage(0x01, 0x07, 1);  // UBX-NAV-PVT
  configureUbxMessage(0x01, 0x35, 1);  // UBX-NAV-SAT
  configureUbxMessage(0x01, 0x03, 1);  // UBX-NAV-STATUS

  Serial.println(F("[CMD] Configuration sent."));
}

static void pollReceiverVersion() {
  Serial.println(F("[CMD] Polling UBX-MON-VER..."));
  sendUbx(0x0A, 0x04, nullptr, 0);
}

static void clearReportedNavigationState() {
  pvtSeen = false;
  navSatSeen = false;
  navStatusSeen = false;

  fixType = 0;
  pvtFlags = 0;
  satellites = 0;
  horizontalAccuracyMm = 0xFFFFFFFFUL;
  verticalAccuracyMm = 0xFFFFFFFFUL;
  speedAccuracyMmPerSec = 0xFFFFFFFFUL;

  navSatReported = 0;
  navSatUsed = 0;
  navSatWithSignal = 0;
  navSatMaxCno = 0;
}

static void coldStartGps() {
  // UBX-CFG-RST:
  // navBbrMask = 0xFFFF -> clear all navigation backup/aiding data
  // resetMode   = 0x01   -> controlled software reset
  // reserved    = 0x00
  const uint8_t payload[4] = {0xFF, 0xFF, 0x01, 0x00};

  Serial.println(F("\n[CMD] Sending GPS cold-start reset."));
  Serial.println(F("[CMD] Fix acquisition will now begin from scratch."));
  sendUbx(0x06, 0x04, payload, sizeof(payload));

  clearReportedNavigationState();
  waitingForGpsRestart = true;
  configureAfterMs = millis() + GPS_RESTART_WAIT_MS;
}

// ---------------- Packet reporting ----------------

static void printPvtPacket() {
  const bool fixOk = (pvtFlags & 0x01U) != 0;

  Serial.print(F("[PVT] iTOW="));
  Serial.print(pvtITowMs);
  Serial.print(F(" ms, fix="));
  Serial.print(fixOk ? F("YES") : F("NO"));
  Serial.print(F(", type="));
  Serial.print(fixType);
  Serial.print(F(" ("));
  Serial.print(fixTypeName(fixType));
  Serial.print(F("), sats="));
  Serial.print(satellites);
  Serial.print(F(", hAcc="));
  printAccuracyMm(horizontalAccuracyMm);
  Serial.print(F(", vAcc="));
  printAccuracyMm(verticalAccuracyMm);
  Serial.print(F(", sAcc="));
  printSpeedAccuracy(speedAccuracyMmPerSec);
  Serial.println();

  if (fixOk && fixType >= 2) {
    Serial.print(F("      lat="));
    Serial.print(latitudeRaw * 1.0e-7, 7);
    Serial.print(F(", lon="));
    Serial.print(longitudeRaw * 1.0e-7, 7);
    Serial.print(F(", altitudeMSL="));
    Serial.print(altitudeMslMm * 0.001f, 3);
    Serial.println(F(" m"));
  }
}

static void printNavSatPacket() {
  Serial.print(F("[SAT] reported="));
  Serial.print(navSatReported);
  Serial.print(F(", signal>0dBHz="));
  Serial.print(navSatWithSignal);
  Serial.print(F(", usedInFix="));
  Serial.print(navSatUsed);
  Serial.print(F(", strongestCNO="));
  Serial.print(navSatMaxCno);
  Serial.println(F(" dB-Hz"));
}

static void handleUbxPacket() {
  validUbxPackets++;
  lastValidUbxMs = millis();

  if (ubxClass == 0x01 && ubxId == 0x07 && ubxLength >= 92) {
    // UBX-NAV-PVT
    pvtSeen = true;
    lastPvtMs = millis();
    pvtITowMs = readU32Le(&ubxPayload[0]);
    fixType = ubxPayload[20];
    pvtFlags = ubxPayload[21];
    satellites = ubxPayload[23];
    longitudeRaw = readI32Le(&ubxPayload[24]);
    latitudeRaw = readI32Le(&ubxPayload[28]);
    altitudeMslMm = readI32Le(&ubxPayload[36]);
    horizontalAccuracyMm = readU32Le(&ubxPayload[40]);
    verticalAccuracyMm = readU32Le(&ubxPayload[44]);
    speedAccuracyMmPerSec = readU32Le(&ubxPayload[68]);
    printPvtPacket();
    return;
  }

  if (ubxClass == 0x01 && ubxId == 0x35 && ubxLength >= 8) {
    // UBX-NAV-SAT
    navSatSeen = true;
    lastNavSatMs = millis();
    navSatReported = ubxPayload[5];
    navSatUsed = 0;
    navSatWithSignal = 0;
    navSatMaxCno = 0;

    const uint16_t completeBlocks =
        (uint16_t)((ubxLength - 8U) / 12U);
    uint16_t blocksToRead = navSatReported;
    if (blocksToRead > completeBlocks) blocksToRead = completeBlocks;

    for (uint16_t i = 0; i < blocksToRead; ++i) {
      const uint16_t offset = (uint16_t)(8U + 12U * i);
      const uint8_t cno = ubxPayload[offset + 2U];
      const uint32_t flags = readU32Le(&ubxPayload[offset + 8U]);

      if (cno > 0) navSatWithSignal++;
      if (cno > navSatMaxCno) navSatMaxCno = cno;
      if ((flags & (1UL << 3)) != 0) navSatUsed++;
    }

    printNavSatPacket();
    return;
  }

  if (ubxClass == 0x01 && ubxId == 0x03 && ubxLength >= 16) {
    // UBX-NAV-STATUS
    navStatusSeen = true;
    navStatusFixType = ubxPayload[4];
    navStatusFlags = ubxPayload[5];

    Serial.print(F("[STATUS] fixType="));
    Serial.print(navStatusFixType);
    Serial.print(F(" ("));
    Serial.print(fixTypeName(navStatusFixType));
    Serial.print(F("), fixOK="));
    Serial.println((navStatusFlags & 0x01U) ? F("YES") : F("NO"));
    return;
  }

  if (ubxClass == 0x05 && (ubxId == 0x00 || ubxId == 0x01)
      && ubxLength >= 2) {
    Serial.print(F("[ACK] "));
    Serial.print(ubxId == 0x01 ? F("ACK") : F("NAK"));
    Serial.print(F(" for class 0x"));
    if (ubxPayload[0] < 0x10) Serial.print('0');
    Serial.print(ubxPayload[0], HEX);
    Serial.print(F(", id 0x"));
    if (ubxPayload[1] < 0x10) Serial.print('0');
    Serial.println(ubxPayload[1], HEX);
    return;
  }

  if (ubxClass == 0x0A && ubxId == 0x04 && ubxLength >= 40) {
    Serial.print(F("[VERSION] Software: "));
    printFixedAscii(&ubxPayload[0], 30);
    Serial.print(F(" | Hardware: "));
    printFixedAscii(&ubxPayload[30], 10);
    Serial.println();

    for (uint16_t offset = 40; offset + 30 <= ubxLength; offset += 30) {
      Serial.print(F("          Extension: "));
      printFixedAscii(&ubxPayload[offset], 30);
      Serial.println();
    }
    return;
  }
}

// ---------------- Incoming stream processing ----------------

static void processNmeaByte(uint8_t value) {
  if (!collectingNmea) {
    if (value == '$') {
      collectingNmea = true;
      nmeaIndex = 0;
      nmeaBuffer[nmeaIndex++] = '$';
    }
    return;
  }

  if (value == '\n') {
    nmeaBuffer[nmeaIndex] = '\0';
    collectingNmea = false;
    nmeaSentenceCount++;

    if (printNmea) {
      Serial.print(F("[NMEA] "));
      Serial.println(nmeaBuffer);
    }
    return;
  }

  if (value == '\r') return;

  if (value < 32 || value > 126 || nmeaIndex >= sizeof(nmeaBuffer) - 1U) {
    collectingNmea = false;
    nmeaIndex = 0;
    return;
  }

  nmeaBuffer[nmeaIndex++] = (char)value;
}

static void parseUbxByte(uint8_t value) {
  switch (ubxState) {
    case UBX_SYNC_1:
      if (value == 0xB5) ubxState = UBX_SYNC_2;
      break;

    case UBX_SYNC_2:
      ubxState = (value == 0x62) ? UBX_CLASS : UBX_SYNC_1;
      break;

    case UBX_CLASS:
      ubxClass = value;
      calculatedCkA = 0;
      calculatedCkB = 0;
      addChecksumByte(value);
      ubxState = UBX_ID;
      break;

    case UBX_ID:
      ubxId = value;
      addChecksumByte(value);
      ubxState = UBX_LENGTH_1;
      break;

    case UBX_LENGTH_1:
      ubxLength = value;
      addChecksumByte(value);
      ubxState = UBX_LENGTH_2;
      break;

    case UBX_LENGTH_2:
      ubxLength |= (uint16_t)value << 8;
      addChecksumByte(value);
      ubxIndex = 0;
      ubxPayloadTooLong = ubxLength > UBX_MAX_PAYLOAD;
      if (ubxPayloadTooLong) oversizedPackets++;
      ubxState = (ubxLength == 0) ? UBX_CHECKSUM_A : UBX_PAYLOAD;
      break;

    case UBX_PAYLOAD:
      if (ubxIndex < UBX_MAX_PAYLOAD) {
        ubxPayload[ubxIndex] = value;
      }
      ubxIndex++;
      addChecksumByte(value);
      if (ubxIndex >= ubxLength) ubxState = UBX_CHECKSUM_A;
      break;

    case UBX_CHECKSUM_A:
      if (value == calculatedCkA) {
        ubxState = UBX_CHECKSUM_B;
      } else {
        checksumErrors++;
        ubxState = UBX_SYNC_1;
      }
      break;

    case UBX_CHECKSUM_B:
      if (value == calculatedCkB) {
        if (!ubxPayloadTooLong) handleUbxPacket();
      } else {
        checksumErrors++;
      }
      ubxState = UBX_SYNC_1;
      break;
  }
}

static void readGpsStream() {
  while (Serial1.available() > 0) {
    const uint8_t value = (uint8_t)Serial1.read();

    totalGpsBytes++;
    lastGpsByteMs = millis();

    processNmeaByte(value);
    parseUbxByte(value);
  }
}

// ---------------- Human-readable summary ----------------

static void printSummary() {
  const uint32_t now = millis();

  Serial.println(F("\n========== GPS DIAGNOSTIC SUMMARY =========="));
  Serial.print(F("Uptime:                 "));
  Serial.print(now / 1000UL);
  Serial.println(F(" s"));

  Serial.print(F("UART bytes received:    "));
  Serial.println(totalGpsBytes);

  Serial.print(F("Valid UBX packets:      "));
  Serial.println(validUbxPackets);

  Serial.print(F("UBX checksum errors:    "));
  Serial.println(checksumErrors);

  Serial.print(F("Oversized UBX packets:  "));
  Serial.println(oversizedPackets);

  Serial.print(F("NMEA sentences seen:    "));
  Serial.println(nmeaSentenceCount);

  Serial.print(F("Last GPS byte age:      "));
  if (totalGpsBytes == 0) {
    Serial.println(F("NEVER"));
  } else {
    Serial.print(now - lastGpsByteMs);
    Serial.println(F(" ms"));
  }

  Serial.print(F("Last valid UBX age:     "));
  if (validUbxPackets == 0) {
    Serial.println(F("NEVER"));
  } else {
    Serial.print(now - lastValidUbxMs);
    Serial.println(F(" ms"));
  }

  if (pvtSeen) {
    const bool fixOk = (pvtFlags & 0x01U) != 0;

    Serial.print(F("PVT fix:                "));
    Serial.println(fixOk ? F("YES") : F("NO"));

    Serial.print(F("PVT fix type:           "));
    Serial.print(fixType);
    Serial.print(F(" ("));
    Serial.print(fixTypeName(fixType));
    Serial.println(')');

    Serial.print(F("PVT satellites:         "));
    Serial.println(satellites);

    Serial.print(F("Horizontal accuracy:    "));
    printAccuracyMm(horizontalAccuracyMm);
    Serial.println();

    Serial.print(F("Vertical accuracy:      "));
    printAccuracyMm(verticalAccuracyMm);
    Serial.println();

    Serial.print(F("Last PVT age:           "));
    Serial.print(now - lastPvtMs);
    Serial.println(F(" ms"));
  } else {
    Serial.println(F("PVT data:               NOT SEEN"));
  }

  if (navSatSeen) {
    Serial.print(F("NAV-SAT reported:       "));
    Serial.println(navSatReported);

    Serial.print(F("Satellites with signal: "));
    Serial.println(navSatWithSignal);

    Serial.print(F("Satellites used:        "));
    Serial.println(navSatUsed);

    Serial.print(F("Strongest signal:       "));
    Serial.print(navSatMaxCno);
    Serial.println(F(" dB-Hz"));

    Serial.print(F("Last NAV-SAT age:       "));
    Serial.print(now - lastNavSatMs);
    Serial.println(F(" ms"));
  } else {
    Serial.println(F("NAV-SAT data:           NOT SEEN"));
  }

  Serial.println(F("--------------------------------------------"));

  if (totalGpsBytes == 0) {
    Serial.println(F("INTERPRETATION: No UART traffic. Check GPS power,"));
    Serial.println(F("                ground, TX/RX wiring, and baud rate."));
  } else if (validUbxPackets == 0 && nmeaSentenceCount == 0) {
    Serial.println(F("INTERPRETATION: Bytes exist, but no valid UBX or NMEA."));
    Serial.println(F("                Wrong baud rate or corrupted UART data."));
  } else if (pvtSeen && satellites == 0) {
    Serial.println(F("INTERPRETATION: Digital GPS/UART is alive, but the"));
    Serial.println(F("                receiver currently sees zero satellites."));
    Serial.println(F("                Suspect antenna/RF, power noise, or GNSS"));
    Serial.println(F("                configuration rather than TX/RX wiring."));
  } else if (navSatSeen && navSatWithSignal > 0 && navSatUsed == 0) {
    Serial.println(F("INTERPRETATION: Satellites are visible, but none are"));
    Serial.println(F("                usable in a fix yet. Leave it outdoors."));
  } else if (pvtSeen && ((pvtFlags & 0x01U) != 0)) {
    Serial.println(F("INTERPRETATION: GPS has a valid navigation fix."));
  } else {
    Serial.println(F("INTERPRETATION: Receiver is communicating and acquiring."));
  }

  Serial.println(F("============================================\n"));
}

static void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  c  cold-start GPS"));
  Serial.println(F("  r  resend diagnostic configuration"));
  Serial.println(F("  v  poll firmware/version"));
  Serial.println(F("  n  toggle raw NMEA printing"));
  Serial.println(F("  s  print summary now"));
  Serial.println(F("  h  print this help"));
}

static void processUsbCommands() {
  while (Serial.available() > 0) {
    const char command = (char)Serial.read();

    switch (command) {
      case 'c':
      case 'C':
        coldStartGps();
        break;

      case 'r':
      case 'R':
        configureGpsDiagnostics();
        break;

      case 'v':
      case 'V':
        pollReceiverVersion();
        break;

      case 'n':
      case 'N':
        printNmea = !printNmea;
        Serial.print(F("[CMD] Raw NMEA printing "));
        Serial.println(printNmea ? F("ON") : F("OFF"));
        break;

      case 's':
      case 'S':
        printSummary();
        lastSummaryMs = millis();
        break;

      case 'h':
      case 'H':
      case '?':
        printHelp();
        break;

      default:
        // Ignore CR/LF and other accidental characters.
        break;
    }
  }
}

// ---------------- Arduino entry points ----------------

void setup() {
  Serial.begin(USB_BAUD);

  const uint32_t waitStart = millis();
  while (!Serial && (millis() - waitStart < 5000UL)) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("============================================"));
  Serial.println(F(" HGLRC M100 MINI GPS STANDALONE DIAGNOSTIC"));
  Serial.println(F("============================================"));
  Serial.print(F("USB serial baud: "));
  Serial.println(USB_BAUD);
  Serial.print(F("GPS UART baud:   "));
  Serial.println(GPS_BAUD);
  Serial.println(F("Using Serial1 with the board's existing GPS wiring."));
  Serial.println(F("For acquisition testing, place the antenna outdoors"));
  Serial.println(F("with a broad, unobstructed view of the sky."));
  printHelp();

  Serial1.begin(GPS_BAUD);
  delay(500);

  // Discard any partial packet left in the UART during startup.
  while (Serial1.available() > 0) {
    Serial1.read();
  }

  if (AUTO_COLD_START) {
    coldStartGps();
  } else {
    configureGpsDiagnostics();
    pollReceiverVersion();
  }

  lastSummaryMs = millis();
}

void loop() {
  readGpsStream();
  processUsbCommands();

  const uint32_t now = millis();

  if (waitingForGpsRestart
      && (int32_t)(now - configureAfterMs) >= 0) {
    waitingForGpsRestart = false;
    Serial.println(F("[CMD] GPS restart wait complete."));
    configureGpsDiagnostics();
    pollReceiverVersion();
  }

  if ((uint32_t)(now - lastSummaryMs) >= SUMMARY_INTERVAL_MS) {
    lastSummaryMs = now;
    printSummary();
  }
}
