#include <Arduino.h>

static const uint32_t USB_BAUD = 115200;
static const uint32_t GPS_BAUD = 115200;

// ========================= UBX GPS parser =========================
struct GpsSample {
  bool bytesSeen;
  bool fix;
  bool fresh;
  uint8_t fixType;
  uint8_t satellites;
  uint32_t iTowMs;
  uint32_t receivedMs;
  double latitudeDeg;
  double longitudeDeg;
  float altitudeM;
  float hAccM;
  float vAccM;
  float sAccMps;
  float velocityNed[3];
};

static GpsSample gps = {};

enum UbxState : uint8_t { UBX_SYNC1, UBX_SYNC2, UBX_CLASS, UBX_ID, UBX_LEN1,
                          UBX_LEN2, UBX_PAYLOAD, UBX_CK_A, UBX_CK_B };
static UbxState ubxState = UBX_SYNC1;
static uint8_t ubxClass = 0, ubxId = 0, ubxChecksumA = 0, ubxChecksumB = 0;
static uint16_t ubxLength = 0, ubxIndex = 0;
static uint8_t ubxPayload[100];
static uint32_t gpsEpochCount = 0;

static uint32_t readU32Le(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t readI32Le(const uint8_t *p) { return (int32_t)readU32Le(p); }
static void ubxChecksumAdd(uint8_t value) {
  ubxChecksumA += value;
  ubxChecksumB += ubxChecksumA;
}

static void handleUbxPacket() {
  if (ubxClass != 0x01) return;
  
  if (ubxId == 0x07 && ubxLength == 92) {  // NAV-PVT
    gps.iTowMs = readU32Le(&ubxPayload[0]);
    gps.fixType = ubxPayload[20];
    gps.satellites = ubxPayload[23];
    gps.fix = (ubxPayload[21] & 0x01U) && gps.fixType >= 3;
    gps.longitudeDeg = readI32Le(&ubxPayload[24]) * 1.0e-7;
    gps.latitudeDeg = readI32Le(&ubxPayload[28]) * 1.0e-7;
    gps.altitudeM = readI32Le(&ubxPayload[36]) * 1.0e-3f;
    gps.hAccM = readU32Le(&ubxPayload[40]) * 1.0e-3f;
    gps.vAccM = readU32Le(&ubxPayload[44]) * 1.0e-3f;
    gps.velocityNed[0] = readI32Le(&ubxPayload[48]) * 1.0e-3f;
    gps.velocityNed[1] = readI32Le(&ubxPayload[52]) * 1.0e-3f;
    gps.velocityNed[2] = readI32Le(&ubxPayload[56]) * 1.0e-3f;
    gps.sAccMps = readU32Le(&ubxPayload[68]) * 1.0e-3f;
    
    gps.receivedMs = millis();
    gps.fresh = true;
    gpsEpochCount++;
  }
}

static void parseUbxByte(uint8_t value) {
  switch (ubxState) {
    case UBX_SYNC1: ubxState = value == 0xB5 ? UBX_SYNC2 : UBX_SYNC1; break;
    case UBX_SYNC2: ubxState = value == 0x62 ? UBX_CLASS : UBX_SYNC1; break;
    case UBX_CLASS: ubxClass = value; ubxChecksumA = ubxChecksumB = 0; ubxChecksumAdd(value); ubxState = UBX_ID; break;
    case UBX_ID: ubxId = value; ubxChecksumAdd(value); ubxState = UBX_LEN1; break;
    case UBX_LEN1: ubxLength = value; ubxChecksumAdd(value); ubxState = UBX_LEN2; break;
    case UBX_LEN2: ubxLength |= (uint16_t)value << 8; ubxChecksumAdd(value); ubxIndex = 0; ubxState = ubxLength > sizeof(ubxPayload) ? UBX_SYNC1 : (ubxLength ? UBX_PAYLOAD : UBX_CK_A); break;
    case UBX_PAYLOAD: ubxPayload[ubxIndex++] = value; ubxChecksumAdd(value); if (ubxIndex >= ubxLength) ubxState = UBX_CK_A; break;
    case UBX_CK_A: ubxState = value == ubxChecksumA ? UBX_CK_B : UBX_SYNC1; break;
    case UBX_CK_B: if (value == ubxChecksumB) handleUbxPacket(); ubxState = UBX_SYNC1; break;
  }
}

static void sendUbx(uint8_t cls, uint8_t id, const uint8_t *data, uint16_t length) {
  uint8_t ckA = 0, ckB = 0;
  const uint8_t header[4] = {cls, id, (uint8_t)length, (uint8_t)(length >> 8)};
  Serial1.write(0xB5); Serial1.write(0x62);
  for (uint8_t value : header) { Serial1.write(value); ckA += value; ckB += ckA; }
  for (uint16_t i = 0; i < length; ++i) { Serial1.write(data[i]); ckA += data[i]; ckB += ckA; }
  Serial1.write(ckA); Serial1.write(ckB); Serial1.flush();
}

static void configureUbxMessage(uint8_t cls, uint8_t id, uint8_t rate) {
  const uint8_t data[3] = {cls, id, rate};
  sendUbx(0x06, 0x01, data, sizeof(data));
  delay(15);
}

static void configureUbxReceiver() {
  Serial.println(F("Configuring GPS messages..."));
  // UBX-CFG-RATE: 1000 ms measurement period
  const uint8_t rate[6] = {0xE8, 0x03, 1, 0, 1, 0};
  sendUbx(0x06, 0x08, rate, sizeof(rate));
  delay(30);
  
  // Enable NAV-PVT
  configureUbxMessage(0x01, 0x07, 1);
  Serial.println(F("Configuration sent."));
}

static void coldRestartGps() {
  Serial.println(F("Sending GPS cold-start reset..."));
  // UBX-CFG-RST: 0xFFFF = clear retained navigation data, 0x01 = controlled software restart
  const uint8_t payload[] = { 0xFF, 0xFF, 0x01, 0x00 };
  sendUbx(0x06, 0x04, payload, sizeof(payload));
  delay(3000);  // Allow GPS to restart
  configureUbxReceiver();
}

static void printFrequencyStatus() {
  Serial.print(F("GPS_STATUS,bytes_seen="));
  Serial.print(gps.bytesSeen ? 1 : 0);
  Serial.print(F(",epochs="));
  Serial.print(gpsEpochCount);
  Serial.print(F(",fix="));
  Serial.print(gps.fix ? 1 : 0);
  Serial.print(F(",type="));
  Serial.print(gps.fixType);
  Serial.print(F(",sats="));
  Serial.print(gps.satellites);
  
  if (gps.fixType >= 2) {
    Serial.print(F(",lat="));
    Serial.print(gps.latitudeDeg, 7);
    Serial.print(F(",lon="));
    Serial.print(gps.longitudeDeg, 7);
    Serial.print(F(",alt_m="));
    Serial.print(gps.altitudeM, 1);
    Serial.print(F(",hAcc_m="));
    Serial.print(gps.hAccM, 2);
  }
  Serial.println();
}

void setup() {
  Serial.begin(USB_BAUD);
  const uint32_t waitStart = millis();
  while (!Serial && (millis() - waitStart < 5000UL)) { delay(10); }

  Serial.println(F("Starting GPS Cold Restart standalone script..."));

  Serial1.begin(GPS_BAUD);
  delay(500);

  // Clear pending data
  while (Serial1.available()) Serial1.read();

  coldRestartGps();
  Serial.println(F("Cold restart complete. GPS is now acquiring fix from scratch."));
}

uint32_t lastPrintMs = 0;

void loop() {
  while (Serial1.available() > 0) { 
    gps.bytesSeen = true; 
    parseUbxByte((uint8_t)Serial1.read()); 
  }
  
  if (gps.fix && (uint32_t)(millis() - gps.receivedMs) > 1500) {
    gps.fix = false;
    gps.fresh = false;
  }

  uint32_t now = millis();
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    printFrequencyStatus();
  }
}
