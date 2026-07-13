/*
  UBX GPS bench test for a u-blox NEO-6M / GY-NEO6MV2.

  This is intentionally GPS-only: it does not include or control servos.
  Wiring: GPS TX -> Nano D1/RX, GPS RX -> Nano D0/TX, and common GND.
  The receiver is configured at runtime only; settings are not saved to flash.
*/

#include <Arduino.h>

static const uint32_t USB_BAUD = 115200;
static const uint32_t GPS_BAUD = 9600;
static const uint16_t GPS_PERIOD_MS = 200;  // 5 Hz

enum UbxState : uint8_t {
  UBX_SYNC1, UBX_SYNC2, UBX_CLASS, UBX_ID, UBX_LEN1, UBX_LEN2,
  UBX_PAYLOAD, UBX_CK_A, UBX_CK_B
};

static UbxState parserState = UBX_SYNC1;
static uint8_t messageClass = 0, messageId = 0;
static uint16_t payloadLength = 0, payloadIndex = 0;
static uint8_t checksumA = 0, checksumB = 0;
static uint8_t payload[100];

static uint32_t uartBytes = 0, ubxPackets = 0;
static uint32_t positionTow = 0, solutionTow = 0, velocityTow = 0;
static bool validFix = false;
static uint8_t fixType = 0, satellites = 0;
static double latitudeDeg = 0.0, longitudeDeg = 0.0;
static float altitudeM = 0.0f, hAccM = 0.0f, vAccM = 0.0f;
static float velocityNed[3] = {};

static uint32_t readU32Le(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t readI32Le(const uint8_t *p) {
  return (int32_t)readU32Le(p);
}

static void checksumAdd(uint8_t value) {
  checksumA += value;
  checksumB += checksumA;
}

static void handlePacket() {
  ubxPackets++;
  if (messageClass != 0x01) return;  // UBX-NAV

  if (messageId == 0x02 && payloadLength == 28) {  // NAV-POSLLH
    positionTow = readU32Le(&payload[0]);
    longitudeDeg = readI32Le(&payload[4]) * 1.0e-7;
    latitudeDeg = readI32Le(&payload[8]) * 1.0e-7;
    altitudeM = readI32Le(&payload[16]) * 1.0e-3f;
    hAccM = readU32Le(&payload[20]) * 1.0e-3f;
    vAccM = readU32Le(&payload[24]) * 1.0e-3f;
  } else if (messageId == 0x06 && payloadLength == 52) {  // NAV-SOL
    solutionTow = readU32Le(&payload[0]);
    fixType = payload[10];
    satellites = payload[47];
    validFix = (payload[11] & 0x01U) != 0U && fixType >= 3;
  } else if (messageId == 0x12 && payloadLength == 36) {  // NAV-VELNED
    velocityTow = readU32Le(&payload[0]);
    velocityNed[0] = readI32Le(&payload[4]) * 0.01f;
    velocityNed[1] = readI32Le(&payload[8]) * 0.01f;
    velocityNed[2] = readI32Le(&payload[12]) * 0.01f;
  }
}

static void parseUbxByte(uint8_t value) {
  switch (parserState) {
    case UBX_SYNC1: parserState = value == 0xB5 ? UBX_SYNC2 : UBX_SYNC1; break;
    case UBX_SYNC2: parserState = value == 0x62 ? UBX_CLASS : UBX_SYNC1; break;
    case UBX_CLASS:
      messageClass = value;
      checksumA = checksumB = 0;
      checksumAdd(value);
      parserState = UBX_ID;
      break;
    case UBX_ID: messageId = value; checksumAdd(value); parserState = UBX_LEN1; break;
    case UBX_LEN1: payloadLength = value; checksumAdd(value); parserState = UBX_LEN2; break;
    case UBX_LEN2:
      payloadLength |= (uint16_t)value << 8;
      checksumAdd(value);
      payloadIndex = 0;
      parserState = payloadLength > sizeof(payload) ? UBX_SYNC1 :
                    (payloadLength == 0 ? UBX_CK_A : UBX_PAYLOAD);
      break;
    case UBX_PAYLOAD:
      payload[payloadIndex++] = value;
      checksumAdd(value);
      if (payloadIndex >= payloadLength) parserState = UBX_CK_A;
      break;
    case UBX_CK_A: parserState = value == checksumA ? UBX_CK_B : UBX_SYNC1; break;
    case UBX_CK_B:
      if (value == checksumB) handlePacket();
      parserState = UBX_SYNC1;
      break;
  }
}

static void sendUbx(uint8_t cls, uint8_t id, const uint8_t *data, uint16_t length) {
  uint8_t ckA = 0, ckB = 0;
  const uint8_t header[4] = {cls, id, (uint8_t)(length & 0xFFU), (uint8_t)(length >> 8)};
  Serial1.write(0xB5); Serial1.write(0x62);
  for (uint8_t value : header) {
    Serial1.write(value); ckA += value; ckB += ckA;
  }
  for (uint16_t i = 0; i < length; i++) {
    Serial1.write(data[i]); ckA += data[i]; ckB += ckA;
  }
  Serial1.write(ckA); Serial1.write(ckB);
  Serial1.flush();
}

static void configureMessage(uint8_t cls, uint8_t id, uint8_t rate) {
  const uint8_t data[3] = {cls, id, rate};
  sendUbx(0x06, 0x01, data, sizeof(data));  // UBX-CFG-MSG
  delay(15);
}

static void configureReceiver() {
  const uint8_t rate[6] = {
    (uint8_t)(GPS_PERIOD_MS & 0xFFU), (uint8_t)(GPS_PERIOD_MS >> 8),
    0x01, 0x00, 0x01, 0x00
  };
  sendUbx(0x06, 0x08, rate, sizeof(rate));  // UBX-CFG-RATE
  delay(30);
  for (uint8_t nmeaId = 0; nmeaId <= 5; nmeaId++) configureMessage(0xF0, nmeaId, 0);
  configureMessage(0x01, 0x02, 1);  // NAV-POSLLH
  configureMessage(0x01, 0x06, 1);  // NAV-SOL
  configureMessage(0x01, 0x12, 1);  // NAV-VELNED
}

void setup() {
  Serial.begin(USB_BAUD);
  const uint32_t serialStart = millis();
  while (!Serial && millis() - serialStart < 1500) delay(10);
  Serial1.begin(GPS_BAUD);
  delay(300);
  configureReceiver();
  Serial.println(F("UBX GPS test started at 9600 baud."));
}

void loop() {
  while (Serial1.available() > 0) {
    uartBytes++;
    parseUbxByte((uint8_t)Serial1.read());
  }

  static uint32_t lastReportMs = 0;
  if (millis() - lastReportMs < 1000) return;
  lastReportMs = millis();

  Serial.print(F("uart_bytes=")); Serial.print(uartBytes);
  Serial.print(F(", ubx_packets=")); Serial.print(ubxPackets);
  Serial.print(F(", fix=")); Serial.print(validFix ? F("yes") : F("no"));
  Serial.print(F(", fix_type=")); Serial.print(fixType);
  Serial.print(F(", satellites=")); Serial.print(satellites);
  if (validFix && positionTow == solutionTow && velocityTow == solutionTow) {
    Serial.print(F(", lat=")); Serial.print(latitudeDeg, 7);
    Serial.print(F(", lon=")); Serial.print(longitudeDeg, 7);
    Serial.print(F(", alt_m=")); Serial.print(altitudeM, 1);
    Serial.print(F(", hacc_m=")); Serial.print(hAccM, 2);
    Serial.print(F(", vn=")); Serial.print(velocityNed[0], 2);
    Serial.print(F(", ve=")); Serial.print(velocityNed[1], 2);
    Serial.print(F(", vd=")); Serial.print(velocityNed[2], 2);
  }
  Serial.println();
}
