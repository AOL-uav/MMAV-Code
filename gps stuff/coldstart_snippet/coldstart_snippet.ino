#include <Arduino.h>

static const uint32_t USB_BAUD = 115200;
static const uint32_t GPS_BAUD = 115200;

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
  const uint8_t payload[3] = {messageClass, messageId, rate};
  sendUbx(0x06, 0x01, payload, sizeof(payload));
  delay(30);
}

static void configureUbxReceiver() {
  Serial.println(F("Configuring GPS messages..."));

  // UBX-CFG-RATE: 1000 ms measurement period
  const uint8_t ratePayload[6] = {
    0xE8, 0x03,  // 1000 ms
    0x01, 0x00,  // navRate = 1
    0x00, 0x00   // timeRef = UTC
  };
  sendUbx(0x06, 0x08, ratePayload, sizeof(ratePayload));
  delay(50);

  configureUbxMessage(0x01, 0x07, 1);  // UBX-NAV-PVT
  Serial.println(F("Configuration sent."));
}

static void coldRestartGps() {
  Serial.println(F("Sending GPS cold-start reset..."));
  // UBX-CFG-RST:
  // 0xFFFF = clear retained navigation data
  // 0x01   = controlled software restart
  const uint8_t payload[] = {
    0xFF, 0xFF,
    0x01,
    0x00
  };

  sendUbx(0x06, 0x04, payload, sizeof(payload));

  delay(3000);  // Allow GPS to restart

  // Reapply UART message/rate settings after restart.
  configureUbxReceiver();
}

void setup() {
  Serial.begin(USB_BAUD);
  // Wait for USB serial connection
  const uint32_t waitStart = millis();
  while (!Serial && (millis() - waitStart < 5000UL)) {
    delay(10);
  }

  Serial.println(F("Starting GPS Cold Restart standalone script..."));

  Serial1.begin(GPS_BAUD);
  delay(500);

  coldRestartGps();
  
  Serial.println(F("Cold restart complete. GPS is now acquiring fix from scratch."));
}

void loop() {
  // Echo GPS data to Serial monitor if any
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
