#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// Inert USB/SD bench diagnostic for the Nano RP2040 Connect.
// It deliberately does not initialize Wi-Fi, GPS, IMU, or any servo pins.
static const uint8_t SD_CS_PIN = 10;
static const char *diagnosticResult = "NOT_RUN";

static File openDiagnosticFile() {
  char name[16];
  for (unsigned int index = 0; index < 1000; ++index) {
    snprintf(name, sizeof(name), "SDTEST_%03u.TXT", index);
    if (!SD.exists(name)) return SD.open(name, FILE_WRITE);
  }
  return File();
}

void setup() {
  Serial.begin(115200);
  const uint32_t serialStart = millis();
  while (!Serial && millis() - serialStart < 3000) delay(10);

  Serial.println("SD_DIAGNOSTIC_BEGIN");
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  SPI.begin();
  delay(250);

  bool mounted = false;
  for (uint8_t attempt = 1; attempt <= 5; ++attempt) {
    Serial.print("SD_BEGIN_ATTEMPT=");
    Serial.println(attempt);
    if (SD.begin(SD_CS_PIN)) {
      mounted = true;
      break;
    }
    delay(500);
  }
  if (!mounted) {
    diagnosticResult = "MOUNT_FAILED";
    Serial.println("SD_RESULT=MOUNT_FAILED");
    return;
  }

  File file = openDiagnosticFile();
  if (!file) {
    diagnosticResult = "OPEN_FAILED";
    Serial.println("SD_RESULT=OPEN_FAILED");
    return;
  }
  Serial.print("SD_FILE=");
  Serial.println(file.name());
  file.println("ms,sequence");
  file.flush();
  if (file.getWriteError()) {
    diagnosticResult = "HEADER_WRITE_FAILED";
    Serial.println("SD_RESULT=HEADER_WRITE_FAILED");
    file.close();
    return;
  }

  for (uint8_t sequence = 0; sequence < 40; ++sequence) {
    file.print(millis());
    file.print(',');
    file.println(sequence);
    file.flush();
    if (file.getWriteError()) {
      diagnosticResult = "WRITE_FAILED";
      Serial.print("SD_RESULT=WRITE_FAILED_AT=");
      Serial.println(sequence);
      file.close();
      return;
    }
    delay(250);
  }
  file.close();
  diagnosticResult = "PASS";
  Serial.println("SD_RESULT=PASS");
}

void loop() {
  Serial.print("SD_DIAGNOSTIC_RESULT=");
  Serial.println(diagnosticResult);
  delay(1000);
}
