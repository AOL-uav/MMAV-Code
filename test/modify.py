import os
import re

src = r"C:\Users\Kai\Downloads\rotational_mode_ubx_pvt_tuned_SDlog50Hz.ino"
dst_dir = r"C:\Users\Kai\OneDrive - purdue.edu\Purdue\AOL\MAV-2026\github stuff\test\telemetry_wifi_ota"
dst = os.path.join(dst_dir, "telemetry_wifi_ota.ino")

os.makedirs(dst_dir, exist_ok=True)

with open(src, "r", encoding="utf-8") as f:
    code = f.read()

# 1. Add headers and globals
header_addition = """#include <SD.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "Kai's A55";
const char* pass = "Mika12345.";
WiFiUDP Udp;
const char* udpAddress = "255.255.255.255";
const int udpPort = 5000;

rtos::Thread otaThread;
void otaTask() {
  while (true) {
    ArduinoOTA.poll();
    rtos::ThisThread::yield();
    delay(20);
  }
}
"""
code = code.replace("#include <SD.h>", header_addition)

# 2. Inject WiFi and OTA setup in setup()
setup_addition = """void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t serialStartMs = millis();
  while (!Serial && (uint32_t)(millis() - serialStartMs) < SERIAL_WAIT_MS) {
    delay(10);
  }

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  int wifi_retries = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_retries < 15) {
    delay(500);
    Serial.print(".");
    wifi_retries++;
  }
  if(WiFi.status() == WL_CONNECTED) {
      Serial.println("\\nWiFi connected.");
      ArduinoOTA.setHostname("Nano-Telemetry");
      ArduinoOTA.begin();
      otaThread.start(otaTask);
      Udp.begin(udpPort);
  } else {
      Serial.println("\\nWiFi failed, continuing without wireless.");
  }
"""
code = code.replace("""void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t serialStartMs = millis();
  while (!Serial &&
         (uint32_t)(millis() - serialStartMs) < SERIAL_WAIT_MS) {
    delay(10);
  }""", setup_addition)

# 3. Inject UDP broadcast inside loggerTask
udp_addition = """      writeRecord(logFile, *rec);
      
      // UDP Broadcast of binary record
      if (WiFi.status() == WL_CONNECTED) {
        Udp.beginPacket(udpAddress, udpPort);
        Udp.write((const uint8_t*)rec, sizeof(ControlRecord));
        Udp.endPacket();
      }
"""
code = code.replace("      writeRecord(logFile, *rec);", udp_addition)

with open(dst, "w", encoding="utf-8") as f:
    f.write(code)

print("Modification complete.")
