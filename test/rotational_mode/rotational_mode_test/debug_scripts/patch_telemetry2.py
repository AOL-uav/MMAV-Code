import sys

file_path = "/home/kaikeller/Purdue/AOL/MAV/test/rotational_mode/rotational_mode_test/rotational_mode_test.ino"

with open(file_path, 'r') as f:
    content = f.read()

# Add global vars
if "volatile bool globalSdReady" not in content:
    insert_point = content.find("static Servo servoTail;")
    if insert_point != -1:
        content = content[:insert_point] + "volatile bool globalSdReady = false;\nvolatile bool globalSdFailed = false;\n" + content[insert_point:]

# Update loggerTask
old_logger = """  if (!sdReady) {
    safeSerialPrintln(F("[Core 1] ERROR: SD.begin failed - logging disabled."));
  } else {"""

new_logger = """  if (!sdReady) {
    globalSdFailed = true;
    safeSerialPrintln(F("[Core 1] ERROR: SD.begin failed - logging disabled."));
  } else {
    globalSdReady = true;"""

content = content.replace(old_logger, new_logger)

# Update telemetry
old_telemetry = """    if (tinyGps.location.isValid()) {"""
new_telemetry = """    Serial.print(F("SD Status:   "));
    if (globalSdFailed) Serial.println(F("FAILED"));
    else if (globalSdReady) Serial.println(F("OK"));
    else Serial.println(F("INIT..."));
    if (tinyGps.location.isValid()) {"""

content = content.replace(old_telemetry, new_telemetry)

with open(file_path, 'w') as f:
    f.write(content)
print("Patched rotational_mode_test.ino to print SD status in telemetry loop.")
