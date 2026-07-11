import sys

file_path = "/home/kaikeller/Purdue/AOL/MAV/test/rotational_mode/rotational_mode_test/rotational_mode_test.ino"

with open(file_path, 'r') as f:
    lines = f.readlines()

new_lines = []
skip = False
for i, line in enumerate(lines):
    if line.startswith("#include <Arduino.h>"):
        new_lines.append(line)
        new_lines.append("#include <TinyGPSPlus.h>\n")
        continue

    # Global variables
    if line.startswith("static Servo servoLeft;"):
        new_lines.append("static TinyGPSPlus tinyGps;\n")
        new_lines.append(line)
        continue

    # Remove UBX parser completely
    if "// ========================= GPS NEO-6M UBX parser =========================" in line:
        skip = True
        new_lines.append("// ========================= GPS TinyGPSPlus parser =========================\n")
        continue

    if skip and "// ========================= IMU =========================" in line:
        skip = False
        # Insert the new pollGps function here
        new_poll_gps = """
static void pollGps() {
  while (Serial1.available() > 0) {
    tinyGps.encode(Serial1.read());
  }

  if (tinyGps.location.isUpdated()) {
    gps.fix = tinyGps.location.isValid();
    gps.fresh = true;
    gps.satellites = tinyGps.satellites.value();
    gps.latitudeDeg = tinyGps.location.lat();
    gps.longitudeDeg = tinyGps.location.lng();
    gps.altitudeM = tinyGps.altitude.meters();
    gps.hAccM = tinyGps.hdop.value() / 100.0f; // Approx conversion
    gps.vAccM = gps.hAccM * 1.5f;

    float speed = tinyGps.speed.mps();
    float courseRad = tinyGps.course.deg() * DEG_TO_RAD_F;
    gps.velocityNed[0] = speed * cosf(courseRad);
    gps.velocityNed[1] = speed * sinf(courseRad);
    gps.velocityNed[2] = 0.0f; // NMEA doesn't give reliable vertical velocity usually
  }
}

"""
        new_lines.append(new_poll_gps)
        new_lines.append(line)
        continue

    if skip:
        continue

    # Remove configureNeo6m call
    if "configureNeo6m();" in line:
        continue

    # Inject telemetry print into loop()
    # At the end of loop(), right before the last closing brace
    # Wait, the loop function ends with a `}` and there is no function after it.
    
    # We will just append to `new_lines` for now, we'll patch the loop function after the loop.
    new_lines.append(line)

content = "".join(new_lines)

telemetry_print = """
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) { 
    lastPrint = millis();
    
    Serial.println(F("========= TELEMETRY ========="));

    if (imu.valid) {
      Serial.print(F("Accel (m/s2): X=")); Serial.print(imu.accel[0], 3);
      Serial.print(F(", Y=")); Serial.print(imu.accel[1], 3);
      Serial.print(F(", Z=")); Serial.println(imu.accel[2], 3);

      Serial.print(F("Gyro (rad/s): X=")); Serial.print(imu.gyro[0], 3);
      Serial.print(F(", Y=")); Serial.print(imu.gyro[1], 3);
      Serial.print(F(", Z=")); Serial.println(imu.gyro[2], 3);
    }
    
    if (gps.fix) {
      Serial.print(F("Global Pos:  Lat=")); Serial.print(gps.latitudeDeg, 6);
      Serial.print(F(", Lon=")); Serial.print(gps.longitudeDeg, 6);
      Serial.print(F(", Alt=")); Serial.print(gps.altitudeM, 2);
      Serial.println(F(" m"));
      
      float speed = sqrtf(gps.velocityNed[0]*gps.velocityNed[0] + gps.velocityNed[1]*gps.velocityNed[1]);
      Serial.print(F("Velocity:    Speed=")); Serial.print(speed, 2);
      Serial.println(F(" m/s"));
      
      if (gpsOriginSet) {
        float gpsPosition[3];
        gpsToLocalNeu(gpsPosition);
        Serial.print(F("Local XYZ:   X(North)=")); Serial.print(gpsPosition[0], 2);
        Serial.print(F("m, Y(East)=")); Serial.print(gpsPosition[1], 2);
        Serial.print(F("m, Z(Up)=")); Serial.print(gpsPosition[2], 2);
        Serial.println(F("m"));
      }
    } else {
      Serial.println(F("GPS: No valid fix yet (waiting for satellites...)"));
    }
    Serial.println(F("=============================\\n"));
  }
"""

# Insert telemetry print into loop() before the final brace
# Find the last closing brace of the file
last_brace_index = content.rfind("}")
content = content[:last_brace_index] + telemetry_print + "\n}\n"

with open(file_path, 'w') as f:
    f.write(content)

print("Patching complete!")
