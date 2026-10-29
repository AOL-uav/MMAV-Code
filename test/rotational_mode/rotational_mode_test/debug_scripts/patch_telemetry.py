import sys

file_path = "/home/kaikeller/Purdue/AOL/MAV/test/rotational_mode/rotational_mode_test/rotational_mode_test.ino"

with open(file_path, 'r') as f:
    content = f.read()

old_block = """    if (imu.valid) {
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
    } else {"""

new_block = """    float ax = 0, ay = 0, az = 0;
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      Serial.print(F("Accel (g):   X=")); Serial.print(ax, 3);
      Serial.print(F(", Y=")); Serial.print(ay, 3);
      Serial.print(F(", Z=")); Serial.println(az, 3);
    }
    
    float gx = 0, gy = 0, gz = 0;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      Serial.print(F("Gyro (dps):  X=")); Serial.print(gx, 1);
      Serial.print(F(", Y=")); Serial.print(gy, 1);
      Serial.print(F(", Z=")); Serial.println(gz, 1);
    }

    if (tinyGps.location.isValid()) {
      double lat = tinyGps.location.lat();
      double lon = tinyGps.location.lng();
      double alt = tinyGps.altitude.meters();
      
      Serial.print(F("Global Pos:  Lat=")); Serial.print(lat, 6);
      Serial.print(F(", Lon=")); Serial.print(lon, 6);
      Serial.print(F(", Alt=")); Serial.print(alt, 2);
      Serial.println(F(" m"));
      
      Serial.print(F("Velocity:    Speed=")); Serial.print(tinyGps.speed.mps(), 2);
      Serial.print(F(" m/s, Course=")); Serial.print(tinyGps.course.deg(), 2);
      Serial.println(F(" deg"));
      
      if (gpsOriginSet) {
        float gpsPosition[3];
        gpsToLocalNeu(gpsPosition);
        Serial.print(F("Local XYZ:   X(North)=")); Serial.print(gpsPosition[0], 2);
        Serial.print(F("m, Y(East)=")); Serial.print(gpsPosition[1], 2);
        Serial.print(F("m, Z(Up)=")); Serial.print(gpsPosition[2], 2);
        Serial.println(F("m"));
      }
    } else {"""

content = content.replace(old_block, new_block)

with open(file_path, 'w') as f:
    f.write(content)

print("Telemetry patch complete!")
