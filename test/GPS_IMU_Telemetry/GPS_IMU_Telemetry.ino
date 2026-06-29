#include <TinyGPSPlus.h>
#include <Arduino_LSM6DSOX.h>

TinyGPSPlus gps;

// Variables for calculating relative XYZ (Local Tangent Plane approximation)
bool homeSet = false;
double homeLat = 0.0;
double homeLon = 0.0;
double homeAlt = 0.0;

// Earth radius in meters
const double R = 6371000.0; 

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  
  while (!Serial) {
    delay(10);
  }
  
  Serial.println(F("Nano RP2040 GPS & IMU Telemetry"));
  
  // Initialize the built-in IMU on the Nano RP2040 Connect
  if (!IMU.begin()) {
    Serial.println(F("Failed to initialize IMU!"));
    // We won't block forever just in case, but warn the user
  } else {
    Serial.println(F("IMU Initialized successfully."));
  }
  
  Serial.println(F("Waiting for GPS data..."));
}

void loop() {
  // 1. Process all available GPS characters
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // 2. Print our telemetry at ~2Hz (every 500ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) { // Update at 5Hz for faster IMU feedback
    // Wait, let's make it print every 1 second
    lastPrint = millis();
    
    Serial.println(F("========= TELEMETRY ========="));

    // --- ACCELERATION (IMU) ---
    float ax = 0, ay = 0, az = 0;
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(ax, ay, az);
      // IMU returns data in g's (1g = 9.81 m/s^2)
      Serial.print(F("Accel (g):   X=")); Serial.print(ax, 3);
      Serial.print(F(", Y=")); Serial.print(ay, 3);
      Serial.print(F(", Z=")); Serial.println(az, 3);
    }
    
    // --- GYROSCOPE (IMU) ---
    float gx = 0, gy = 0, gz = 0;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      Serial.print(F("Gyro (dps):  X=")); Serial.print(gx, 1);
      Serial.print(F(", Y=")); Serial.print(gy, 1);
      Serial.print(F(", Z=")); Serial.println(gz, 1);
    }

    // --- GPS POSITION & VELOCITY ---
    if (gps.location.isValid()) {
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      double alt = gps.altitude.meters();
      
      Serial.print(F("Global Pos:  Lat=")); Serial.print(lat, 6);
      Serial.print(F(", Lon=")); Serial.print(lon, 6);
      Serial.print(F(", Alt=")); Serial.print(alt, 2);
      Serial.println(F(" m"));
      
      Serial.print(F("Velocity:    Speed=")); Serial.print(gps.speed.mps(), 2);
      Serial.print(F(" m/s, Course=")); Serial.print(gps.course.deg(), 2);
      Serial.println(F(" deg"));
      
      // Calculate local XYZ (North-East-Down approximation) relative to first fix
      if (!homeSet) {
        homeLat = lat;
        homeLon = lon;
        homeAlt = alt;
        homeSet = true;
        Serial.println(F("*** HOME POSITION SET ***"));
      } else {
        // Very basic equirectangular approximation for small distances
        double latRad = radians(lat);
        double lonRad = radians(lon);
        double homeLatRad = radians(homeLat);
        double homeLonRad = radians(homeLon);
        
        double x_north = R * (latRad - homeLatRad);
        double y_east  = R * cos(homeLatRad) * (lonRad - homeLonRad);
        double z_up    = alt - homeAlt;
        
        Serial.print(F("Local XYZ:   X(North)=")); Serial.print(x_north, 2);
        Serial.print(F("m, Y(East)=")); Serial.print(y_east, 2);
        Serial.print(F("m, Z(Up)=")); Serial.print(z_up, 2);
        Serial.println(F("m"));
      }
    } else {
      Serial.println(F("GPS: No valid fix yet (waiting for satellites...)"));
    }
    Serial.println(F("=============================\n"));
  }
}


