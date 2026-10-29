// ========================= IMU: LSM6DSOX =========================
//
// The LSM6DSOX is accessed via the Arduino_LSM6DSOX library.
// readImu() polls until both accel and gyro are available;
// calibrateGyro() collects CAL_SAMPLES readings and computes bias.
//
// Axis mapping (library → body frame):
//   accel[0] = X forward,  accel[1] = Y left,  accel[2] = Z up   (m/s²)
//   gyro[0]  = roll rate,  gyro[1]  = pitch rate, gyro[2] = yaw rate (rad/s)

static bool readImu(ImuSample &s) {
  const uint32_t t0 = micros();
  while (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
    if ((uint32_t)(micros() - t0) > 8000UL) { s.valid = false; return false; }
  }
  float axG, ayG, azG, gxDps, gyDps, gzDps;
  IMU.readAcceleration(axG, ayG, azG);
  IMU.readGyroscope(gxDps, gyDps, gzDps);
  s.accel[0] = axG * G_MPS2;
  s.accel[1] = ayG * G_MPS2;
  s.accel[2] = azG * G_MPS2;
  s.gyro[0]  = gxDps * DEG_TO_RAD_F;
  s.gyro[1]  = gyDps * DEG_TO_RAD_F;
  s.gyro[2]  = gzDps * DEG_TO_RAD_F;
  s.valid    = true;
  return true;
}

// Collects CAL_SAMPLES static samples and fits a mean gyro bias.
// Returns false if:
//   - fewer than half the samples were usable (IMU timeout)
//   - accelerometer magnitude is outside 1g gate (board was moved)
//   - any gyro axis bias exceeds 5·GYRO_NOISE_RADPS (sensor fault)
//   - gyro variance exceeds 3·GYRO_NOISE_RADPS² (vibration present)
static bool calibrateGyro() {
  float sum[3] = {}, sumSq[3] = {};
  uint16_t good = 0;
  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    ImuSample s = {};
    if (readImu(s)) {
      const float aN = sqrtf(s.accel[0]*s.accel[0] +
                             s.accel[1]*s.accel[1] +
                             s.accel[2]*s.accel[2]);
      if (aN < ACCEL_GATE_LOW_MPS2 || aN > ACCEL_GATE_HIGH_MPS2) return false;
      for (int a = 0; a < 3; a++) {
        sum[a]   += s.gyro[a];
        sumSq[a] += s.gyro[a] * s.gyro[a];
      }
      good++;
    }
    delay(CAL_DT_MS);
  }
  if (good < CAL_SAMPLES / 2) return false;
  for (int a = 0; a < 3; a++) {
    gyroCalibration[a] = sum[a] / (float)good;
    const float var = sumSq[a] / (float)good - sqf(gyroCalibration[a]);
    if (fabsf(gyroCalibration[a]) > 5.0f * GYRO_NOISE_RADPS) return false;
    if (var > sqf(3.0f * GYRO_NOISE_RADPS)) return false;
  }
  return true;
}
