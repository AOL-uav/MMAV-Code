// ========================= ESEKF: 15-state Error-State Kalman Filter =========================
//
// State vector (nominal):  q(4) v(3) p(3) bg(3) ba(3)
// Error-state vector dx:  dθ(3) dv(3) dp(3) dbg(3) dba(3)
//
// Prediction:  IMU integration at 80 Hz (predictEsekf)
// Updates:
//   - Accelerometer attitude leveling (updateEsekfWithAccelerometer)
//     Skipped when: |accel| outside 1g gate  OR  |omega| > rate gate
//   - GPS position + velocity (updateEsekfWithGps)
//     Uses adaptive velocity noise from M10 sAcc field
//
// Sequential scalar updates (scalarStateUpdate) keep covariance update O(N²)
// instead of O(N³) — necessary for a 15-state filter at 80 Hz on the RP2040.

// ========================= Quaternion math =========================

static void quatNormalize(float q[4]) {
  float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n < 1.0e-8f) { q[0] = 1.f; q[1] = q[2] = q[3] = 0.f; return; }
  for (int i = 0; i < 4; i++) q[i] /= n;
}

static void quatMultiply(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static void integrateQuaternion(float q[4], const float omega[3], float dt) {
  const float wn = sqrtf(omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]);
  float dq[4];
  if (wn < 1.0e-8f) {
    dq[0] = 1.f; dq[1] = 0.5f*omega[0]*dt;
    dq[2] = 0.5f*omega[1]*dt; dq[3] = 0.5f*omega[2]*dt;
  } else {
    const float ha = 0.5f * wn * dt;
    const float sc = sinf(ha) / wn;
    dq[0] = cosf(ha); dq[1] = sc*omega[0]; dq[2] = sc*omega[1]; dq[3] = sc*omega[2];
  }
  float qn[4]; quatMultiply(q, dq, qn);
  for (int i = 0; i < 4; i++) q[i] = qn[i];
  quatNormalize(q);
}

static void quaternionToRotation(const float q[4], float r[3][3]) {
  const float w=q[0], x=q[1], y=q[2], z=q[3];
  r[0][0]=1.f-2.f*(y*y+z*z); r[0][1]=2.f*(x*y-z*w); r[0][2]=2.f*(x*z+y*w);
  r[1][0]=2.f*(x*y+z*w); r[1][1]=1.f-2.f*(x*x+z*z); r[1][2]=2.f*(y*z-x*w);
  r[2][0]=2.f*(x*z-y*w); r[2][1]=2.f*(y*z+x*w); r[2][2]=1.f-2.f*(x*x+y*y);
}

static void eulerFromQuaternion(
    const float q[4], float &roll, float &pitch, float &yaw) {
  const float w=q[0], x=q[1], y=q[2], z=q[3];
  roll  = atan2f(2.f*(w*x+y*z), 1.f-2.f*(x*x+y*y));
  pitch = asinf(clampfLocal(2.f*(w*y-z*x), -1.f, 1.f));
  yaw   = atan2f(2.f*(w*z+x*y), 1.f-2.f*(y*y+z*z));
}

static void quaternionFromRollPitch(float roll, float pitch, float q[4]) {
  const float cr=cosf(0.5f*roll),  sr=sinf(0.5f*roll);
  const float cp=cosf(0.5f*pitch), sp=sinf(0.5f*pitch);
  q[0]=cp*cr; q[1]=cp*sr; q[2]=sp*cr; q[3]=-sp*sr;
  quatNormalize(q);
}

static void injectSmallAngle(float q[4], const float dtheta[3]) {
  float dq[4] = {1.f, 0.5f*dtheta[0], 0.5f*dtheta[1], 0.5f*dtheta[2]};
  quatNormalize(dq);
  float qn[4]; quatMultiply(q, dq, qn);
  for (int i = 0; i < 4; i++) q[i] = qn[i];
  quatNormalize(q);
}

// ========================= ESEKF core =========================

static void resetEsekfFromAccel(const ImuSample &s) {
  const float roll0  = atan2f(s.accel[1], s.accel[2]);
  const float pitch0 = atan2f(-s.accel[0],
      sqrtf(s.accel[1]*s.accel[1] + s.accel[2]*s.accel[2]));
  quaternionFromRollPitch(roll0, pitch0, esekf.q);
  for (int i = 0; i < 3; i++) {
    esekf.v[i]  = 0.f; esekf.p[i]  = 0.f;
    esekf.bg[i] = gyroCalibration[i]; esekf.ba[i] = 0.f;
  }
  for (int r = 0; r < ESEKF_N; r++)
    for (int c = 0; c < ESEKF_N; c++) esekf.P[r][c] = 0.f;
  for (int i = 0; i < 3; i++) {
    esekf.P[i][i]       = sqf(8.0f  * DEG_TO_RAD_F);  // attitude: 8°
    esekf.P[i+3][i+3]   = sqf(1.0f);                   // velocity: 1 m/s
    esekf.P[i+6][i+6]   = sqf(3.0f);                   // position: 3 m
    esekf.P[i+9][i+9]   = sqf(0.04f);                  // gyro bias
    esekf.P[i+12][i+12] = sqf(0.30f);                  // accel bias
  }
  esekf.initialized = true;
}

static void applyErrorState(const float dx[ESEKF_N]) {
  injectSmallAngle(esekf.q, dx);
  for (int i = 0; i < 3; i++) {
    esekf.v[i]  += dx[i+3];
    esekf.p[i]  += dx[i+6];
    esekf.bg[i] += dx[i+9];
    esekf.ba[i] += dx[i+12];
  }
}

static void symmetrizeAndFloor() {
  for (int r = 0; r < ESEKF_N; r++) {
    for (int c = r+1; c < ESEKF_N; c++) {
      const float avg = 0.5f * (esekf.P[r][c] + esekf.P[c][r]);
      esekf.P[r][c] = avg; esekf.P[c][r] = avg;
    }
    if (esekf.P[r][r] < 1.0e-10f) esekf.P[r][r] = 1.0e-10f;
  }
}

static bool esekfHealthy() {
  float qn = 0.f;
  for (int i = 0; i < 4; i++) {
    if (!isfinite(esekf.q[i])) return false;
    qn += esekf.q[i] * esekf.q[i];
  }
  if (qn < 0.5f || qn > 1.5f) return false;
  for (int i = 0; i < 3; i++) {
    if (!isfinite(esekf.v[i])  || !isfinite(esekf.p[i]) ||
        !isfinite(esekf.bg[i]) || !isfinite(esekf.ba[i])) return false;
  }
  for (int i = 0; i < ESEKF_N; i++)
    if (!isfinite(esekf.P[i][i]) || esekf.P[i][i] <= 0.f) return false;
  return true;
}

// ========================= ESEKF prediction (80 Hz) =========================

static void predictEsekf(const ImuSample &s, float dt) {
  float omega[3], ab[3];
  for (int i = 0; i < 3; i++) {
    omega[i] = s.gyro[i]  - esekf.bg[i];
    ab[i]    = s.accel[i] - esekf.ba[i];
  }

  integrateQuaternion(esekf.q, omega, dt);

  float R[3][3]; quaternionToRotation(esekf.q, R);
  float aw[3] = {
    R[0][0]*ab[0]+R[0][1]*ab[1]+R[0][2]*ab[2],
    R[1][0]*ab[0]+R[1][1]*ab[1]+R[1][2]*ab[2],
    R[2][0]*ab[0]+R[2][1]*ab[1]+R[2][2]*ab[2] - G_MPS2
  };
  for (int i = 0; i < 3; i++) {
    esekf.p[i] += esekf.v[i]*dt + 0.5f*aw[i]*dt*dt;
    esekf.v[i] += aw[i]*dt;
  }

  // Build state-transition matrix Φ (identity + first-order terms).
  for (int r = 0; r < ESEKF_N; r++)
    for (int c = 0; c < ESEKF_N; c++) phiMat[r][c] = (r == c) ? 1.f : 0.f;

  const float skewW[3][3] = {{0,-omega[2],omega[1]},{omega[2],0,-omega[0]},{-omega[1],omega[0],0}};
  const float skewA[3][3] = {{0,-ab[2],ab[1]},{ab[2],0,-ab[0]},{-ab[1],ab[0],0}};
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      phiMat[r][c] += -skewW[r][c]*dt;
      float rsa = 0.f;
      for (int k = 0; k < 3; k++) rsa += R[r][k]*skewA[k][c];
      phiMat[r+3][c]   += -rsa*dt;
      phiMat[r+3][c+12] += -R[r][c]*dt;
    }
    phiMat[r][r+9]   += -dt;
    phiMat[r+6][r+3] +=  dt;
  }

  // P_next = Φ P Φᵀ + Q
  for (int r = 0; r < ESEKF_N; r++)
    for (int c = 0; c < ESEKF_N; c++) {
      float v = 0.f;
      for (int k = 0; k < ESEKF_N; k++) v += phiMat[r][k]*esekf.P[k][c];
      tempMat[r][c] = v;
    }
  for (int r = 0; r < ESEKF_N; r++)
    for (int c = 0; c < ESEKF_N; c++) {
      float v = 0.f;
      for (int k = 0; k < ESEKF_N; k++) v += tempMat[r][k]*phiMat[c][k];
      covarianceNext[r][c] = v;
    }

  const float qTh = sqf(GYRO_NOISE_RADPS)*dt;
  const float qV  = sqf(ACCEL_NOISE_MPS2)*dt;
  const float qP  = qV*dt*dt;
  const float qBg = sqf(GYRO_BIAS_RW_RADPS)*dt;
  const float qBa = sqf(ACCEL_BIAS_RW_MPS2)*dt;
  for (int i = 0; i < 3; i++) {
    covarianceNext[i][i]       += qTh;
    covarianceNext[i+3][i+3]   += qV;
    covarianceNext[i+6][i+6]   += qP;
    covarianceNext[i+9][i+9]   += qBg;
    covarianceNext[i+12][i+12] += qBa;
  }
  for (int r = 0; r < ESEKF_N; r++)
    for (int c = 0; c < ESEKF_N; c++) esekf.P[r][c] = covarianceNext[r][c];
  symmetrizeAndFloor();
}

// ========================= Scalar state update ===========================
//
// Updates a single observed state index with a scalar residual.
// K = P[:,idx] / (P[idx,idx] + R)
// dx = K * residual
// P_new = P - K * P[idx,:]
// Limit gate: reject if |residual| > limit (outlier protection).

static bool scalarStateUpdate(int idx, float residual, float R, float limit) {
  if (!isfinite(residual) || !isfinite(R) || fabsf(residual) > limit) return false;
  const float S = esekf.P[idx][idx] + R;
  if (S < 1.0e-9f) return false;
  float K[ESEKF_N], Hrow[ESEKF_N], dx[ESEKF_N];
  for (int i = 0; i < ESEKF_N; i++) {
    K[i]    = esekf.P[i][idx] / S;
    Hrow[i] = esekf.P[idx][i];
    dx[i]   = K[i] * residual;
  }
  applyErrorState(dx);
  for (int r = 0; r < ESEKF_N; r++)
    for (int c = 0; c < ESEKF_N; c++) esekf.P[r][c] -= K[r]*Hrow[c];
  symmetrizeAndFloor();
  return true;
}

// ========================= Accelerometer attitude update =================

static void updateEsekfWithAccelerometer(const ImuSample &s) {
  const float ax=s.accel[0], ay=s.accel[1], az=s.accel[2];
  const float norm = sqrtf(ax*ax + ay*ay + az*az);

  // Magnitude gate: only trust gravity vector when not accelerating.
  // Free fall (≈0g) and deployment loads both fall outside [0.82g, 1.18g]
  // automatically — no special-case code needed.
  if (norm < ACCEL_GATE_LOW_MPS2 || norm > ACCEL_GATE_HIGH_MPS2) return;

  // Rate gate: skip when rotating — centripetal acceleration would bias
  // the roll/pitch estimate. 0.25 rad/s ≈ 14°/s.
  // Validated by Kai Keller version: roll RMSE 14.9°→3.0° during morphing.
  const float gyroNorm = sqrtf(s.gyro[0]*s.gyro[0] +
                               s.gyro[1]*s.gyro[1] +
                               s.gyro[2]*s.gyro[2]);
  if (gyroNorm > ESEKF_LEVEL_RATE_GATE_RADPS) return;

  const float mRoll  = atan2f(ay, az);
  const float mPitch = atan2f(-ax, sqrtf(ay*ay + az*az));
  eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
  scalarStateUpdate(0, wrapPi(mRoll  - rollRad),  sqf(ACCEL_LEVEL_NOISE_RAD), 25.f*DEG_TO_RAD_F);
  eulerFromQuaternion(esekf.q, rollRad, pitchRad, yawRad);
  scalarStateUpdate(1, wrapPi(mPitch - pitchRad), sqf(ACCEL_LEVEL_NOISE_RAD), 25.f*DEG_TO_RAD_F);
}

// ========================= GPS fusion =========================

// Convert GPS lat/lon/alt to local NEU frame relative to origin.
static void gpsToLocalNeu(float neu[3]) {
  static const double RE  = 6378137.0;
  static const double D2R = 0.017453292519943295;
  const double dlat  = (gps.latitudeDeg  - gpsOriginLatDeg) * D2R;
  const double dlon  = (gps.longitudeDeg - gpsOriginLonDeg) * D2R;
  const double lat0  = gpsOriginLatDeg * D2R;
  neu[0] = (float)(RE * dlat);
  neu[1] = (float)(RE * cos(lat0) * dlon);
  neu[2] = gps.altitudeM - gpsOriginAltM;
}

static void updateEsekfWithGps() {
  if (!gps.fix || !gps.fresh) return;
  gps.fresh = false;

  if (!gpsOriginSet) {
    if (gps.fixType >= 3 && gps.satellites >= GPS_ORIGIN_MIN_SATS &&
        gps.hAccM <= GPS_ORIGIN_MAX_HACC_M) {
      gpsOriginGoodCount++;
      if (gpsOriginGoodCount >= GPS_ORIGIN_STABLE_FIXES) {
        gpsOriginLatDeg = gps.latitudeDeg;
        gpsOriginLonDeg = gps.longitudeDeg;
        gpsOriginAltM   = gps.altitudeM;
        gpsOriginSet    = true;
        // Save GPS state to BBR so next power-on achieves a hot start.
        // Sent exactly once, right after the best-quality fix is locked.
        if (!gpsSosSent) { gpsSaveToBackupRam(); gpsSosSent = true; }
      }
    } else { gpsOriginGoodCount = 0; }
    return;
  }

  float neu[3]; gpsToLocalNeu(neu);
  const float hStd = fmaxf(gps.hAccM, GPS_POSITION_NOISE_FLOOR_M);
  const float vStd = fmaxf(gps.vAccM, GPS_VERTICAL_NOISE_FLOOR_M);
  // Adaptive velocity noise: use M10's reported sAcc, floored at
  // GPS_VELOCITY_NOISE_MPS. Tighter in open sky, looser when obstructed.
  const float velStd = fmaxf(gps.sAccMps, GPS_VELOCITY_NOISE_MPS);

  // NIS (Normalized Innovation Squared) accumulated across the sequential
  // scalar updates: sum of residual^2 / S with S = P[i][i] + R taken BEFORE
  // each update. Approximates the full-covariance chi-square (3 dof) —
  // standard filter-consistency metric, logged for offline validation.
  // Computed even for residuals the outlier gate rejects, so gate events
  // show up as NIS spikes in the log.
  const float vNeu[3] = {gps.velocityNed[0], gps.velocityNed[1], -gps.velocityNed[2]};
  float nisV = 0.f;
  for (int i = 0; i < 3; i++) {
    const float r_ = vNeu[i] - esekf.v[i];
    nisV += r_*r_ / (esekf.P[i+3][i+3] + sqf(velStd));
    scalarStateUpdate(i+3, r_, sqf(velStd), 25.f);
  }
  gpsNisVel = nisV;

  float nisP = 0.f;
  {
    const float rN = neu[0] - esekf.p[0];
    nisP += rN*rN / (esekf.P[6][6] + sqf(hStd));
    scalarStateUpdate(6, rN, sqf(hStd), 100.f);
    const float rE = neu[1] - esekf.p[1];
    nisP += rE*rE / (esekf.P[7][7] + sqf(hStd));
    scalarStateUpdate(7, rE, sqf(hStd), 100.f);
    const float rU = neu[2] - esekf.p[2];
    nisP += rU*rU / (esekf.P[8][8] + sqf(vStd));
    scalarStateUpdate(8, rU, sqf(vStd), 100.f);
  }
  gpsNisPos = nisP;
}
