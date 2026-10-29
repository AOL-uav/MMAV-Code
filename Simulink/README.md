# Flight Control & IMU Sensor Fusion Software-in-the-Loop (SIL) Benchmarking

This repository contains MATLAB scripts and Simulink models for Software-in-the-Loop (SIL) simulation, real-time IMU data acquisition, filter tuning ($Q$ and $R$ noise parameters), and flight control architecture evaluation (PID vs. Lead-Lag controllers).

---

## Repository Files Overview

The repository contains the following root-level files:

```text
.
├── Collect_IMU_Data.m       # Serial ingestion script to collect raw 6-DoF static IMU data from an RP2040/microcontroller
├── IMU_Filter.slx           # Simulink model for real-time serial hardware data ingestion and filter comparison (CF, KF, EKF, IEKF)
├── IMU_Filter_Benchmarking.slx # Simulink model for benchmarking filter tracking RMSE against synthesized reference state data
├── KF_QnR_Finder.m          # MATLAB script to compute measurement variance (R) and process noise covariance (Q)
├── PIL_Master_Analysis.m    # Master execution script to run SIL simulations programmatically and compute tracking/jitter performance metrics
└── SIL_PID_vs_LeadLag.slx   # Simulink model comparing PID and Lead-Lag pitch/roll controllers using EKF estimation
```

---

## Technical Specifications

- **Hardware Stream Format:** 6-DoF IMU packet streaming at $115,200\text{ baud}$ over serial (`/dev/ttyACM0`). Packets consist of 24 bytes (6 single-precision floats: `[ax, ay, az, gx, gy, gz]`).
- **Attitude Estimation Filters:**
  - Complementary Filter (CF)
  - Standard Kalman Filter (KF)
  - Extended Kalman Filter (EKF)
  - Iterated Extended Kalman Filter (IEKF)
- **Control Architectures Evaluated:** Classical PID vs. Lead-Lag Compensator.

---

## Workflow Instructions

### 1. Noise Parameter Identification ($Q$ & $R$)
1. Connect the micro-controller streaming IMU data over serial.
2. Keep the sensor stationary and execute:
   ```matlab
   run('Collect_IMU_Data.m')
   ```
3. The script collects 1,000 static samples, calculates pitch measurement variance ($R$), and estimates process noise covariance ($Q$) based on rate gyro standard deviation ($\sigma_{\text{gyro}}$).
4. Alternatively, use `KF_QnR_Finder.m` to compute noise parameters from existing dataset matrices.

### 2. Filter Benchmarking
- **Live Hardware Data:** Open `IMU_Filter.slx` to inspect incoming serial data streams and observe real-time filter response across CF, KF, EKF, and IEKF architectures.
- **Synthetic Performance Evaluation:** Open `IMU_Filter_Benchmarking.slx` to quantify filter performance against synthetic truth trajectories.

### 3. Automated SIL Controller Analysis
Execute the master runner to run the `SIL_PID_vs_LeadLag.slx` model programmatically and output performance comparative metrics:

```matlab
run('PIL_Master_Analysis.m')
```

#### Evaluation Metrics Calculated:
- **Tracking RMSE ($\text{deg}$):** Measures trajectory-following accuracy against step inputs across pitch and roll axes.
- **Control Effort Variance ($\text{deg}^2$):** Measures control surface chatter and actuator jitter on elevator and aileron channels.

---

## Requirements & Dependencies

- **MATLAB / Simulink:** R2022b or newer
- **Toolboxes Required:**
  - Control System Toolbox
  - Instrument Control Toolbox (for serial buffer reading)
  - Simulink Coder