# Purdue AOL MAV - SD Logging Project

## Directory Structure
- **Firmware/**: Contains all Arduino sketches.
    - `gliding_pid_sd/`: **Primary** flight controller sketch (100Hz logging).
    - `gliding_pid_buffered/`: High-speed logger (500Hz) for actuator lag measurement.
    - `gliding_pid_v62_logged/`: **June 2** integration with full state logging.
    - `original_62_gliding_pid/`: The unmodified original **June 2** code for reference.
- **test/**: Hardware verification and debugging tools.
    - `mini_logger/`: Minimal SD writing test.
    - `fetch_logs.py`: Python script to download data over Serial.

## File Naming Conventions
- **Firmware Numbering**: The `62` in file and folder names refers to the development date of **June 2nd**, not a version number.
- **Log Files**: 
    - `fl000.csv`: Standard flight logs from `gliding_pid_sd`.
    - `bt00.csv`: Burst/High-speed logs from `gliding_pid_buffered`.

## CSV Header Definitions

### Standard Log (`flXXX.csv`)
| Header | Unit | Description |
| :--- | :--- | :--- |
| `ms` | milliseconds | System uptime since boot. |
| `armed` | boolean | 1 if motors/servos are active, 0 otherwise. |
| `est_ok` | boolean | 1 if IMU orientation estimator has converged. |
| `acc_ok` | boolean | 1 if accelerometer magnitude is within 1G limits (static/quasi-static). |
| `roll` | degrees | Estimated vehicle roll angle. |
| `pitch` | degrees | Estimated vehicle pitch angle. |
| `p, q, r` | deg/s | Filtered angular rates (Roll, Pitch, Yaw). |
| `a` | Gs | Normalized acceleration magnitude (Total G-force). |
| `lcmd, rcmd` | degrees | Filtered PID commands for left/right elevons. |
| `ls, rs` | degrees | Final output servo angles (0-180). |
| `lp, rp` | microseconds | Raw PWM pulses sent to left/right servos. |

### Burst Log (`btXX.csv`)
| Header | Unit | Description |
| :--- | :--- | :--- |
| `ms` | milliseconds | System uptime. |
| `roll` | degrees | Raw accelerometer-based roll (unfiltered). |
| `pitch` | degrees | Raw accelerometer-based pitch (unfiltered). |
| `gx, gy` | deg/s | Raw gyro rates for X and Y axes. |
| `cmd` | degrees | The current servo command (used to correlate lag). |

## How to use the SD Files
1.  **Via fetch_logs.py (Recommended):**
    *   Connect the FC via USB and run `python test/fetch_logs.py`.
2.  **Via Serial Monitor (Manual):**
    *   Open the Serial Monitor (115200 baud).
    *   Type **`r`** to dump the current log file text.
3.  **Via SD Card (Direct):**
    *   Plug the microSD into your PC using an adapter.

## Things to Know
*   **Logging Frequency**: `gliding_pid_sd` (100Hz), `gliding_pid_buffered` (500Hz).
*   **Buffer Management**: The buffered version flushes to the SD card every 0.5 seconds to ensure control loop stability.
*   **Excel Formatting**: CSV files are best viewed in Excel or Google Sheets for automatic column alignment.
