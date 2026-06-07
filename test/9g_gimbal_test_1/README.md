# MAV 3-Axis Gimbal Test v1

This is the initial setup for the 3-axis servo gimbal using the Seeed XIAO SAMD21. It includes the Arduino control firmware and a local web dashboard for live telemetry and control.

### Hardware Setup
- **MCU:** Seeed Studio XIAO SAMD21
- **Servos:** 3x 9g plastic gear servos
- **Pins:** Yaw (D0), Pitch (D1), Roll (D2)
- **Power:** External 5V PSU (Standard 9g servos pull ~1.4A under load). 
- **Note:** Connect PSU Ground to XIAO Ground or signals won't work.

### How to Run
1. **Flash the Board:** Open `gimbal.ino` in Arduino IDE (or use arduino-cli) and upload to the XIAO.
2. **Start the UI:**
   ```bash
   cd ui
   python3 app.py
   ```
3. **Control:** Open `http://localhost:5000` in a browser.

### Features
- **Smooth Motion:** Uses exponential smoothing and `writeMicroseconds` to cut down on jerkiness.
- **Safety Range:** Clamped to 60-120 degrees in code to prevent the servos from stalling against the frame.
- **Auto Sweep:** "Lissajous" mode for testing/demoing all axes at once.
- **Relax Mode:** Use the "Relax" button in the UI to cut power to the motors if they start getting hot.
