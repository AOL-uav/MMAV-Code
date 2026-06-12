# 25kg 2-Axis Servo Gimbal Controller

A web-based interface and Arduino controller for a high-torque (25kg) servo gimbal system.

## Project Structure
- `gimbal/gimbal.ino`: Arduino firmware for servo control.
- `app.py`: Flask-based Python server for serial bridging and web UI.
- `templates/index.html`: Web interface with sliders and telemetry.

## Hardware Setup
- **Microcontroller**: Arduino (or compatible).
- **Servos**: 2x 25kg High-Torque Servos.
- **Connections**:
  - Yaw Servo: Signal Pin **0**
  - Pitch Servo: Signal Pin **1**
  - Baud Rate: **115200**
  - Power: External battery/power supply (Batt) required for 25kg servos.

## Quick Start

### 1. Flash Arduino
1. Open `gimbal/gimbal.ino` in Arduino IDE.
2. Select your board and port.
3. Click **Upload**.

### 2. Start the Python Server
1. Navigate to the project directory:
   ```bash
   cd "/home/kaikeller/Purdue/AOL/MAV/test/gimbal 2-axis 25kg servos"
   ```
2. Run the application:
   ```bash
   python3 app.py
   ```
   The server will automatically detect the serial port (e.g., `/dev/ttyACM0`).

### 3. Open Web UI
1. Open your web browser.
2. Go to: `http://localhost:5000` (or the IP of the machine running the server).

## Controls
- **Yaw/Pitch Sliders**: Manual adjustment within safety limits.
- **Movement Speed**: Controls the slew rate (smoothness) of the movement.
- **Center All**: Returns gimbal to the neutral 1500us position.
- **Start Auto Sweep**: Executes a pre-programmed sinusoidal movement pattern.
- **Relax Servos**: Detaches servos to save power and allow manual rotation.

## Safety Limits (Hardware Constrained)
- **Yaw**: 1167us - 1744us (60° - 112°)
- **Pitch**: 1000us - 2000us (45° - 135°)
- **Slew Rate**: Prevents rapid jerks that could damage the high-torque servos.
