# GPS Stuff

Test scripts for integrating GPS with the Arduino Nano RP2040 Connect.

## Hardware Setup
- **Power**: Using **3.3V** for the GPS module.
- **Data**: GPS TX/RX connect to the Nano's hardware Serial1 (Pins D0 and D1).

## Code Details

### 1. Basic Passthrough (`gps_basic_passthrough`)
- **Serial1 (Pins 0/1)**: Listens for data from the GPS module.
- **Serial (USB)**: Echoes data to the PC Serial Monitor.
- **Bi-directional**: Supports sending commands from the Serial Monitor back to the GPS.
- **Purpose**: Diagnostic tool to verify the hardware link and see raw NMEA sentences.

### 2. Advanced Parsing (`gps_parsing_test`)
- **Library**: Uses `TinyGPS++` to decode raw NMEA strings.
- **Telemetry**: 
  - Latitude/Longitude (6 decimal places)
  - Altitude (Meters)
  - Speed (**m/s**)
  - Fix Quality (Satellite count and HDOP)
- **Logic**: 
  - Feeds characters into the GPS object until a full sentence is encoded.
  - Updates the display only when a valid sentence is processed.
  - Includes a 5-second timeout check that warns if no data is coming across the serial line.

## Usage
- Start with the passthrough sketch to confirm the baud rate and hardware connection.
- Use the parsing test once raw data is confirmed for readable flight telemetry.
