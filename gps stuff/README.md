# GPS Stuff

Code for testing the GPS module with the Arduino Nano RP2040 Connect.

## Wiring
- GPS TX -> Nano D0 (RX1)
- GPS RX -> Nano D1 (TX1)
- VCC -> 3.3V or 5V (Check module)
- GND -> GND

## Files
- **gps_basic_passthrough**: Just echoes raw NMEA sentences to the Serial Monitor. Good for checking if the wiring works.
- **gps_parsing_test**: Uses TinyGPS++ to get actual Lat/Lon, Alt, and Speed in **m/s**.

## Notes
- Most modules default to 9600 baud.
- You usually need to be near a window or outside to get a satellite fix (Sats > 3).
- HDOP under 2.0 is ideal for good accuracy.
