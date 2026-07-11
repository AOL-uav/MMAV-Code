# Rotational Mode (Philhower Core Port)

This directory contains otational_mode_test_phil.ino, which is a complete port of otational_mode_usb_pull.ino from the Mbed OS core to the Earle Philhower RP2040 core.

## Why was this ported?
The original firmware stopped logging to the SD card spontaneously. Debugging on the Mbed OS core (v4.6.0) revealed that SD.begin() was failing due to SPI initialization regressions in the Mbed core for the Nano RP2040 Connect. 

To resolve the software side of things, we successfully migrated the codebase to the more stable **Earle Philhower** dual-core architecture. This involved:
- Replacing tos::Queue and tos::Thread with the native RP2040 multicore API (setup1() / loop1()) and a thread-safe ring buffer (mutex_t).
- Configuring SdFat with SdSpiConfig tailored to the physical SPI pins (D10=CS, D11=MOSI, D12=MISO, D13=SCK).
- Adjusting the IMU.begin() initialization to explicitly set the I2C pins (SDA=12, SCL=13) to remain compatible with the V2 hardware.

## Results & Next Steps
Despite a fully successful compilation and flash of this ported code, the SD card still fails to initialize (# ERROR: SD not ready or no file.). A subsequent minimal SD card test also failed to initialize the SD card.

Given that this issue spontaneously occurred across **two different boards** and **three different SD cards**, and persists across **two entirely different core architectures** (Mbed OS and Philhower), it is highly likely that there is a **hardware issue** or **electrical fault**. 

### Hardware Debugging Checklist for the Lab:
1. **Check 3.3V Logic:** Ensure the SD card module is receiving a stable 3.3V. Some SD card modules with built-in regulators require 5V on the VCC pin to output 3.3V correctly, or they might brownout if powered from the Nano's 3.3V rail.
2. **Check Continuity:** Verify continuity from the physical pins (D10-D13) on the Nano to the SD card module's pins.
3. **Check formatting:** Reformat the SD cards using the official SD Association Formatter tool (FAT32).
4. **Try a slower SPI speed:** The code is currently set to SD_SCK_MHZ(12). Try lowering this to SD_SCK_MHZ(4) or even less in setup1() if wiring capacitance is high.
